//#define DEBUGEEE
#include <avr/wdt.h>

volatile unsigned long millisM = 0; //GLOBAL MILLIS COUNTER
unsigned long bluetoothPollTime = 0;//TIMER FOR BLUETOOTH POLLING
unsigned long CDCPollTime = 0; //TIMER FOR CDC POLLING
unsigned long CD6Timer = 0; //TIMER FOR CD6 restore after annother pressed

#include "BLT_DEFS.h"
#include "CDC_DEFS.h"



int main() {
  wdt_reset();
  wdt_enable(WDTO_120MS);

  Init_VWCDC();

  bluetoothInit();        //BT.begin();
  bluetoothReset();       //BT.resetModule();/////////////ARTIN ADDED

  DDRB |= (1 << PB5);//PIN MODE 13 OUTPUT
  while (1)
  {
    wdt_reset();
    CDC_Protocol();
    bluetoothcheckResponse();
    if (millisM - bluetoothPollTime > BT_POLL_TIME) {
      bluetoothPollTime = millisM;
      bluetoothPoll();///SEND COMMAND IF CUED ELSE POLL NEXT STATUS
    }
    if (millisM - CD6Timer > 300)//cd 6 recovery
    {
      CD6Timer = millisM;

      if (disc != 0x46 && playing)
        disc = 0x46;
      mix = false;
      flag_cd6 = false;//RESET FLAG FOR RESETING TO CD6
      if (blt.pairingWorkaround)
        PORTB ^= (1 << PB5);
      else {
        if (blt.BTState == Connected)
          PORTB |= (1 << PB5);
        else
          PORTB &= ~(1 << PB5);
      }

    }
  }
}



ISR(TIMER0_COMPA_vect) {
  millisM++;
}





//-----------------------------------------------------------------------------
/*!
  \brief    ISR(TIMER2_COMPA_vect)
  Timer2 ensures 700µs timing between display package bytes
  Shift bytes out to head unit
  \author     Koelling
  \date       06.10.2007
  \param[in]  none
  \param[out] none
  \return     none
*/
//-----------------------------------------------------------------------------

ISR(TIMER2_COMPA_vect) {
  static uint8_t display_byte_counter_u8 = 0;
  uint8_t byte_u8;
  //TCCR2B &= ~_BV(CS20); //set to 0 already
  //TCCR2B &= ~_BV(CS21); //set to 0 already
  TCCR2B &= ~_BV(CS22); // stop Timer2, CS22 was set, prescaler 64
  //TCNT2 = 0; // clear Timer2
  //TIFR2 |= _BV(OCF2A); //no need to do this

  if (display_byte_counter_u8 < 8)
  {
    byte_u8 = display_byte_buffer_mau8[display_byte_counter_u8];

#ifdef DUMPMODE2
    Serial.print("|");
    Serial.print(byte_u8, HEX);
    Serial.print("|");
#endif

    for (sendbitcount = -8; sendbitcount != 0; sendbitcount++)
    {
      RADIO_CLOCK_PORT |= _BV(RADIO_CLOCK); // SCLK high
      _delay_loop_1(40);
      if ((byte_u8 & 0x80) == 0) // mask highest bit and test if set
      {
        RADIO_DATA_PORT |= _BV(RADIO_DATA); // DATA high
      }
      else
      {
        RADIO_DATA_PORT &= ~_BV(RADIO_DATA); // DATA low
      }
      byte_u8 <<= 1; // load the next bit
      RADIO_CLOCK_PORT &= ~_BV(RADIO_CLOCK); // SCLK low
      _delay_loop_1(40);
    }

    display_byte_counter_u8++;

    TCCR2B |= _BV(CS22); // prescaler = 64 -> 1 timer clock tick is 4us long
    //TCCR2B |= _BV(CS20);
  }
  else
  { //display_byte_counter_u8 is ==8
#ifdef DUMPMODE2
    Serial.println();
#endif
    display_byte_counter_u8 = 0;
    //TIMSK2 &= ~_BV(OCIE2A); // disable output compare interrupt on timer2
  }
}



//-----------------------------------------------------------------------------
/*!
  \brief     ISR(TIMER1_OVF_vect)
  timer1 overflow interrupt service routine for cdc protocol
  \author     Koelling
  \date       26.09.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

ISR(TIMER1_OVF_vect) {
  TIMSK1 &= ~_BV(TOIE1); // disable further timer 1 interrupts
  capbusy = FALSE; // set flag signifying packet capture done

  if (capbit > -8) // are we already capturing on a blank byte?
  {
    dataerr = TRUE;
    // Note: This should never happen on normal head unit sending 32 bit
    //        command strings with error free data.
    //
    // if the capture bits were not a complete 8 bits, we need to finish
    // rotating the bits upward so that the data is nicely formatted

    while (capbit != 0) // have we finished rotating all bits up?
    {
      capbuffer[capptr] <<= 1; // rotate in 0 bit
      capbit++;
    }

    capbit = -8;
    capptr++; // move to new capture byte

    if (capptr == CAP_BUFFER_END) // have we gone past the end of the
    { // capture buffer?
      capptr = 0; // yes, roll over to beginning
    }

    if (capptr == scanptr) // have we overflowed the capture queue?
    {
      overflow = TRUE; // yes, set error flag
    }
  }
}



//-----------------------------------------------------------------------------
/*!
  \brief     ISR(TIMER1_CAPT_vect)
  input capture interrupt service routine for cdc protocol
  \author     Koelling
  \date       26.09.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

ISR(TIMER1_CAPT_vect) {
  captime = ICR1; // save a copy of current TMR1 count
  // in case PWTXCaptureBit needs it
  TCNT1 = 0; // clear timer1

  if ((RADIO_COMMAND_PIN & _BV(RADIO_COMMAND)) == 0)
  {
    // We have interrupted at beginning of low pulse (falling edge)
    // Low pulse length must be timed to determine bit value
    TIFR1 |= _BV(TOV1); // clear timer1 overflow flag
    TIMSK1 |= _BV(TOIE1); // enable timer1 interrupt on overflow
    TCCR1B |= _BV(ICES1); // change input capture to rising edge
    TIFR1 |= _BV(ICF1); // clear input capture interrupt request flag
  }
  else
  {
    // We have interrupted at beginning of high pulse (rising edge)
    // High pulse length doesn't matter. We need to check out
    // captured low pulse width if we are capturing data at the moment

    capbusy = TRUE;
    TCCR1B &= ~_BV(ICES1); // change input capture to falling edge
    TIFR1 |= _BV(ICF1); // clear input capture interrupt request flag

    if (TIMSK1 & _BV(TOIE1)) // are we trying to capture data?
    {
      capbusy = TRUE;
      TIMSK1 &= ~_BV(TOIE1); // turn off capturing time for high pulse

      if (captime > STARTTHRESHOLD)
      { // yes, start bit
#ifdef DUMPMODE
        startbit = TRUE;
#endif
        capbitpacket = PKTSIZE;

        // don't store start bits, just frame around them
        if (capbit > -8) // are we already capturing on a blank byte?
        {
          dataerr = TRUE;
          // Note: This should never happen on normal head unit sending 32 bit
          //       command strings with error free data.
          //
          // if the capture bits were not a complete 8 bits, we need to finish
          // rotating the bits upward so that the data is nicely formatted

          while (capbit != 0) // have we finished rotating all bits up?
          {
            capbuffer[capptr] <<= 1; // rotate in 0 bit
            capbit++;
          }
          capbit = -8;
          capptr++; // move to new capture byte

          if (capptr == CAP_BUFFER_END) // have we gone past the end of the
          { // capture buffer?
            capptr = 0; // yes, roll over to beginning
          }

          if (capptr == scanptr) // have we overflowed the capture queue?
          {
            overflow = TRUE; // yes, set error flag
          }
        }
      }
      else
      { // no, just a regular data bit
        if (captime > LOWTHRESHOLD)
        { // yes, go ahead and store this data
          capbuffer[capptr] <<= 1; // nope

          if (captime > HIGHTHRESHOLD)
          {
            capbuffer[capptr] |= 1;
          }

          capbitpacket++;

          if (capbitpacket == 0)
          {
            // we've received PKTSIZE number of bits, so let's assume that we're done
            // capturing bits for now.
            capbusy = FALSE; // clear capture busy flag
          }

          capbit++;

          if (capbit == 0) // have we collected all 8 bits?
          { // yep, get ready to capture next 8 bits
            capbit = -8;
            capptr++; // move to new capture byte

            if (capptr == CAP_BUFFER_END) // have we gone past the end of the
            { // capture buffer?
              capptr = 0; // yes, roll over to beginning
            }

            if (capptr == scanptr) // have we overflowed the capture queue?
            {
              overflow = TRUE; // yes, set error flag
            }
          }
        }
      }
    }
  }
}
