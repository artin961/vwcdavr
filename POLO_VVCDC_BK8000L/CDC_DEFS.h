// Low period of PWTX line:
// 0: ~650us
// 1: ~1.77ms
// S: ~4.57ms

// one tick is 0.5µs
#define STARTTHRESHOLD  6400            // greater than this signifies START bit
#define HIGHTHRESHOLD   2496       // greater than this signifies 1 bit.
#define LOWTHRESHOLD     512        // greater than this signifies 0 bit.
#define PKTSIZE          -32            // command packets are 32 bits long.

// do not refresh head unit faster than 5.5ms (currently not implemented)
// 5.24s slow refresh rate when head unit in FM/AM/Tape mode (not implemented)

//#define REFRESH_PERIOD   50        // default to refresh head unit every 50.0ms
#define SECONDWAIT      -20         // wait 20 * 50ms to get 1 second (50ms*20 = 1000ms = 1s)
#define POWERIDENTWAIT  -10         // wait 10 * 1s to 10 seconds between VWCDPICx.x display
#define SCANWAIT        -100             // wait 100 * 50ms to get 5sec (50ms*100 = 5000ms = 5s)
#define _10MS           156
#define _50MS            500
#define _700US            7

#define TX_BUFFER_END   12
#define CAP_BUFFER_END  24

#define VER_MAJOR       '1'
#define VER_MINOR       '0'
#define VER_PATCHLEVEL  'b'

#define RADIO_COMMAND      PB0 //ICP1
#define RADIO_COMMAND_DDR  DDRB
#define RADIO_COMMAND_PORT  PORTB
#define RADIO_COMMAND_PIN PINB
#define RADIO_CLOCK        PD4//PB5
#define RADIO_CLOCK_DDR    DDRD//DDRB
#define RADIO_CLOCK_PORT  PORTD//PORTB
#define RADIO_DATA         PC3//PB3
#define RADIO_DATA_DDR     DDRC//DDRB
#define RADIO_DATA_PORT  PORTC//PORTB
//#define RADIO_ACC 3 // PD3 = INT1

// Command Codes
// -------------
//
// First byte is always 53
// Second byte is always 2C
// Third and Fourth bytes always add up to FF
// All command codes seem to be a multiple of 4.
//
//
// 53 2C 0C F3 CD 1
// 53 2C 10 EF DISABLE
// 53 2C 14 EB Change CD (ignored)
// 53 2C 18 E7 Previous CD (only on Audi Concert <)
// 53 2C 2C D3 CD 5 (first packet)
// 53 2C 38 C7 Change CD/Next CD (aka MINQUIRY)
// 53 2C 4C B3 CD 3 (first packet)
// 53 2C 58 A7 Seek Back Pressed
// 53 2C 60 9F Mix 1
// 53 2C 68 97 Up on Mk3 premium (Adam Yellen)
// 53 2C 78 87 Dn
// 53 2C 8C 73 CD 2 (first packet)
// 53 2C A0 5F Scan
// 53 2C A4 5B something to do with power on (Audi Concert)
// 53 2C A8 57 Dn on Mk3 premium (Adam Yellen <adam@yellen.com>)
// 53 2C AC 53 CD 6 (first packet)
// 53 2C CC 33 CD 4 (first packet)
// 53 2C D8 27 Seek Forward Pressed
// 53 2C E0 1F Mix 6
// 53 2C E4 1B ENABLE
// 53 2C F8 07 Up


//#define  Do_PLAY           0x08  // mix button held down (RCD300 head unit only)
#define  Do_PLAY           0x03  // mix button held down (RCD300 head unit only)
#define  Do_LOADCD    0x01  // not used
#define  Do_ENABLE_MK      0x08  // mk concert1
#define  Do_CD1            0x0C  // CD 1
#define  Do_DISABLE        0x10  // DISABLE
#define  Do_CHANGECD       0x14  // Change CD (changer ignores & no ACK)
#define  Do_PREVCD         0x18  // PREVIOUS CD (Audi Concert head unit)
#define  Do_CD5            0x2C  // CD 5
#define  Do_TP             0x30  // TP info (TP button pressed)
#define  Do_SEEKFORWARD_MK 0x38  // mk concert 1 LOAD CD (aka MINQUIRY).
// Also means "Next CD" if no CD button pressed
#define  Do_CD3            0x4C  // CD 3
#define  Do_SEEKBACK       0x58  // SEEK BACK
#define  Do_MIX_CD         0x60  // MIX 1 (mix tracks within one disc)
#define  Do_UP_MK3         0x68  // UP (Mk3 head unit)
#define  Do_DOWN           0x78  // DOWN
#define  Do_CD2            0x8C  // CD 2
#define  Do_SCAN           0xA0  // SCAN
#define  Do_UNKNOWNCMD     0xA4  // power on CD mode?? (Audi Concert head unit)
#define  Do_DOWN_MK3       0xA8  // DOWN (Mk3 head unit only)
#define  Do_CD6            0xAC  // CD 6
#define  Do_CD4            0xCC  // CD 4
#define  Do_SEEKFORWARD    0xD8  // Seek Forward
#define  Do_MIX            0xE0  // MIX 6 (mix tracks across all discs)
#define  Do_ENABLE         0xE4  // ENABLE
#define  Do_UP             0xF8  // UP

enum CDC_STATES
{
  StateIdle,
  StateIdleThenPlay,
  StateInitPlay,
  StateInitPlayEnd,
  StateInitPlayAnnounceCD,
  StatePlayLeadIn,
  StatePlayLeadInEnd,
  StatePlayLeadInAnnounceCD,
  StateTrackLeadIn,
  StatePlay,
  StateTP
};

uint8_t sendreg;
uint8_t sendbitcount; // Used in SendByte routine

uint8_t disc = 6;
uint8_t track = 1;
uint8_t minute = 0;
uint8_t second = 0;
uint8_t leds = 0;
// 'mix' and 'scan' flags specify whether we want to light up the MIX light
// or SCAN display on the head unit.
uint8_t mix;
uint8_t scan;
uint8_t playing;

uint8_t cd_button = 0;
uint8_t mix_button = 0;
// The ISR will set 'dataerr' flag if it detected a framing error
// or 'overflow' if it overflowed the capture buffer queue.
uint8_t overflow;
uint8_t dataerr;

uint8_t scanptr; // pointer to command byte to inspect next
uint8_t fsr;
uint8_t scanbyte; // most recently retrieved command byte
uint8_t cmdcode; // command code received from head unit (byte 3)
// these storage bytes are for the ISR to save process state so that it
// doesn't adversely affect the main loop code. you must -not- use these
// variables for anything else, as the ISR -will- corrupt them.

uint8_t intwsave;
uint8_t intstatussave;
uint8_t intfsrsave;

// the 'capbusy' flag will be set when the ISR is busy capturing a command
// packet from the head unit. The TMR0 ISR will clear it once the recieve
// timeout has been exceeded or the PWTX capture ISR will clear it once
// 32 bits have been captured (command packets are supposed to be 32 bits
// long only).

uint8_t capbusy;



#ifdef DUMPMODE
// The ISR will set 'startbit' flag if it detected a start bit.
uint8_t startbit;
#endif



uint16_t captime; // timer count of low pulse (temp)
int8_t capbit; // bits left to capture for this byte
int8_t capbitpacket; // bits left to capture for the entire packet
uint8_t capptr; // pointer to packet capture buffer loc

uint8_t BIDIstate; // pointer to the current state handler routine
int8_t BIDIcount; // counts how long to stay in current state
uint8_t ACKcount; // number of ACK bits to set.
uint8_t discload; // next disc number to load

int8_t poweridentcount; // counts down until we should send VWCDPICx.x
uint8_t secondcount; // counts down until one second has passed
int8_t scancount; // used to count down displaying SCAN mode

uint8_t txinptr;
uint8_t txoutptr;
uint8_t display_byte_buffer_mau8[8]; // holds display bytes sent to the head unit
//uint8_t const *txbuffer[TX_BUFFER_END]; // 39-26-1 = 12 serial output strings queue
uint8_t capbuffer[CAP_BUFFER_END]; // 64-39-1 = 24 bytes for head unit command
unsigned long autoOffTmr = 0;
uint8_t callStateTmr = 0;
bool flag_cd6 = false;
uint8_t inreset = 0;
/* -- Modul Global Function Prototypes ------------------------------------- */

static void ScanCommandBytes(void);
static void DumpFullCommand(void);
static void DecodeCommand(void);
static uint8_t GetCaptureByte(void);
static void SetStateIdle(void);
static void SetStatePlay(void);
static void SetStateInitPlay(void);
static void SetStatePlayLeadIn(void);
static void SetStateTrackLeadIn(void);
static void SendDisplayBytes(void);
static void SendDisplayBytesNoCD(void);
static void SendDisplayBytesInitCD(void);
static void SendFrameByte(uint8_t byte_u8);
static void SendByte(uint8_t byte_u8);
//static void EnqueueString(const uint8_t *addr PROGMEM);
//static void EnqueueHex(uint8_t hexbyte_u8);
static void ResetTime(void);
static void SetStateIdleThenPlay(void);
static void SetStateTP(void);
static void SendStateIdle(void);
static void SendStatePlayLeadInEnd(void);
static void SendPacket(void);
static void SendStateInitPlayEnd(void);
static void SendStateInitPlayAnnounceCD(void);
static void SendStatePlayLeadInAnnounceCD(void);
static void SendStateTP(void);
//static void printstr_p(const char *s);
static uint8_t cdButtonPushed(uint8_t cdnumber);
#define TRUE 1
#define FALSE 0

/* -- Implementation Functions --------------------------------------------- */

//-----------------------------------------------------------------------------
/*!
  \brief     Init_VWCDC
  initialization for cdc protocol
  \author     Koelling
  \date       26.09.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

void Init_VWCDC(void)
{
  cli();
  TIMSK0 = 0x00; //on arduino timer0 is used for millis(), we change prescaler, but also need to disable overflow interrupt
  RADIO_CLOCK_DDR |= _BV(RADIO_CLOCK);
  RADIO_DATA_DDR  |= _BV(RADIO_DATA);
  RADIO_COMMAND_DDR &= ~_BV(RADIO_COMMAND); // input capture as input
  RADIO_COMMAND_PORT |= _BV(RADIO_COMMAND); // enable pull up

  //Timer1 init
  //Used for timing the incoming commands from headunit
  TCCR1A = 0x00; // Normal port operation, OC1A/OC1B/OC1C disconnected
  TCCR1B = _BV(ICNC1); // noise canceler, int on falling edge
  TCCR1B |= _BV(CS11); // prescaler = 8 -> 1 timer clock tick is 0.5µs long
  TIFR1 |= _BV(ICF1); // clear pending interrupt
  TIMSK1 |= _BV(ICIE1); // enable input capture interrupt on timer1

  //Timer 2 Init
  //Timer 2 used to time the intervals between package bytes
  OCR2A = 175; // 4µs x 175 = 700µs
  TCCR2A = _BV(WGM21); // Timer2 in CTC Mode
  TCCR2B = _BV(CS22); // prescaler = 64 -> 1 timer clock tick is 4us long
  TIMSK2 |= _BV(OCIE2A);

  capptr = 0; // indirect pointer to capture buffer
  scanptr = 0;
  capbit = -8;
  txinptr = 0; // queue pointers
  txoutptr = 0;

  capbusy = FALSE; // reset flags
  mix = FALSE;
  scan = FALSE;
  playing = FALSE;
  overflow = FALSE;
  dataerr = FALSE;

#ifdef DUMPMODE
  startbit = FALSE;
#endif
  ACKcount = 0;

  // these values can be set depending on the state of mp3
  // it has to be evaluated wether CD number can be grater than 6
  disc = 0x46; // CD 1
  track = 0x01; // track 1

  poweridentcount = POWERIDENTWAIT;

  ResetTime();
  SetStateIdle(); //start at idle!!! no idle then play!! but SetStateIdleThenPlay exist cose here was "start idle then play" maybe it will work better, should checkit our

  //Timer 0 init
  //Timer 0 used to time the interval between each update
  TCCR0A = (1 << WGM01); //Set the CTC mode
  OCR0A = 0xF9; //Value for ORC0A for 1ms

  TIMSK0 |= (1 << OCIE0A); //Set the interrupt request
  sei(); //Enable interrupt

  TCCR0B |= (1 << CS01); //Set the prescale 1/64 clock
  TCCR0B |= (1 << CS00);
  /*
    #else
    OCR0A = _10MS; // 10ms Intervall
    TCCR0A = 0x00; // Normal port operation, OC0 disconnected
    TCCR0A |= _BV(WGM01); // CTC mode
    TCCR0B |= _BV(CS00) | _BV(CS02); // prescaler = 1024 -> 1 timer clock tick is 64us long
    TIMSK0 |= _BV(OCIE0A); // enable output compare interrupt on timer0
    #endif*/

  SendPacket(); // force first display update packet
  sei();
  //    SREG |= 0x80;   // enable interrupts
}

//-----------------------------------------------------------------------------
/*!
  \brief     CDC_Protocol(void)
  cyclic called main program for cdc protocol (50ms?)
  \author     Koelling
  \date       26.09.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

void CDC_Protocol(void) {
  uint8_t decimal_adjust_u8;

  if (millisM - CDCPollTime > 50)
  {
    CDCPollTime = millisM;
    SendPacket();
    scancount++;
    if (scancount == 0)
    {
      scancount = SCANWAIT;
      scan = FALSE; // turn off scan display
      leds &= 0xEF;
    }
    secondcount++;
    if (secondcount == 0)
    {
      secondcount = SECONDWAIT;
      poweridentcount++;

      if (poweridentcount == 0)
      {
        poweridentcount = POWERIDENTWAIT;
      }

      second++; // increment the time display
      decimal_adjust_u8 = second & 0x0F; // skip past hexidecimal codes
      if (decimal_adjust_u8 == 0x0A) // are with at xA?
      {
        second += 6; // yes, add 6 and we'll be at x0 instead
      }

      if (second == 0x60)
      {
        second = 0;
        minute++;
        decimal_adjust_u8 = minute & 0x0F; // skip past hexidecimal codes
        if (decimal_adjust_u8 == 0x0A) // are with at xA?
        {
          minute += 6; // yes, add 6 and we'll be at x0 instead
        }

        if (minute == 0xA0) // have we gone beyond 99 minutes?
        {
          minute = 0;
        }
      }
    }
  }

  if (overflow == TRUE) // has the command receive code detected
  { // an overflow error?
    overflow = FALSE; // clear error flag
  }

  if (dataerr == TRUE) // has the command receive code detected
  { // a framing type data error?
    dataerr = FALSE; // clear error flag
  }

#ifndef DUMPMODE
  ScanCommandBytes();
#else

  if (startbit == TRUE) // have we just recieved a start bit?
  {
    startbit = FALSE;
  }

  fsr = scanptr;

  while (GetCaptureByte() == TRUE)
  {
    scanptr = fsr;
  }

#endif
}

//-----------------------------------------------------------------------------
/*!
  \brief    void DecodeCommand(void)
  decode cmdcode and do required actions
  ;--------------------------------------------------------------------------
  ; Button Push Packets
  ;--------------------------------------------------------------------------
  ; 532C609F Mix 1
  ; 532CE01F Mix 6
  ; 532CA05F Scan
  ;     Note: Blaupunkt Gamma V head unit will continue to send scan key code
  ;       unless display is switched into scan mode.
  ;       (reported by tony.gilbert@orange.co.uk)
  ; 532C10EF Head Unit mode change. Emitted at power up, power down, and
  ;        any mode change. (disable playing)
  ; 532C58A7 Seek Back Pressed
  ; 532CD827 Seek Forward Pressed
  ; 532C7887 Dn
  ; 532CA857 Dn on Mk3 premium (Adam Yellen <adam@yellen.com>)
  ; 532CF807 Up
  ; 532C6897 Up on Mk3 premium (Adam Yellen)
  ; 532C38C7 CD Change (third packet)
  ; 532CE41B Seek Forward Released (enable playing)
  ; 532CE41B Seek Back Released (enable playing)
  ; 532CE41B CD Mode selected. Emitted at power up (if starting in CD
  ;            mode), change to CD mode. (enable playing)
  ; 532C14EB CD Change (second packet)
  ; 532C0CF3 CD 1 (first packet)
  ; 532C8C73 CD 2 (first packet)
  ; 532C4CB3 CD 3 (first packet)
  ; 532CCC33 CD 4 (first packet)
  ; 532C2CD3 CD 5 (first packet)
  ; 532CAC53 CD 6 (first packet)
  ;
  ; Monsoon State Changes:
  ; 532CE41B enable playing (transition to State 2)
  ; 532C38C7 disc loaded inquiry (transition to State 5)
  ; 532C10EF disable playing (transition to State 1)
  ;--------------------------------------------------------------------------
  \author     Koelling
  \date       05.10.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

static void DecodeCommand(void) {
  uint8_t decimal_adjust_u8 = 0;
  switch (cmdcode) {
    case Do_CHANGECD:
      // Head unit seems to send this after each CDx number change
      // but the CD Changer seems to completely ignore (doesn't even ACK it).
      ACKcount = 0; // do not ack this command
      break;

    case Do_ENABLE:
    case Do_ENABLE_MK://AFTER RELEASING FORWARD AND FAST REWIND BUTTONS

      mix = FALSE;
      if (playing == FALSE)
      {
        // Serial.println("DO_INIT");///DEBUGGGGG
        SetStateInitPlay(); // skip this if already playing
        //              if (BT.PowerState == BT.Off || BT.PowerState == BT.ShutdownInProgress )
        //              {
        //                BT.resetModule();
        //              }
        track++;
      }
      //TODO IF IDLE SET PLAY AUTO
      break;


    case Do_LOADCD:
      if (playing == TRUE)
      {
        SetStateInitPlay(); // skip this if we're in idle mode
      }
      ResetTime();
      break;


    case Do_DISABLE:
      //Serial.println("disss");
      if (playing == TRUE)
        SetStateIdle(); // skip this if we're already in idle mode
      disc = 0x41; // set back to CD 1
      ResetTime();
      if (blt.MusicState == Playing)//IF CURENTLY PLAYING STOP IT
        BTmusicStop();
      break;

    case Do_SEEKBACK:
    case Do_PREVCD:
      ResetTime();
      disc--;
      if ((disc & 0x0F) == 0)
      {
        disc = 0x46; // set back to CD 6
      }
      break;


    case Do_SEEKFORWARD:
    case Do_SEEKFORWARD_MK:
      ResetTime();
      if (cd_button == FALSE) // mk don't increment when previous command was a cd button
      {
        disc++;
        if (disc > 0x46)
        {
          disc = 0x41;
        }
        // Going beyond CD9 displays hex codes on premium head unit.
        // Examples: "CD A"
        //           "CD B"
        //           "CD C" etc...
        //
        // However, going beyond CD6 mutes audio on monsoon head unit, so we
        // definitely don't want to do that.
      }
      else
      {
        cd_button = FALSE; // mk clear cd button flag
      }
      break;


    case Do_MIX:
    case Do_MIX_CD:
      mix_button = 1;
      mix = 1;
      if (!blt.pairingWorkaround)
        BTairingInit();
      else
        BTpairingExit();
      break;
    case Do_SCAN:
      scancount = SCANWAIT;
      if (scan == FALSE)
      {
        scan = TRUE;
      }
      else
      {
        scan = FALSE;
      }
      break;


    case Do_PLAY:
      if (blt.MusicState != Playing)//IF CURENTLY NOT PLAINT START IT
        BTmusicTogglePlayPause();
      //Serial.println("DO_PLAY");
      break;

    case Do_CD1: //CD 1 BUTTON IF CALL ANSWER
      cd_button = TRUE; // mk store cd button pressed
      disc = 0x41; // set CD 1
      switch (blt.CallState) {
        case (IncomingCall)://IF INCOMMING CALL ANSWER
          BTcallANSWER();
          break;
        case (OutgoingCall): //IGNORE FOR OUTGOING CALL
          break;
        case (CallInProgress): //IGNORE FOR ACTIVE CALL
          break;
        default:
          BTmusicTogglePlayPause();
      }
      break;

    case Do_CD2:////BUTTON CD 2 if in call hangup if incomming reject or stop music if pressed
      cd_button = TRUE; // mk store cd button pressed
      disc = 0x42; // set CD 2
      switch (blt.CallState) {
        case (IncomingCall)://IF INCOMMING CALL REJECT
          BTcallReject();
          break;
        case (OutgoingCall):
        case (CallInProgress): //IF CALL IN PROGRESS REJECT
          BTcallHangUp();
          break;
        default:
          BTmusicStop();
      }
      break;


    case Do_CD3: //PLAY POUSE IF THIS BUTTON
      cd_button = TRUE; // mk store cd button pressed
      disc = 0x43; // set CD 3
      BTmusicTogglePlayPause();
      break;

    case Do_CD4://NEXT TRACK
      cd_button = TRUE; // mk store cd button pressed
      disc = 0x44; // set CD 4
      BTmusicPreviousTrack();
      break;

    case Do_CD5://PREVIOUS TRACK
      cd_button = TRUE; // mk store cd button pressed
      disc = 0x45; // set CD 5
      BTmusicNextTrack();
      break;

    case Do_CD6:
      cd_button = TRUE; // mk store cd button pressed
      disc = 0x46; // set CD 6
      break;


    case Do_UP:
    case Do_UP_MK3:
      if (playing == TRUE) // skip track lead-in if not in play mode
      {
        SetStateTrackLeadIn();
      }
      ResetTime();
      track++;
      decimal_adjust_u8 = track & 0x0F; // skip past hexidecimal codes
      if (decimal_adjust_u8 == 0x0A) // are with at xA?
      {
        track += 6; // yes, add 6 and we'll be at x0 instead
      }
      if (track == 0xA0) // have we gone beyond Track 99?
      { // yes, rollover to Track 01 so that jog wheels
        track = 1; // can continue rolling (Audi Concert II)
      }
      BTmusicNextTrack();
      break;

    case Do_DOWN:
    case Do_DOWN_MK3:
      if (playing == TRUE) // skip track lead-in if not in play mode
      {
        SetStateTrackLeadIn();
      }
      ResetTime();
      decimal_adjust_u8 = track & 0x0F; // skip past hexidecimal codes
      if (decimal_adjust_u8 == 0) // are we at x0?
      {
        track -= 6; // yes, subtract 6 and we'll be at x9 instead
      }
      track--;
      if (track == 0) // have we gone below Track 1?
      { // yes, rollover to Track 99 so that jog wheels
        track = 0x99; // can continue rolling (Audi Concert II)
      }
      BTmusicPreviousTrack();
      break;




    case Do_TP:
      if (playing == TRUE) {
        SetStateTP();

      } else {
        SetStateInitPlay();
      }
      break;

    default:
      //      /* if execution reaches here, we have verified that we got
      //        a valid command packet, but the command code received is not
      //        one that we understand.
      //
      //        Dump the unknown command code for the user to view.
      //
      break;
  }
  if (disc != 0x46 && !flag_cd6)
  {
    flag_cd6 = true;
    //Serial.println("R");
  }
}

/*!
  \brief    void ScanCommandBytes(void)
  ScanCommandBytes - Looks in the command receive buffer and tries
  to identify valid command codes.
  \author     Koelling
  \date       05.10.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

static void ScanCommandBytes(void)
{
  fsr = scanptr;

FirstByteLoop:
  //printstr_p(PSTR("1"),DEBUG);

  if (GetCaptureByte() == FALSE)

  {

    return;

  }



FirstByteTest:

  //printstr_p(PSTR("2"),DEBUG);

  if (scanbyte == 0x53)

  {

    goto SecondByte;

  }

  // this byte doesn't match the beginning of a normal command packet,

  scanptr = fsr; // save scanptr, won't look at this byte again

  goto FirstByteLoop;



SecondByte:

  //printstr_p(PSTR("3"),DEBUG);

  if (GetCaptureByte() == FALSE)

  {

    return;

  }

  if (scanbyte == 0x2C) // verify that byte 2 is 0x2C)

  {

    goto ThirdByte;

  }

  // the first byte was a match, but the second byte failed.

  // dump first byte and then see if this one is the real first byte.

  goto FirstByteTest;



ThirdByte:

  //printstr_p(PSTR("4"),DEBUG);

  if (GetCaptureByte() == FALSE)

  {

    return;

  }

  cmdcode = scanbyte; // save command code for later use.



  //FourthByte:

  //printstr_p(PSTR("5"),DEBUG);

  if (GetCaptureByte() == FALSE)

  {

    return;

  }

  // if execution reaches here, we have already verified that

  // bytes 1 and 2 are valid for a command packet.



  // verify that (Byte 3 + Byte 4) = 0xFF

  if ((cmdcode + scanbyte) == 0xFF)

  {

    //printstr_p(PSTR("6"),DEBUG);



    if ((cmdcode & 0x03) == 0) // verify that Byte 3 is a multiple of 4

    {

      //printstr_p(PSTR("7"),DEBUG);



      ACKcount = -4; // acknowledge command

      scanptr = fsr; // save scanptr, won't look at this byte again



      // Now, let's jump to the section of code that handles the

      // command we just received.



      DecodeCommand();

      //printstr_p(PSTR("\n"),DEBUG);



    }

    else

    {

      DumpFullCommand(); // ABORT: dump invalid packet for display

    }

  }

  else

  {

    DumpFullCommand(); // ABORT: dump invalid packet for display

  }

}

//-----------------------------------------------------------------------------
/*!
  \brief    void DumpFullCommand(void)
  dump all received command bytes
  \author     Koelling
  \date       06.10.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

static void DumpFullCommand(void)

{

  fsr = scanptr; // restart back at the beginning of the packet

  if (GetCaptureByte() == TRUE) // send byte 1

  {

    //EnqueueHex(scanbyte);

  }

  if (GetCaptureByte() == TRUE) // send byte 2

  {

    //EnqueueHex(scanbyte);

  }

  if (GetCaptureByte() == TRUE) // send byte 3

  {

    //EnqueueHex(scanbyte);

  }

  if (GetCaptureByte() == TRUE) // send byte 4

  {

    //EnqueueHex(scanbyte);

  }

  //EnqueueString(sNEWLINE);

  scanptr = fsr; // save scanptr, won't look at this byte again

}

//-----------------------------------------------------------------------------
/*!
  \brief    uint8_t GetCaptureByte(void)
  checks wether a command byte is still in queue
  \author     Koelling
  \date       05.10.2007
  \param[in]  none
  \param[out] none
  \return     FALSE ->  no more bytes to collect
  TRUE  -> scanbyte contains next byte
*/
//-----------------------------------------------------------------------------

static uint8_t GetCaptureByte(void)

{

  uint8_t return_u8 = FALSE;

  // have we already caught up with capturer?

  if (fsr != capptr)

  {

    scanbyte = capbuffer[fsr]; // get a byte from the capture buffer

    fsr++;

    if (fsr == CAP_BUFFER_END) // have we overflowed the

    { // capture buffer?

      fsr = 0;

    } // yes, roll over to beginning

    return_u8 = TRUE;

  }

  return return_u8;

}

//-----------------------------------------------------
// Display Update Packets
//-----------------------------------------------------
//-----------------------------------------------------------------------------
/*!
  \brief    void SetStateIdle(void)
  Idle State
  74 BE FE FF FF FF 8F 7C
  74 BE FE FF FF FF 8F 7C
  ...
  \author     Koelling
  \date       06.10.2007
  \param[in]  none
  \param[out] none
  \return     none
*/
//-----------------------------------------------------------------------------

static void SetStateIdle(void)

{

  playing = FALSE;

  BIDIstate = StateIdle;

}
static void SetStateTP(void)
{
  playing = FALSE;
  BIDIstate = StateTP;
}

//-----------------------------------------------------------------------------
/*!
  \brief     SetStateIdleThenPlay(void)
  Real CD Changer doesn't really do this, but we're gonna do it to try
  and make sure we unmute the audio even if the user didn't connect
  the PW-TX pin properly.
  \author     Koelling
  \date       27.09.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

static void SetStateIdleThenPlay(void)

{

  playing = 0;

  BIDIstate = StateIdleThenPlay;

  BIDIcount = -20;

}

//-----------------------------------------------------------------------------
/*!
  \brief    void (void)
  set state to play mode
  \author     Koelling
  \date       06.10.2007
  \param[in]  none
  \param[out] none
  \return     none
*/
//-----------------------------------------------------------------------------

static void SetStatePlay(void)

{

  playing = TRUE;

  BIDIstate = StatePlay;

}

//-----------------------------------------------------------------------------
/*!
  \brief    void SetStateInitPlay(void)
  Initiate Playing
  \author     Koelling
  \date       06.10.2007
  \param[in]  none
  \param[out] none
  \return     none
*/
//-----------------------------------------------------------------------------

static void SetStateInitPlay(void)

{

  playing = TRUE;

  BIDIstate = StateInitPlay;

  discload = 0xD1; //0xFF - 0x2E

  BIDIcount = -24;

}

//-----------------------------------------------------------------------------
/*!
  \brief    void SetStatePlayLeadIn(void)
  34 BE FE FF FF FF AE 3C (play lead-in)
  34 2E ED DE AF B7 FF 3C
  34 BE FE FF FF FF AE 3C
  34 2E ED DE AF B7 FF 3C
  34 BE FE FF FF FF AE 3C
  34 2E ED DE AF B7 FF 3C
  34 BE FE FF FF FF AE 3C
  34 2E ED DE AF B7 FF 3C
  34 BE FE FF FF FF AE 3C
  \author     Koelling
  \date       06.10.2007
  \param[in]  none
  \param[out] none
  \return     none
*/
//-----------------------------------------------------------------------------

static void SetStatePlayLeadIn(void)

{

  playing = TRUE;

  BIDIstate = StatePlayLeadIn;

  BIDIcount = -10;

}





//-----------------------------------------------------------------------------
/*!
  \brief    void SetStateTrackLeadIn(void)
  34BEFEFFEEFFCF3C (playing)
  34BEFEFFEEFFCF3C
  14BEFDFFFFFFCF1C (ack)
  14BEFDFFFFFFAE1C (track lead in)
  14BEFDFFFFFFAE1C
  34BEFDFFFFFFAE3C
  34BEFDFFFFFFAE3C
  34BEFDFFFFFFAE3C
  34BEFDFFFFFFAE3C
  34BEFDFFFFFFAE3C
  34BEFDFFFFFFAE3C
  34BEFDFFFFFFAE3C
  34BEFDFFFFFFAE3C
  34BEFDFFFFFFAE3C
  34BEFDFFFFFFAE3C
  34BEFDFFFFFFCF3C (playing)
  34BEFDFFFFFFCF3C
  \author     Koelling
  \date       06.10.2007
  \param[in]  none
  \param[out] none
  \return     none
*/
//-----------------------------------------------------------------------------

static void SetStateTrackLeadIn(void)

{

  playing = TRUE;

  BIDIstate = StateTrackLeadIn;

  BIDIcount = -12;

}



// TODO: We might implement one more state machine for
// the CHANGECD/INQUIRY command. (mute byte goes 0x6F and 0xFF cd load
// while changer is busy motoring next CD into position). Then
// again, maybe we don't need to implement any busy states since
// we are instantly ready (no motoring here!).
// =========================================================================
// SEND DISPLAY UPDATE PACKETS
// =========================================================================
//-----------------------------------------------------------------------------
/*!
  \brief    void SendDisplayBytes(void)
  send display bytes to head unit
  \author     Koelling
  \date       06.10.2007
  \param[in]  none
  \param[out] none
  \return     none
*/
//-----------------------------------------------------------------------------

static void SendDisplayBytes(void)

{

  SendByte(disc); // disc display value

  SendDisplayBytesNoCD();

}

//-----------------------------------------------------------------------------
/*!
  \brief    void SendDisplayBytesNoCD(void)
  \author     Koelling
  \date       06.10.2007
  \param[in]  none
  \param[out] none
  \return     none
*/
//-----------------------------------------------------------------------------

static void SendDisplayBytesNoCD(void)

{

  uint8_t send_byte_u8 = 0;



  SendByte(track);

  SendByte(minute);

  SendByte(second);



  // D4 - scan on, mix on

  // D0 - scan on, mix off

  // 04 - scan off, mix on

  // 00 - scan off, mix off



  if (mix == TRUE) // mode (scan/mix)

  {

    send_byte_u8 |= 0x20; // turn on mix light

  }

  if (scan == TRUE)

  {

    send_byte_u8 |= 0x10; // turn on scan display //this probably cose mute for few microsec.

  }

  SendByte(send_byte_u8);

}

//-----------------------------------------------------------------------------
/*!
  \brief    void SendDisplayBytesInitCD(void)
  When sending an "init cd" packet, we need to send it the number of
  tracks and whatnot available on the CD. Required on Audi Concert II so
  that track up/dn buttons work.
  \author     Koelling
  \date       06.10.2007
  \param[in]  none
  \param[out] none
  \return     none
*/
//-----------------------------------------------------------------------------

static void SendDisplayBytesInitCD(void)

{

  SendByte(0x99); // number of tracks total (99)?

  SendByte(0x99); // total minutes?

  SendByte(0x59); // total seconds?

  SendByte(0x49);//0xFF - 0xB7 = 48, 53, 31, 25, and 37 seen from real CDC,

  // no idea what it really means.

}

//-----------------------------------------------------------------------------
/*!
  \brief    void SendFrameByte(uint8_t byte_u8)
  SendFrameByte - sends a framing byte to head unit (first and last bytes).
  If the ACK flag is set, we modify the send byte to send an
  acknowledgement.
  \author     Koelling
  \date       06.10.2007
  \param[in]  byte_u8 -> byte to send
  \param[out] none
  \return     none
*/
//-----------------------------------------------------------------------------

static void SendFrameByte(uint8_t byte_u8)

{

  if (ACKcount == 0)

  {

    SendByte(byte_u8);

  }

  else

  {

    byte_u8 |= 0x20; // flag acknowledgement

    ACKcount++;

    SendByte(byte_u8);

  }

}

//-----------------------------------------------------------------------------
/*!
  \brief    void SendFrameByte(uint8_t byte_u8)
  SendByte - sends a byte to head unit.
  \author     Koelling
  \date       06.10.2007
  \param[in]  byte_u8 -> byte to send
  \param[out] none
  \return     none
*/
//-----------------------------------------------------------------------------

static void SendByte(uint8_t byte_u8)

{

  static uint8_t display_byte_counter_u8 = 0;

  // wait for head unit to store sent byte

  // 335us didn't work so good on late 2003 wolfsburg double din,

  // so we now wait 700us instead.

  display_byte_buffer_mau8[display_byte_counter_u8] = byte_u8;

  display_byte_counter_u8++;

  if (display_byte_counter_u8 == 8)

  {

    display_byte_counter_u8 = 0;
    TCCR2B |= _BV(CS22); // prescaler = 64 -> 1 timer clock tick is 4us long
    //TIMSK2 |= _BV(OCIE2A); // enable output compare interrupt on timer2

  }

}

//-----------------------------------------------------------------------------
/*!
  \brief     void EnqueueString(const uint8_t addr PROGMEM)
  EnqueueString - Adds a new string pointer into the outgoing serial string
  queue.
  \author     Koelling
  \date       02.10.2007
  \param[in]  const uint8_t addr PROGMEM -> start address of string to display
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

//static void EnqueueString(const uint8_t *addr PROGMEM)
//
//{
//
//  txbuffer[txinptr] = addr;
//
//  txinptr++;
//
//  if (txinptr == TX_BUFFER_END)
//
//  {
//
//    txinptr = 0;
//
//  }
//
//}

//-----------------------------------------------------------------------------
/*!
  \brief     void EnqueueHex(uint8_t hexbyte_u8)
  The byte is converted to a two byte ASCII hexidecimal string
  \author     Koelling
  \date       05.10.2007
  \param[in]  uint8_t hexbyte -> hexbyte to display
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

//static void EnqueueHex(uint8_t hexbyte_u8)
//
//{
//
//  uint8_t nibble_u8;
//
//
//
//  nibble_u8 = hexbyte_u8 >> 4; // send high nibble first
//
//  nibble_u8 <<= 1; // multiply high nibble by 2
//
//  EnqueueString(&sHEX[nibble_u8]);
//
//
//
//  nibble_u8 = hexbyte_u8 & 0x0F; // prepare low nibble
//
//  nibble_u8 <<= 1; // multiply low nibble by 2
//
//  EnqueueString(&sHEX[nibble_u8]);
//
//}

//-----------------------------------------------------------------------------
/*!
  \brief     ResetTime(void)
  reset time information
  \author     Koelling
  \date       27.09.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

static void ResetTime(void)

{

  secondcount = SECONDWAIT;

  second = 0;

  minute = 0;

}

//-----------------------------------------------------------------------------
/*!
  \brief     void SendStateIdle(void)
  send data for idle state
  \author     Koelling
  \date       29.09.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

static void SendStateIdle(void)

{

  secondcount = SECONDWAIT; // stop display from ticking time

  SendFrameByte(0x8B);//FF - 0x74

  SendDisplayBytes();

  SendByte(0x70);//FF - 0x8F, mutes audio on Monsoon head units

  SendFrameByte(0x83);//FF - 0x7C

}

static void SendStateTP(void)
{ //B4 BE EF FE DB FF DF BC
  secondcount = SECONDWAIT; // stop display from ticking time
  SendFrameByte(0x4B);//FF - 0x4b
  SendDisplayBytes();
  SendByte(0x20);
  SendFrameByte(0x43);//FF - 0x7C
}

//-----------------------------------------------------------------------------
/*!
  \brief     SendStatePlayLeadInEnd(void)
  send data for state PlayLeadInEnd
  \author     Koelling
  \date       29.09.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

static void SendStatePlayLeadInEnd(void)
{

  SendFrameByte(0xC3);//FF - 0x3C

  BIDIcount++;

  if (BIDIcount == 0)

  {
    SetStatePlay();

  }

}

//-----------------------------------------------------------------------------
/*!
  \brief     SendStateInitPlayEnd(void)
  send data for state StateInitPlayEnd
  \author     Koelling
  \date       06.10.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

static void SendStateInitPlayEnd(void)

{

  SendFrameByte(0xC3);//FF - 0x3C

  BIDIcount++;

  if (BIDIcount == 0)

  {

    SetStatePlayLeadIn();

  }

}

//-----------------------------------------------------------------------------
/*!
  \brief     SendStateInitPlayAnnounceCD(void)
  send data for state StateInitPlayAnnounceCD
  \author     Koelling
  \date       06.10.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

static void SendStateInitPlayAnnounceCD(void)

{

  // 0xF6..0xF0: CD-ROM Loaded (seen on changer)

  // 0xE6..0xE0: CD-ROM Loaded. (made up)

  // 0x96..0x90: Slot Empty (seen on changer)

  // 0x86..0x80: Slot Empty (made up)

  // 0xD6..0xD0: AUDIO CD Loaded. (seen on changer)

  SendByte(discload);

  if (discload == 0xD6)

  {

    discload = 0xD1;

  }

  else

  {

    discload++;

  }

  SendDisplayBytesInitCD();

  SendByte(0x00);//0xFF - 0xFF

  SendStateInitPlayEnd();

}

//-----------------------------------------------------------------------------
/*!
  \brief     SendStatePlayLeadInAnnounceCD(void)
  send data for state StatePlayLeadInAnnounceCD
  \author     Koelling
  \date       06.10.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

static void SendStatePlayLeadInAnnounceCD(void)

{

  SendByte((disc & 0x0F) | 0xD0);

  SendDisplayBytesInitCD();

  SendByte(0x00);//0xFF - 0xFF

  SendStatePlayLeadInEnd();

}

//-----------------------------------------------------------------------------
/*!
  \brief     void SendPacket(void)
  depending on BIDIstate data packet will be sent
  \author     Koelling
  \date       27.09.2007
  \param[in]  none
  \param[out] none
  \return     void
*/
//-----------------------------------------------------------------------------

static void SendPacket(void)

{

  switch (BIDIstate) {
    case StateTP:
      SendStateTP();
      break;

    case StateIdle:
      SendStateIdle();
      break;

    case StateIdleThenPlay:
      BIDIcount++;

      if (BIDIcount == 0)
      {
        SetStateInitPlay();
        SendStateIdle();
      }
      else
      {
        SendStateIdle();
      }
      break;

    // 34 2E ED DE AF B7 FF 3C
    // 34 BE FE FF FF FF EF 3C
    // 34 2D EB BE AB AC FF 3C
    // 34 BE FE FF FF FF EF 3C
    // 34 2C EC CE AA CE FF 3C
    // 34 BE FE FF FF FF EF 3C
    // 34 2B EE EE B7 DA FF 3C
    // 34 BE FE FF FF FF EF 3C
    // 34 2A EB BE A6 C8 FF 3C
    // 34 BE FE FF FF FF EF 3C
    // 34 69 00 FF FF FF FF 3C
    // 34 BE FE FF FF FF EF 3C

    case StateInitPlay:
      secondcount = SECONDWAIT; // stop display from ticking time
      SendFrameByte(0xCB);//0xFF - 0x34

      if ((BIDIcount & 0x01) == 0)
      {
        SendStateInitPlayAnnounceCD();
        break;
      }
      SendDisplayBytes();
      SendByte(0x10);//0xFF - 0xEF
    //no break here!

    case StateInitPlayEnd:
      SendStateInitPlayEnd();
      break;

    case StateInitPlayAnnounceCD:
      SendStateInitPlayAnnounceCD();
      break;

    case StatePlayLeadIn:
      // 34 BE FE FF FF FF AE 3C (play lead-in)
      // 34 2E ED DE AF B7 FF 3C
      // 34 BE FE FF FF FF AE 3C
      // 34 2E ED DE AF B7 FF 3C
      // 34 BE FE FF FF FF AE 3C
      // 34 2E ED DE AF B7 FF 3C
      // 34 BE FE FF FF FF AE 3C
      // 34 2E ED DE AF B7 FF 3C
      // 34 BE FE FF FF FF AE 3C
      secondcount = SECONDWAIT; // stop display from ticking time
      SendFrameByte(0xCB);//0xFF - 0x34

      if ((BIDIcount & 0x01) == 0)
      {
        SendStatePlayLeadInAnnounceCD();
        break;
      }

      SendDisplayBytes();
      SendByte(0x51);//0xFF - 0xAE
    //no break here!

    case StatePlayLeadInEnd:
      SendStatePlayLeadInEnd();
      break;

    case StatePlayLeadInAnnounceCD:
      SendStatePlayLeadInAnnounceCD();
      break;

    case StateTrackLeadIn:
      secondcount = SECONDWAIT; // stop display from ticking time
      SendFrameByte(0xCB);//0xFF - 0x34
      SendDisplayBytes();
      SendByte(0x51);//0xFF - 0xAE
      SendFrameByte(0xC3);//0xFF - 0x3C
      BIDIcount++;
      if (BIDIcount == 0)
      {
        break;
      }
      SetStatePlay();
      break;

    case StatePlay:
      SendFrameByte(0xCB);//0xFF - 0x34
      SendDisplayBytes();
      SendByte(0x30);//0xFF - 0xCF
      SendFrameByte(0xC3);//FF - 0x3C
      break;
    default:
      break;
  }
}

static uint8_t cdButtonPushed(uint8_t cdnumber) {
  static uint8_t cd1pushed, cd2pushed, cd3pushed, cd4pushed, cd5pushed, cd6pushed = 0; //variable to cound how many times we pressed button for spectial actions
  switch (cdnumber) {
    case 1:
      cd2pushed = cd3pushed = cd4pushed = cd5pushed = cd6pushed;
      if (++cd1pushed == 6)
        cd1pushed = 0;
      return cd1pushed;
    case 2:
      cd1pushed = cd3pushed = cd4pushed = cd5pushed = cd6pushed;
      if (++cd2pushed == 6)
        cd2pushed = 0;
      return cd2pushed;

    case 3:
      cd1pushed = cd2pushed = cd4pushed = cd5pushed = cd6pushed;
      if (++cd3pushed == 6)
        cd3pushed = 0;
      return cd3pushed;

    case 4:
      cd1pushed = cd2pushed = cd3pushed = cd5pushed = cd6pushed;
      if (++cd4pushed == 6)
        cd4pushed = 0;
      return cd4pushed;

    case 5:
      cd1pushed = cd2pushed = cd3pushed = cd4pushed = cd6pushed;
      if (++cd5pushed == 6)
        cd5pushed = 0;
      return cd5pushed;

    case 6:
      cd1pushed = cd2pushed = cd3pushed = cd4pushed = cd5pushed;
      if (++cd6pushed == 6)
        cd6pushed = 0;
      //Serial.println(cd6pushed);
      return cd6pushed;
  }
  return 0;
}
