//#define DEBUGEEE

#define MAX_BLT_COMMAND 17
#define BT_POLL_TIME 50
#define BT_BOOTUP_TIME 30 //xpolltimes
#define BT_RESET_TIME 4   //xpolltimes

enum BT_CMD {
  NOOP,
  SHUTDOWN,
  PAIR,
  EXIT_PAIRING,
  CON_LAST_DEVICE,


  CALL_ANSWER,
  CALL_REJECT,
  CALL_HANGUP,

  MUSIC_PLAY,
  MUSIC_STOP,
  MUSIC_NEXT,
  MUSIC_PREV,
  MUSIC_FF,
  MUSIC_FR,

  CHECK_CONN_STAT,
  CHECK_MUSIC_STAT,
  CHECK_HFP_STAT,
};

enum BT_STATES
{
  Playing, //MA
  Idle, //MB
  IncomingCall, //IR- or M2
  OutgoingCall, //PR- or M3
  CallInProgress, //M4
  Connected, // M1
  Disconnected, //M0
  On,
  Off,
  Pairing,
  ShutdownInProgress
};
struct BKCMD {
  BT_CMD cmd;
  char stringcmd[2];
};
struct BK8000L {
  BT_STATES BTState;//CONNECTION STATE
  BT_STATES CallState;//HSP STATE
  BT_STATES MusicState;//MUSIC STATE
  BT_STATES PowerState;//POWER STATE

  bool pairingWorkaround;
  String receivedSppData;
  String CallerID;
  uint8_t inreset;///COUNTER FOR RESET CHECK
  uint8_t bootup;///COUNTER FOR BOOTUP DELAY
  uint8_t lastReaponse;//last response was n pool times ago
  BT_CMD sendCMD;//COMMAND TO SEND FROM DEFINES
  BT_CMD lastCMD;//LAST COMMAND SENT, FROM DEFINES
};
static BK8000L blt;
static BKCMD bltcommands[MAX_BLT_COMMAND] = {
  {NOOP, "  "},
  {SHUTDOWN, "CP"},
  {PAIR, "CA"},
  {EXIT_PAIRING, "CB"},
  {CON_LAST_DEVICE, "CC"},

  {CALL_ANSWER, "CE"},
  {CALL_REJECT, "CF"},
  {CALL_HANGUP, "CG"},

  {MUSIC_PLAY, "MA"},
  {MUSIC_STOP, "MC"},
  {MUSIC_NEXT, "MD"},
  {MUSIC_PREV, "ME"},
  {MUSIC_FF, "MF"},
  {MUSIC_FR, "MH"},

  {CHECK_CONN_STAT, "MO"},
  {CHECK_MUSIC_STAT, "MV"},
  {CHECK_HFP_STAT, "MY"},
};



//#define DEBUGEEE
#define DEBUGSERIAL Serial

static int  bluetoothtimedRead(void);
static void bluetoothInit(void);
static void bluetoothReset(void);
static void bluetoothCheckReset(void);
static void bluetoothPoll(void);
static void BTsendCMD(BT_CMD cmd) ;
static String bluetoothreturnCallerID(String receivedString);
static void bluetoothcheckResponse(void);
static uint8_t bluetoothdecodeReceivedString(String receivedString) ;

static void BTairingInit(void);
static void BTpairingExit(void);
static void BTcallANSWER(void);
static void BTcallReject(void);
static void BTcallHangUp(void);
static void BTshutdownBT(void);
static void BTmusicTogglePlayPause(void);
static void BTmusicStop(void);
static void BTmusicNextTrack(void);
static void BTmusicPreviousTrack(void);
static void BTmusicFastForward(void);
static void BTmusicRewind(void);


int  bluetoothtimedRead() {
  int c;
  unsigned long startMillis = millisM;
  do {
    c = Serial.read();
    if (c >= 0) return c;
  } while (millisM - startMillis < 3);
  return -1;     // -1 indicates timeout
}
void bluetoothInit() {
  blt.pairingWorkaround = false;
  blt.lastReaponse = 0;
  blt.BTState = Disconnected;
  blt.CallState = Disconnected;
  blt.MusicState = Idle;
  blt.PowerState = Off;
  blt.receivedSppData = "";
  blt.CallerID = "";
  blt.inreset = 0;
  blt.bootup = 0;
  blt.lastCMD = NOOP;//LAST COMMAND SENT, FROM DEFINES
  blt.sendCMD = NOOP; //COMMAND TO SEND, FROM DEFINES
  Serial.begin(9600);
  Serial.setTimeout(10);
}
void bluetoothReset() {
  if (blt.inreset != 0 || blt.bootup != 0) return;
  PORTD &= ~(1 << PD5); //digitalWrite(_reset, LOW);
  DDRD |= (1 << PD5); //pinMode(_reset,OUTPUT);
  blt.inreset = BT_RESET_TIME;
}
void bluetoothCheckReset() {
  if (blt.inreset == 0)return;
  blt.inreset--;
  if (blt.inreset == 0)
  {
    blt.bootup = BT_BOOTUP_TIME;
    blt.PowerState = On;
    DDRD &= ~(1 << PD5); //pinMode(_reset,HIGH);
    PORTD |= (1 << PD5); //digitalWrite(_reset, HIGH);
    Serial.flush();
  }
}
void bluetoothPoll() { //SEND IF CMD IS CUED ELSE POLL
  bluetoothCheckReset();
  if (blt.bootup > 0)
  {
    blt.bootup--;
    return;
  }
  if (blt.lastReaponse > 10)
  {
    blt.lastReaponse = 0;
    bluetoothReset();
    return;
  }
  if (blt.PowerState == On && blt.lastReaponse < 100)
    blt.lastReaponse++;
  if (blt.sendCMD == NOOP) { ///THERE IS NOT A CUED COMMAND
    switch (blt.lastCMD) {
      case CHECK_CONN_STAT:
        blt.sendCMD = CHECK_MUSIC_STAT;
        break;
      case CHECK_MUSIC_STAT:
        blt.sendCMD = CHECK_HFP_STAT;
        break;
      default:
      case NOOP:
      case CHECK_HFP_STAT:
        blt.sendCMD = CHECK_CONN_STAT;
        break;
    }
  }
  blt.lastCMD = blt.sendCMD;
  String cmd = "  ";
  for (uint8_t i = 0; i < MAX_BLT_COMMAND; i++)
  {
    if (bltcommands[i].cmd == blt.sendCMD)
    {
      cmd = "AT+" + String(bltcommands[i].stringcmd) + "\r\n";
      break;
    }
  }
  // Serial.println("R" + String(blt.inreset) + "p" + String(blt.PowerState));
  if (blt.inreset == 0 && blt.PowerState == On)// IF POWERED ON and NOT in RESET
  {
    Serial.print(cmd);
#ifdef DEBUGEEE
    DEBUGSERIAL.println("SENT->" + cmd);
#endif
  }
  blt.sendCMD = NOOP;
}
void BTsendCMD(BT_CMD cmd) {
  blt.sendCMD = cmd;
}
String bluetoothreturnCallerID(String receivedString) {
  return receivedString.substring(4, (receivedString.length() - 2)); //start at 4 cose: IR-"+123456789" or PR-"+123456789" and one before end to remove " and \0
}
void bluetoothcheckResponse() {
  if ( blt.inreset == 0 &&  Serial.available() > 0)
  {
    String receivedString = "";
    int c =  bluetoothtimedRead();
    while (c >= 0 && c != 0xA)
    {
      receivedString += (char)c;
      c =  bluetoothtimedRead();
    }
    if (receivedString.length() > 2)
    {
#ifdef DEBUGEEE
      DEBUGSERIAL.println("REC<-" + receivedString);
#endif
      bluetoothdecodeReceivedString(receivedString);
    }
  }
}
uint8_t bluetoothdecodeReceivedString(String receivedString) {
  blt.lastReaponse = 0;
  switch (receivedString[0]) {
    case 'C': //c1 connected c0 disconnected
      blt.PowerState = On;//we received a respone bluetooth is on
      switch (receivedString[1]) {
        case '1':
          blt.BTState = Connected;
          blt.pairingWorkaround=false;
          break;
        case '0':
          blt.BTState = Disconnected;
          break;
      }
      break;
    case 'E':
      blt.PowerState = On;
      switch (receivedString[1]) {
        case 'R':
          if (receivedString[2] == 'R') return 0;///ERRR
          break;
      }
      break;
    case 'I':// connection info
      blt.PowerState = On;
      switch (receivedString[1]) {
        case 'I': //BT connected
          blt.BTState = Connected;
          break;
        case 'A': //BT disconected
          blt.BTState = Disconnected;
          break;
        case 'R': //caller
          if (receivedString[2] == '-') blt.CallState = IncomingCall;
          // blt.CallerID = returnCallerID(receivedString);
          break;
      }
      break;
    case 'M': { //music
        blt.PowerState = On;
        switch (receivedString[1]) {
          case 'B':
            blt.MusicState = Playing;
            break;
          case 'A':
            blt.MusicState = Idle;
            break;
          case '0':
            blt.BTState = Disconnected;
            break;
          case '1':
            blt.BTState = Connected;
            break;
          case '2':
            blt.CallState = IncomingCall;
            break;
          case '3':
            blt.CallState = OutgoingCall;
            break;
          case '4':
            blt.CallState = CallInProgress;
            break;
        }
      }
      break;
    //    case 'N':
    //      {
    //        blt.PowerState = On;
    //        //         if (receivedString[1] == 'A' && receivedString[2] == ':') {//name
    //        //           BT_NAME = BK8000L::returnBtModuleName(receivedString);
    //        //          }
    //      }
    //      break;
    case 'P':
      {
        blt.PowerState = On;
        switch (receivedString[1]) {
          case 'R': //outgoing call
            if (receivedString[2] == '-') blt.CallState = OutgoingCall;
            blt.CallerID = bluetoothreturnCallerID(receivedString);
            break;
            //            case 'N':
            //              if (receivedString[2] == ':') {
            //                BT_PIN = receivedString.substring(4);
            //              }
            //              break;
        }
      }
      break;
    case 'O': //BT On or received OK
      switch (receivedString[1]) {
        case 'N':
          blt.PowerState = On;
          break;
        case 'K':
          switch (blt.lastCMD)
          {
            case SHUTDOWN:
              blt.PowerState = Off;
#ifdef DEBUGEEE
              DEBUGSERIAL.println("POWEROFF-OK");
#endif;
              break;
            case CALL_HANGUP:
              blt.CallState = Disconnected;
#ifdef DEBUGEEE
              DEBUGSERIAL.println("HANGUP-OK");
#endif;
              break;
            case CALL_REJECT:
              blt.CallState = Disconnected;
#ifdef DEBUGEEE
              DEBUGSERIAL.println("REJECT-OK");
#endif;
              break;
            case CALL_ANSWER:
              blt.CallState = CallInProgress;
#ifdef DEBUGEEE
              DEBUGSERIAL.println("ANSWER-OK");
#endif;
              break;
            case NOOP:
              blt.PowerState = On;
              break;
            case PAIR:
              blt.pairingWorkaround = true;
              break;
            case EXIT_PAIRING:
              blt.pairingWorkaround = false;
              BTsendCMD(CON_LAST_DEVICE);
              break;
            case CON_LAST_DEVICE:
              blt.pairingWorkaround = false;
              break;
          }
          break;
      }
      break;
    case 0xA: //\r
      bluetoothdecodeReceivedString(receivedString.substring(1));
      break;
    case 0x20: //space
      bluetoothdecodeReceivedString(receivedString.substring(1));
      break;
      blt.lastCMD = NOOP;
  }
  return 1;
}

void BTairingInit() { //  pairing   AT+CA\r\n
  blt.BTState = Pairing;
  // blt.pairingWorkaround = true; ///WORKAROUND TO INDICATE THAT WE ARE IN PAIRING MODE
  //blt.lastCMD = PAIR;
  BTsendCMD(PAIR);
}

void BTpairingExit() {//  Exit pairing  AT+CB\r\n
  blt.BTState = Disconnected;
  BTsendCMD(EXIT_PAIRING);
}

void BTcallANSWER() { //  Answer the call   AT+CD\r\n
  blt.CallState = CallInProgress;
  BTsendCMD(CALL_ANSWER);
}

void BTcallReject() { //  reject a call   AT+CF\r\n
  blt.CallState = Disconnected;
  BTsendCMD(CALL_REJECT);
}

void BTcallHangUp() { //  Hang up   AT+CG\r\n
  blt.CallState = Disconnected;
  BTsendCMD(CALL_HANGUP);
}
void BTshutdownBT() { //  Shutdown  AT+CP\r\n
  blt.PowerState = ShutdownInProgress;
  BTsendCMD(SHUTDOWN);
}



void BTmusicTogglePlayPause() { //  Music Play / Pause  AT+MA\r\n
  BTsendCMD(MUSIC_PLAY);
}

void BTmusicStop() { //  The music stops   AT+MC\r\n
  BTsendCMD(MUSIC_STOP);
}

void BTmusicNextTrack() { //  next track  AT+MD\r\n
  BTsendCMD(MUSIC_NEXT);
}

void BTmusicPreviousTrack() { //  previous track  AT+ME\r\n
  BTsendCMD(MUSIC_PREV);
}

void BTmusicFastForward() { //  fast forward  AT+MF\r\n     test how does this exacly works?
  BTsendCMD(MUSIC_FF);
}

void BTmusicRewind() { //  rewind  AT+MH\r\n     test how does this exacly works?
  BTsendCMD(MUSIC_FR);
}
