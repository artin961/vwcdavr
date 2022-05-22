
#CD CHANGER EMULATOR

arduino uno nano atmega328 or atmega168  based vw/audi/skoda/seat radio emulator, I made this to revers eng. on vwcdpic and improve arduino version of CDchanger
 it is designed to work with BK8000L bluetooth module diy kit using at commands
 
 
 The main idea is that changer is always in cd6 and changing disks are interpreted as buttons
 I'm using it on 2006 POLO with its original radio RCD200 
 You need to have BK8000l dev board with AT commands enabled or just ignore bluetooth and use it as line in only
 
 radio buttons:
	CD1 - PLAY_PAUSE or ANSWER CALL
	CD2 - REJECT or HANGUP
	CD3 - PLAY PAUSE
	CD4 - PREV TRACK
	CD5 - NEXT TRACK
	CD6 - nothing, changer is always on cd6
	MIX - pairing mode
	PREV - PREV TRACK
	NEXT - NEXT TRACK
 
#Connectios:

- DataOut -> arduino pin 12

- Clk     -> arduino pin 3

- DataIn  -> arduino pin 4 

- BK8000L reset -> arduino pin 5
- BK8000L TX -> arduino rx
- BK8000l RX -> arduino tx via 1k resistor

- LED CON Status -> arduino pin 13


