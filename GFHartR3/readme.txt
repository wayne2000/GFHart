//	GF Hart Communication Module - code Rev 3. renew 
//	10/11/12
Integrates Hart Receiver state machine - protocols.c
- copy files from 
	- C:\Users\Marco.Ginn\git\Hart\HartFw3 project (win)
	- marco/git/Hart/HartFw3	(linux)
	
- Rx Fifo stores received byte with status
- Using events to enter main loop
- define a version of hartReceiverSm()
- Increase stack 200->240 emulator losses some calls to onlines

If a string "go" is received, when the gap timer expires sends 'g' every second. When "end" is received, it sends
'r' every two seconds

  

- Basic Clock System prepared for LPM0
	ACLK  = XT1CLK  =   32.768Khz external crystal
	MCLK = SMCLK = DCOCLKDIV=  1,058,576
- Testing Commit/pull from two machines
- Testing the fifo/app using of Tx and Rx
- Working in the driverUart - APP/ISR and define members/events to xmit a complete message              
- Debugginfg TxIsr and RxIsr
	System Timer is Disabled for Now

Emulation won't work well 
- CHanged inlines - Not solved
Set stack 160->200 - It worked => CCS doen't check stack. Need to estimate and test

=== Test with small Modem Board
This
- Tx and Rx interrupts always enabled
- Add LoopBack enable() disable() isEnabled() handlers 
- Loopback is enabled before txsbuf is loaded (this makes Rx listen to all Txmited bytes) 
- bHalfDuplex -> bHwFlowControl changed names  
- TODO: Create a HW/SW timer
 
 EVENTS:
ISR Receiver
 	eHartRx
 	eHartRxError

Timer
	eRxGapError
 	eHartReply

ISR Transmitter
	opeSendResponse
	eResponseSent
	
HartReceiverSm
	commands
		build, NewChar	==> 
		reset 			==> start over
		continue
	results
		validFrame 		==> A valid message with my address, internal state to xtra data
		done			==>	a valid message and at least one extra char received (1) 
		error			==> Error found (1)
		busy			==>	A receiver is in progress with no errors so far
		idle			==> Waitting for a valid preamble 
					(1) Internal state changes to Idle state
		
	
		 	