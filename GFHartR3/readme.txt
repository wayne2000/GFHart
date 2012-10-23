//	GF Hart Communication Module - code Rev 3. renew 
//	10/23/12
Added hartcommandr3.x, common_h_cmd.*
TEMP increase stack from 240 to 300
-- Still not working as I need to anlize how hartTransmitter() is initialized
// 10/22/12
-Receiver State machine design complete
-Need the processHartCommand() function defined in HartCommand.x
 
//	10/18/12 - The original receiver works with Rx/Driver!!
1) Merging files from the original (trimmed) project
2) added files will have a "r3" suffix in its name: hartr3, utilitiesr3, main9900r3
3) create a contention fileset merge.x to incremental add files
4) Project Run-Time model options setting changed:
	--silicon_version	msp 	uses msp430 cpu
	--abi			eabi	uses eabi for aplication binary interface
	--code_model		small	16b function pointers and 64K low memory
	--data_model		small	16b data pointers and 64K low memory

GOAL is to =>>  Validate a RX 475 communicator frame, RX a command and TX a response
- copy files from \git\HartGitHub\HartFw3 project (win - cloned from GitHub)
	- marco/git/Hart/HartFw3	(linux)

Compiler requires processHartCommand() need to add files
- Rx Fifo stores received byte with status
- Using events to enter main loop
- define a version of hartReceiverSm()
- Increase stack 200->240 emulator losses some calls to onlines
- Cleanup protocols.x and call hartReceiverSm() from main. Establish interface

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
		
	
		 	