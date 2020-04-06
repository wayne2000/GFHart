//	GF Hart Communication Module - code Rev 3. renew  FW developing JOURNALING
//	2/5/13
We found that the AC coupling cap C7 for the RTS needed to be increase to 0.1uF to provide enough
charge to reliable turn ON the opto couplers.
A single positive transaciton is needed to set the T-flip flop (Q =RTS = High) 
A init pulse is given as soon as micro jumps to main and another after setting the clock.
//

//	2/4/13
The RTS control signal is initialized with 500uS width pulses and High Drive Strenght at control pin
//  
//	1/24/13
1) hostActive bit stays set when Hart master is connected (it toggled in previous version). 
It is cleared after HART_MASTER_DISCONNECTED_MS ms of removing the hart master - for now is 16 (2 Secs)
2) The reminder UPDATE_REMINDER_COUNT was changed from 20 to 8000 ( 8000 *0.15 sec = 20 min) and is reset 
	every CMD 45/46 (adjust min current and gain)

//	1/18/13
Added a test case comming from ULA038a. In this case we don't have a Hart Master sending cyclical messages. 
A timer of 1.5 secs (HART_CONFIG_CHANGE_SYNC_TICKS) forces the synchronization with flash
  
//	1/17/13
Corrected Following Hart Comands:
 -Command 3. It sends the Dynamic Variables available ONLY (previous version interpreted different this command)
 -Command 8. Sends the correct Variable Classification for PV and SV. TV and QV remains same as previous
 			 SV classsification is obtained as follows:
 			 	- If SV units = NOT_USED (250) thats the Classification we send
 			 	- If PV Classification is not LEVEL, then we send TEMP as the SV class
 			 	- If PV Class is level and then SV class is obtained from SV units
 			 		- Kg or Lb SV Class is Volume per Mass
 			 		- All other SV class is Volume per Volume
 
 Command 9 - functional the same, but removed all unnecesary comments
 			 		 
 
//	1/15/13
Hart received character wit Uart Error (FE, PE, OE and BREAK) goes to the RxFifo. An evHartRxChar event is generated and
the Receiver State machine decides what to do in the Hart Frame building. No noticeables side effects  
//	12/27/12
Previous version with No low Power endured all night. From 4:34PM to 10:34AM, 18hrs
Examination found a race condition when testing for going to sleep - the evHsbRecComplete happens
while the testing (calling a function IS_SYSTEM_EVENT for ery event) says no event, we go to sleep while
the evHsbRecComplete happened. uC doesn't call the loop and goes to sleep. At the start of $I reception,
it wakes-up and resumes the pending transmission.

FIX:	Condition for going to sleep must be atomic.
		Improvement. 
			HSB activity has high traffic, we wake up and not going to sleep during the start of hsb listen slot.
		
		When Hsb Attention timer expires, I do a cleanup of RX- this: a)avoids a extra interrupt, b) clear status from previous


//	12/27/12
It fails at same point 
Let's run it overnight without low power undef  LOW_POWERMODE_ENABLED 


//	12/27/12
** The extra char received after Hart finished transmission, is due the RTS-Modem interaction.
It happens that the RX line stays an extra 3.2 mS after RTS has gone high. This is seen by uC as a RX char, sometimes with error
- FIX:	After the TX ends, in the ISR is perfomed a clean-up which basically sets the UCSWRST bit, leaving the UART as POR.

** HSB recovery
The Error handler is done after the Hart has finished. Also we limit of 5 retries the attempts to reset port (Astro-Med storage)

** Flashing is being forced to every 10 secs

Tests indicates that HSB losses communication (even with flash). Hooked all monitor and waiting to have an incident on 12/27/2012 @2:39PM



// 	12/26/12
1) Plan to Divide-and-conquer: found that it Fails even when flash is disabled start: 8AM failed: 8:36AM
2) Using Astro-Med I founds events in main loop:
	2.1) event evHartRxChar -> this calls hartReceiver() and since was 'prepared' in the last rx loop, it init the Rcv State Machine
	2.2) event evHartRcvReplyTimer -> creates the delay??	
	2.3) but, why I receive an evHartRxChar event?
	There is a xtra character after LRC that generates a Hart RXChar event.

	Did the following to correct:
	a) evHsbRecComplete event has higher priority than evHartRxChar (and any other event)
	b) REPLY timer
	 	- Starts when hartReceiver is in LRC internal state (same as before)
	 	- It is not kicked (this is not a watchdog type)
	 	- It stops by itself at ISR when registering the evHartRcvReplyTimer,i.e. generates only ONE event per frame
	c) GAP timer
		- Starts when hartReceiver has receiver Preambles + STX (same as before)
		- kicked under ISR at every Hart recvd char
		- Stops when hartReceiver is in LRC internal state (same as before), but also
			- Sets a flag that doesn't allow the timer ISR to register an event and
			- event has been sent to loop, the flag skips the Gap error action  
	 	
	
	
	
 
//	12/21/12
GOALS for Today:
- Timer for HSB counter starts when $H is detected at RxInterrupt
- CCR1 is preset at 100mS -> This is the Flag that ends the Flash write slot time
-
(?) These are possible delays

 
CCR1	Resets and Enables RXIE	Start
CCR2	
1) First
//	12/19/12
This version has the proper event priority
//		12/20/12
This Commit version has evHsbRecComplete with lower priorty
- HSB recovery reads UCA0RXBUF diables/enables RXIE in hope to recover port
- TP3 has a toggle to see loop scan but i could not observe a fault
 
//	12/20/12
- HSB recovers UART errors at ISR and has a supervisory to just start RXIE and get in sync
- Only one timer for HSB - 140mS
- HSB flashwrite slot is between TX ends and the hsbAttentionTimer (all under interrupts)
- Test over night with FlashEnable (normal)
- Optimization disabled
- Low Power 

 

//	12/19/12
=== GENERAL
Start with the following
HART Doesn't stop HSB
HSB  Doesn't stop HART

Change the way HSB is received -
- Reception made on ISR
- Single timer (drop active-idle slots, too slow)
- TXIE is always enabled
- RXIE is always ON at start, once HSB syncs it is OFF at TX ends and enabled at hsbAttentionTimer timeout  

1) HSB RECEIVER moved to ISR
All chars are received under interrupt, no RxFifo
1.1 AT beginning RXIE is always ON
1.2 Detect a $H - Start hsbAttentionTimer and capture following Rx bytes
1.3 If Uart error (FE, PE, OE) just signals cancel this frame and RXIE=0
1.4 Capture until CR (or abort if exceed) - 
	When CR detected, RXIE=0, SET_EVENT(evHsbRecComplete)
	NOTE: Latency to start response is 800uS
	
--> Need to take care of UART UCOE error as Rx chars may arrive when flashing blocking
 
2) Main loop catches evHsbRecComplete
2.1 Process Command, which basicaly send-to-completion sz9900CmdBuffer[], 
	Index is volatile WORD i9900CmdBuf	// Command Buffer Index   

3) HSB TRANSMIT ISR
3.1 Chain the next char in the response buffer sz9900CmdBuffer[] (uses TxFifo)
3.2 At completion it signals the flashWriteAllowedHsb to flash if required 
	 
4) hsbAttentionTimer ISR
- Enables the RXIE = 1

5) Supervisory Recovery, 
- Purge andy pending TX and RX
RXIE =1
All variables should be "volatile"


//	12/18/12
Tested that HSB can't complete when flashing (FORCE_FLASH_WRITE. Still do not know if 475 commands to flash but will run overnight to see
- Flag to to flash was changed to flashWriteAllowed. This flasg is set after HSB has completed, and in turn it will take another main loop
cycle to write to flash, but all house cleaning is done before flashing. Tested to write to flash, after more testing try to save TAGS.
- Watchdog clock assigned different in flash routines - Removed all previous functions and modified: start, stop and kick the Dog. 

 
//	12/17/12
Lost communication while flashing (need to know if 475 sends this command)
THe variable flashWriteAllowed should be volatile  
// 12/17/12
- Add flag to reject gap error when reply timer already started
- Add volatile modifier to sEvents[] and now compiler allows Optimization
- Validate hsb buffer txmitt before sending responseSize <= MAX_9900_RESP_SIZE
- Optimization level set to 2, speed (5)

12/15/12
Errors,
1) Hart Gap timer eventually gets trigger, even an answer is sent to Hart the event may indicate a conflict
2) HSB catchs an error code 3, it recovers but data is sent after $I. 
3) After a long time, my guess is about 24 hrs, Hart stops working. Hart becomes irresponsive at that point
==>It is not caused by the 24hrs timestamp


12/11/12
Added a supervisory function to HSB - restarts serial port 
Found that after few hours of operation HSB is not communicating. There was no recovery - provided a reinitialization of serial port


12/11/12
Documenting Code - Several changes but *.HEX file has no change to 12/10/12 (test)

12/10/12
We go for LPM0 instead LPM1 in order to have FLL always stabilizing DCO
Note that Icc is higher as FLLs wasn't working in previous SW (390uA), 
Measured Icc was in the range of 700uA (??) but I think we have a damaged board Rev. F

12/7/12
Previous commit (LPM1) was tested overnigth - Ran good
For this commit
Flash data in the proper slot -
Prepare for a weekend test: 8 channels, two for signals: HSB and HART
Some test points to detect faults
- Power UP Reset
- Hart Gap timer
Idea is to have trigger point to catch errors in recorder

6) Testing LOW POWER MODE 
ACLK and SMCLK are monitored at Pins #define MONITOR_ACLK_SMCLK 
FLL always ON - No ON/OFF control
LPM1 		430uA	===> This is what will be installed on Production version
LPM3		389uA

FLL always OFF - No ON/OFF control
LPM1		400uA
LPM3		490uA	(makes no sense, it may not be entering LPM3)

No monitoring pins 
FLL always ON	== Suggested by John Y.
LPM1		370uA
LPM3		370uA
FLL always OFF 
LPM1		329uA
LPM3		367uA	(may not be entering)

===> Set testing Overnight in LPM1 FLL always ON 

12/6/12 Completed following tasks for TESTING - Commits as necessary
1) dataTimeStamp reflects mS in one day (same as original code) 
2) Flash writting stress protection FLASH_WRITE_MS 4000 logic
3) Return HSB
	Start HSB after Hart (ON sequence)
	Stop Hart if HSB stops (OFF sequence) Use TP11-> Gnd w 2K resistor, Note that this logic works only at powerup
4) WD clk source= ACLK,  Timer Interval selected by WD_TIME_INTERVAL define (1 sec should be production)	
5) Command 42 is not-supported but when received it completes a proper reply 

=== In progress
- Remove any debug left overs (TestPts, Clear oscillator fault in main)
6) Test Low Power
7) Logic for Flashing and testing
- Evaluate Impact of Sillicon revs. E and rev F
- Observe WD value 
	@pre_init.c WDCTL = 0x6904	(this is the POR default)
	@main 		WDCTL = 0x6980	(this is the POR default)
	resetWatchdog() function in toggleRTSline kicks the WD only, need to HOLD or refresh WD afterward
	
	
- Push to upstream 
- Move WDTIS_4 (1 sec)
- Return FLL control loop startegy: FLL starts ON to get closer to calibrated Freq. Control ON/OFF during Idle/Active Slots
- Verify that project has following settings:
	--silicon_version	msp 	uses msp430 cpu
	--abi				eabi	uses eabi for aplication binary interface
	--code_model		small	16b function pointers and 64K low memory
	--data_model		small	16b data pointers and 64K low memory
	--stack_size		380
	C compiler Optimizations OFF
	pre_init.c:			WDTCTL = WDTPW + WDTHOLD;               // Stop WDT

Note: run-time stack over a pattern shows an unsage x43FE-x436A = x94 (224) bytes with HSB and HART 
	Allocation is 380 (0x17C) bytes FFFE- 17C = 0x4282 (bottom)
	POR:		0x43FE
	main()		0x43BA
	endles loop:0x43BA -> Isr (random)	0x43A6  = Stack takes x14 (20) bytes to take Hart isr
	   

12/5/12 - Solved Watchdog long reset (34min) - 
Watchdog wasn't disabled, trip time = 2^31/1,048,576 = 34min
- WD tested setting for now is:	WDTSSEL_1(ACLK), WDTIS_3 (16 sec), kicked every loop scan
	
12/4/12	-- Fixed confusing variable declarations on headers	
These global variables in merge.h were declared duplicated
==>> extern unsigned int flashWriteCount;		BEWARE, it was long in Vijai
==>> extern long dataTimeStamp;		000029a2   dataTimeStamp				
MAP File addresses
000029a0   flashWriteCount	<====
000029a4   dataTimeStamp	<====


11/29/12
==== REALLY HARD
	- disable LOW POWER MODE (to be able to use emulator)
	- Test trap (hart reply timer)
	- Run it for fe minutes
		It Fails to respond to Communiactor after 40mins
		
	===> Run test Overnight with no HSB
			TEST_ALL_NIGHT

** Preparing code for testing **
-> Return from LPM3 to LPM1
-> Organize States to start/stop Hart and Hsb
-> added pre_init as initialization got too high
-> Trap all unused vectors
  
11/28/12
- Hart Alone works in Low Power Mode 1 and 3  
	(to return code to full see sections where HSB is removed as HART_ALONE_LPM)
Code works 
- Solved BUG: From time to time, the GAP timer expires, canceling current transaction
- kickRecTimer() was moved to Hart Rx ISR, it means that main loop may have some latency.
- Ocasionally I got a Reply error, but no Hart RX chars in the 200mS boundary, isolated Char make it fail alone === KEEP observing
	==> It means that the Gap timer can be used to restart the RX state machine, How to do it without causing a Fault????
Other bugs, 
	Notice that there were unread chars in FIFO (hard to say, as Debugger intrudes with program), but handled same as Hsb: read while fifo non-empty 

Other:
	Added vectors for the unused non-maskable ISRs: 
		SYSNMI_VECTOR (Vacant Memory Access, JTAG Mailbox), 
		UNMI_VECTOR (Oscillator Fault, Flash Memory Access Violation)

********
- Hart and Hsb working together - No Low Power Mode yet
- Removed old-style timers from code: incrementMainMsgTimer(), startMainMsgTimer(), stopMainMsgTimer(), init9900Timers()
- Removed Hsb serial port control enableMainRcvIntr(), enableMainTxIntr(),  

11/26/12
- A define TEST_LOW_POWERMODE to enable LowPower.
- Hart fails at about 1:20 min:sec
- FLL is ON only at Powerup - when CPU rev > "E" will enable always
- Sections to save are out
11/15/12
Low Power Mode revisited 

11/14/12
Save Flash after HART transaction & HSB idle

Organize the System events. 
* When HART communication is stopped, databaseOk, updateMsgRcv and hartCommStarted are set to FALSE. 
* When 9900 polls, it will geta bad database status in reply. Db is built and this sets databaseOk TRUE.
* Next update command from 9900 will:	
	* The very first will set current mode and the comm9900Started to TRUE
	* If reestablish a HART stop condition, will require 20 updates to perform this operation
 
it will get  The flag is set to FALSE when  to indicate that Hart has been stopped, 
                                          //  every system tick is polled to reestablished when Database is Ok again 
11/13-12
Clean Code:
- Those that didn't compiled because lack of scope on variables
- Provide equivalent function incrementMainMsgTimer(void) == 10 sec time-out
- Seems that CalculateDatabaseChecksum() is not used
Functions to analyze:
 - Please note that hart_comm_started is also hart_comm_started
 - updateMsgRcvd = TRUE when an update message has been received from the 9900 and HART communications can begin
 - stopHartComm() function coded direclty on  

11/12/12
- Hsb slot timer added with the following functions: start(preset), stop
- Removed a BUG in putcUart(), this conflicted the hLoopBack.enable() function
- Hart replies to 9900 
Only missing so far is the first update command
== I will do a commit as I think is wise to avoid so many edits w/o testings  

Next: 
 -Parse the 9900 command and answer it
 -Enable RX and once in-sync, disable it during the Idle slot.
 
 	  
// 11/9/12 Created a Branch - Where Hart and Hsb worked independenlty
Create a TAG in history hsb_fetching 
Rename legacy project files to _r3 in order to track changes
Edit: Move Variables to better scope main9900_r3.*



//	11/6/12
Corrected generic inline functions: isRxEmpty, isRxFull, isTxEmpty, isTxFull in driverUart.h
Tested with 79 chars (simulating) max Hsb packet (67)
Need to workout more on Uart error handling, so far only the flags are set with no action taken 

//	11/5/12
Testing Hsb Port using single char in/out

//	11/1/12 - 11/4/12
We will test LPM0 later - Move to HSB as schedule needs full functionallity first  
-Tested Hart messages that request saving data to Flash:
	Flash write 40mS, Transaction rate 500 , command_2 = 44*11/1200 +14 = 417. SIlent window = 80mS, margin 50%
	
Commands that generate a write request: 6 Save Polling Address, 17 Write MSG, 18 Write tag, descriptor & date,
19 Write final assembly number, 22 Write Long Tag, 219 Write device ID 
// 10/31/12
Working in saving NV data to flash
== Need to estimate last transaction time -if less than XX then save to flash
  
// 10/30/12
- Analizing Hart command by functions: 0,11,21 = Idenitity
- isAddressValid() moved to protocols.h, reduces scope of defines 
- rtsRcv() not needed (we don't send partial messages in run to completion)
- Later write 18,19, Reset 42
Linker basic options -> ${ProjDirPath}/Debug/GFHartR3.map to aid in Linux/Windows context switch

// 10/29/12
Corrected SendFrame Error

//	10/23/12
DEBUG - Most of the time can't reply to cmd0
- Increase RX buffer to 8-> 16  == no help
- Reply and Gap timers doubled  == they are responding but make it longer no help 

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
		
	
		 	