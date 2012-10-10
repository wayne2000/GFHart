//	GF Hart Communication Module - code Rev 3. renew 
//	10/10/12
This test program receives chars from terminal -
Waits for a number N - outputs OK
If a string "GO" is received it start sending N chars every 125mS
  

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
 