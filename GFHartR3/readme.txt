//	GF Hart Communication Module - code Rev 3. renew 
//	10/8/12
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
- Add LoopBack enable() disable() isEnabled()
- Return Functions to inline
- Need to re-enable timer function and create a HW/SW timer
 