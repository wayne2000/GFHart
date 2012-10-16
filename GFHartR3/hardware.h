#ifndef HARDWARE_H_
#define HARDWARE_H_
/*
 * hardware.h
 *
 *  Created on: Sep 28, 2012
 *      Author: marco
 */

/*************************************************************************
  *   $INCLUDES
*************************************************************************/
/*************************************************************************
  *   $DEFINES
*************************************************************************/
// 	Timers clock source are ACLK/8 = 32768/8 = 4096hz
//	Reload value is (mS -1)*4.096
//		8, 508, 8188 40956u      	Tick aprox 2mS, 125mS, 2 Sec, 10Sec
#define SYSTEM_TICK_TPRESET	508
//
//	Hart slave timers
//	Inter-character GAP time for 2 chars @1200bps =  11* 2 * 4096 /1200 =  75
//	Slave reply time for 1.5 chars @1200bps =  11* 1.5 * 4096 /1200 =  57
//  Time out is from RX STOP bit + 1 Baud time + Timer preset
//
#define GAP_TIMER_PRESET  75	  /* 18mS ~ 75 tested w 50mS~201 and hyperterminal 		 */
#define REPLY_TIMER_PRESET 53   /* 14mS ~ 53, tested w 100mS~406 and hyperterminal */

/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/
void INIT_set_Vcore(unsigned char);
void INIT_SVS_supervisor(void);

void initHardware(void);            //!< Initialize Peripherals for the Hart Application

/*************************************************************************
  *   $GLOBAL VARIABLES
*************************************************************************/
extern volatile WORD SistemTick125mS;
/*************************************************************************
  *   $INLINE FUNCTIONS
*************************************************************************/

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: startWatchdog()
//
// Description:
//
// Starts the watchdog running
//
// Parameters: void
//
// Return Type: void.
//
// Implementation notes:
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////
inline void startWatchdog (void)
{
  // The watchdog is set for 30 mS
  // Clear the count to start
  WDTCTL = WDTPW | WDTCNTCL | WDTHOLD;
  // start the watchdog
  WDTCTL = WDTPW;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: stopWatchdog()
//
// Description:
//
// Stops the watchdog from running
//
// Parameters: void
//
// Return Type: void.
//
// Implementation notes:
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////
inline void stopWatchdog (void)
{
  WDTCTL = WDTPW | WDTHOLD;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: resetWatchdog()
//
// Description:
//
// This function is used to reset the watchdog circuit.
//
// Parameters:
//
// None.
//
// Return Type:
//
// void.
//
// Implementation:
//
// Stops the watchdog, resets the counter to 0, and restarts the watchdog.
//
///////////////////////////////////////////////////////////////////////////////////////////

inline void resetWatchdog (void)
{
  // reset the counter
  WDTCTL = WDTPW | WDTCNTCL;
  // TBAnalized later WDTCTL = 0x5a88;  // Hard code this so that the HOLD bit does not get cleared
}

/*!
 *	kickHartRecTimers()
 *	Reset timer count register to zero
 *
 */
inline void kickHartRecTimers(void)					//!< Kick the Hart DLL dog
{
	HART_RCV_GAP_TIMER_CTL 		|= TACLR;
	HART_RCV_REPLY_TIMER_CTL  |= TACLR;
}

/*!
 *	stopGapTimerEvent()
 *	Set timer to mode 0 (stop counting) and clear count register
 *
 */
inline void stopGapTimerEvent(void)
{
	HART_RCV_GAP_TIMER_CTL	&= ~MC_3;			// This makes MC_0 = stop counting
	HART_RCV_GAP_TIMER_CTL  |= TACLR;			// Clear TAR
}
/*!
 *	startGapTimerEvent()
 *	Start Counting Up and generate an interrupt if reaches preset
 *
 */
inline void startGapTimerEvent(void)					//!< Kick the Hart DLL dog
{
	HART_RCV_GAP_TIMER_CTL |= MC_1;						// Up mode
}
/*!
 *	stopReplyTimerEvent()
 *	Set timer to mode 0 (stop counting) and clear count register
 *
 */
inline void stopReplyTimerEvent(void)
{
	HART_RCV_REPLY_TIMER_CTL	&= ~MC_3;			// This makes MC_0 = stop counting
	HART_RCV_REPLY_TIMER_CTL  |= TACLR;			// Clear TAR
}
/*!
 *	startReplyTimerEvent()
 *	Start Counting Up and generate an interrupt if reaches preset
 *
 */
inline void startReplyTimerEvent(void)					//!< Kick the Hart DLL dog
{
	HART_RCV_REPLY_TIMER_CTL |= MC_1 ;						// Up mode
}


#endif /* HARDWARE_H_ */
