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
/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/
void INIT_set_Vcore(unsigned char);
void INIT_SVS_supervisor(void);

void initHardware(void);            //!< Initialize Peripherals for the Hart Application

/*************************************************************************
  *   $GLOBAL VARIABLES
*************************************************************************/
/*************************************************************************
  *   $INLINE FUNCTIONS
*************************************************************************/
/*!
 *  \function disableHartTxDriver()
 *  Put the hart modem in listen mode
 */
inline void disableHartTxDriver(void)
{
  HART_UART_TXCTRL_PORTOUT |= HART_UART_TXCTRL_MASK;
}

/*!
 *  \function enableHartTxDriver()
 *  Put the hart modem in talk mode
 */
inline void enableHartTxDriver(void)
{
  HART_UART_TXCTRL_PORTOUT &= ~HART_UART_TXCTRL_MASK;
}

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




#endif /* HARDWARE_H_ */
