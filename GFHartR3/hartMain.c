/*!
 *  \file   hartMain.c
 *  \brief  Main application entry point for GF Hart implementation
 *  Created on: Sep 28, 2012
 *  \author: MH
 *
 *  \mainpage Hart Communication Module Firmware Documentation
 *
 *  \section intro_sec Introduction
 *
 *  This document is extracted from the current firmware code tree.\n
 *  \n
 *  \version 3
 *  Revision History:
 *  \verbatim
  - Date -    Rev Engineer    Description
  - 03/24/11  0   Vijay Soni  Creation
  - 04/02/11  1   BDM         Add funcations
  - 05/27/11  2   BMD         Clean up code
  - 09/13/12  3   MH          New code made from above references
 *
 *  \endverbatim
 *
 * - Using CCS V5.1
 * - Network Git repository at \\\\NEW_ENG\\Marco\\git
 *
 */

//==============================================================================
//  INCLUDES
//==============================================================================
#include "define.h"
#include "msp_port.h"
#include "hardware.h"
#include "hartMain.h"
#include "driverUart.h"
#include <string.h>
//==============================================================================
//  LOCAL DEFINES
//==============================================================================

//==============================================================================
//  LOCAL PROTOTYPES.
//==============================================================================
void initSystem(void);
//==============================================================================
//  GLOBAL DATA
//==============================================================================
unsigned int sEvents[1 + (evLastEvent +1)/16];		// Array where events are stored
//==============================================================================
//  LOCAL DATA
//==============================================================================
volatile int16u  low_power=0;
//==============================================================================
// FUNCTIONS
//==============================================================================
/*!
 * \function  initSystem()
 * \brief     Perform the system level Hart Initialization
 *
 */
void initSystem(void)
{
  initHardware();       //!<  Initialize clock system, GPIO, timers and peripherals
  initUart(&hartUart);
  // last setup of peripheral interrupts
  disableHartTxIntr();
  enableHartRxIntr();
}

/*!
 *  waitForEvent()
 *
 *  Waits for the next event to happen. This is where the main event loop spend its time when there is
 *  nothing else to do. The wait is executed at LPM3 and it takes an interrupt to get it started.\n
 *  Once an interrupt is received, the MCLK starts, and the first thing checked is if any interrupt
 *  that can generate an event is the source. If not, we go back to sleep.
 *  If waken up by an event creating interrupt [Note!!! there can be multiple sources at the same time],
 *  I check them as follows:\n
 */
/// \n
///
tEvent waitForEvent()
{

  while(
  		isRxEmpty(&hartUart) &&
  		SistemTick125mS < 16 &&
  		IS_SYSTEM_EVENT(evHartRcvGapTimeout) ==0 &&
  		IS_SYSTEM_EVENT(evHartRcvReplyTimer) ==0
  		)//  && all conditions that indicates no-event)
  {
  	//stop_oscillator();
  	_no_operation();
  }
  //	There is an event Here
  tEvent event = 0;
  while(event < evLastEvent)
  if( IS_SYSTEM_EVENT(event))			// This is a macro
  {
  	_disable_interrupt();
  		CLEAR_SYSTEM_EVENT(event);
  	_enable_interrupt();
  	return event;
  }
  else
  	event++;
  return 0;	// Never happens
}

#define BUFSIZE 50
void main()
{
  volatile unsigned int i;
  low_power =1;

  initSystem();

  // This a Debug test only
  hartUart.bRtsControl = TRUE;
  hartUart.hTxInter.enable();

  _enable_interrupt();    //<   Enable interrupts after all hardware and peripherals set
  volatile BYTE ch= 'A', RxBuf[BUFSIZE], rP=0,LoadSize=1, echo=0;
  for (i=0; i<BUFSIZE ; ++i)
    RxBuf[i]= 0;
  i=0;

  CLEARB(TP_PORTOUT, TP1_MASK);     // Indicate we are running
  // hartReceiverSm(INIT);
  tEvent systemEvent;
  while(1)													// continuous loop
  {
  	systemEvent = waitForEvent();
  	switch(systemEvent)
  	{
  	case evHartRxChar:
  		kickDllTimers();							//	GapTimer and Response Timer
  		ch = getwUart(&hartUart);
  		RxBuf[2] =	RxBuf[1];
  		RxBuf[1] = RxBuf[0];
  		RxBuf[0] = ch;
  		_no_operation();
  		// Start "Rec"
  		if(!strncmp((const char *)RxBuf,"og",2 ))
  			startGapTimerEvent();
  		else
  			if(!strncmp((const char *)RxBuf,"dne",3 ))
  			{
  				stopGapTimerEvent();
  				startReplyTimerEvent();
  			}

  		break;
  	case evHartRcvGapTimeout:
  		putcUart('g',&hartUart);
  		break;
  	case evHartRcvReplyTimer:
  		putcUart('r',&hartUart);
  		break;
  	case evTimerTick:
  		SistemTick125mS =0;						// every 2 secs we get here
  		break;


  	}


#if 0
      if(rP < BUFSIZE)
        RxBuf[rP++]= getcUart(&hartUart);
      else
        RxBuf[rP=0]= getcUart(&hartUart);
#endif

    if(echo && SistemTick125mS >= 1)
    {
      SistemTick125mS =0;
#if 0
      if(hartUart.hTxDriver.isEnabled())
        hartUart.hTxDriver.disable();
      else
        hartUart.hTxDriver.enable();
#endif

      for(i=0; i< LoadSize; ++i)
        putcUart(ch++,&hartUart);

      if(ch > 'Z' )
        ch = '@';


    }

  }

}


#if 0

if(low_power)
{
  __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3, enable interrupts
  __no_operation();                         // For debugger
}
else
  _enable_interrupt();    //<   Enable interrupts after all hardware and peripherals set

etSystemEvent =waitForEvent();
switch(etSystemEvent)
{
case evHartRxChar:
  break;

}

for(i=5000;i>0;i--);                   // Delay
//P1OUT ^=0x01;

#endif

#if 0
  char inisr=0, inget=0, data=0, status =0;
  WORD rword;
  while(1)
  {
  	if(inisr)			// Simulate a Rxif
  	if(!isRxFull(&hartUart))                //  put data in input stream if no errors
  		putwFifo(&hartUart.rxFifo, status <<8 | data);  // Signal an Event to main loop
  	if(inget)			// Get data
  		rword = getwUart(&hartUart);  // Signal an Event to main loop

  }
#endif
#if 0
    if(hartUart.bNewRxChar)
    {
      hartUart.bNewRxChar = FALSE;
      i= getwUart(&hartUart);
      //  Handle events from modules
          if(hartNewCharInRxFifo)     // Functional equivalent to rxIsr()
          {
            hartNewCharInRxFifo = FALSE;
            // Build the message using Original Function
            hartReceiverSm();
          }
      }
#endif

