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
#include "protocols.h"
#include "hartcommandr3.h"
#include "merge.h"
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
unsigned int sEvents[1 + (evLastEvent +1)/16];	// Array where events are stored
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
  initHartRxSm();       // Init Global part, static are intialized at first call



  // Merging original code = higher level inits
  // copy in the 9900 database
  copy9900factoryDb();

  //USE_PMM_CODE-> move to Hw INIT_SVS_supervisor();


  // MH:  I am trying to collect all initializations in a single function
  //      and also to understand the interface, keeping high level here
  initStartUpData();

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
  // Wait until something happens
  while(
  		isRxEmpty(&hartUart) &&
  		//!isEnabledHartTxDriver() &&  // Its is better to test RTS line: isTxEmpty && RTS disable
  		SistemTick125mS < 16 &&
  		IS_SYSTEM_EVENT(evHartRcvGapTimeout) ==0 &&
  		IS_SYSTEM_EVENT(evHartRcvReplyTimer) ==0
  		)//  && all conditions that indicates no-event)
  {
  	//stop_oscillator();
  	_no_operation();
  }
  //	There is an event, need to find from registered ones
  tEvent event = evNull;
  while(event < evLastEvent)
  if( IS_SYSTEM_EVENT(event))			// This is a macro, providing NZ if TRUE
  {
  	_disable_interrupt();
  		CLEAR_SYSTEM_EVENT(event);
  	_enable_interrupt();
  	return event;                 // return the First event found
  }
  else
  	event++;
  return evNull;	                // Never happens, unless unregistered event. Returning NULL is safer (I think)
}


/////////////////////////////////

void main()
{
  volatile unsigned int i;
  low_power =1;

  initSystem();

  // This a Debug test only
  hartUart.bRtsControl = TRUE;
  hartUart.hTxInter.enable();
  _enable_interrupt();    //<   Enable interrupts after all hardware and peripherals set
  volatile BYTE ch= 'A', rP=0,LoadSize=1, echo=0;
  i=0;
  CLEARB(TP_PORTOUT, TP1_MASK);     // Indicate we are running
  tEvent systemEvent;
  initHartRxSm();                   // Init Global part, static are intialized at first call
  while(1)													// continuous loop
  {
  	systemEvent = waitForEvent();
  	switch(systemEvent)
  	{
  	case evHartRxChar:
  	  SETB(TP_PORTOUT, TP1_MASK);
  	  // Just test we are receiving a 475 Frame
  		hartReceiver(getwUart(&hartUart));

  		CLEARB(TP_PORTOUT, TP1_MASK);
  		break;

  	case evHartRcvGapTimeout:
  	  if(hartFrameRcvd ==FALSE)               // Cancel current Hart Command Message, prepare to Rx a new one
  	    initHartRxSm();
  	  HartErrRegister |= GAP_TIMER_EXPIRED;   // Record the Fault
  	  stopGapTimerEvent();                    // Once message cancel is ack, it's ok to turn-off gap check
  	  break;

  	case evHartRcvReplyTimer:
  	  TOGGLEB(TP_PORTOUT,TP2_MASK);
  	  // Process the frame (same as original project)
  	  if (commandReadyToProcess)
  	  {
  	    // clear the flag
  	    commandReadyToProcess = FALSE;
  	    // Initialize the response buffer
  	    initRespBuffer();
  	    // Process the HART command
  	    if (processHartCommand() && !doNotRespond)
  	    {
  	      // If cmdReset is true && cmd reset response is false, then execute this
  	      // ship the HART response
  	      sendHartFrame();
  	      // hartTransmitterSm(TRUE);

  	    }
  	    else
  	      // This command was not for this address or invalid
  	    {
  	      // Get ready to start another frame
  	      initHartRxSm(); //MH: TODO: Look for side effects on Globals

  	    }
  	  }
  	  stopReplyTimerEvent();        // One Reply per command
  	  break;

  	case evHartTxChar:              // Transmission is transparent to user app
  	  _no_operation();
  		break;

  	case evTimerTick:               // System timer event
  		SistemTick125mS =0;						// every 2 secs we get here
  		// Increment the data time stamp and roll it every 24 hours
  		++dataTimeStamp;
  		break;

  	case evNull:
  	default:   // evNull or any non enumerated
  	  break;


  	}




    if(echo && SistemTick125mS >= 1)
    {
      SistemTick125mS =0;
      for(i=0; i< LoadSize; ++i)
        putcUart(ch++,&hartUart);

      if(ch > 'Z' )
        ch = '@';


    }

  }

}



