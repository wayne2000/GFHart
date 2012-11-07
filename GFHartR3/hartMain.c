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
#include "utilitiesr3.h"
// done #include "merge.h"
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
  initUart(&hartUart);	//!< Initialize Hart Uart @1200bps, 8,o,1
  // last setup of peripheral interrupts
  disableHartTxIntr();
  enableHartRxIntr();
  initHartRxSm();       // Init Global part, static vars are initialized at first call

  //	High Speed Bus initialization
  initUart(&hsbUart);		//!< Initialize High Speed Bus Uart @19200bps, 7,o,1
  //(hsbUart.hRxInter.enable)();		//!<	Enable Hsb Rx interrupt
  hsbUart.hRxInter.enable();
  hsbUart.hTxInter.disable();		//!<	Disable Hsb Tx interrupt

  // Merging original code = higher level inits
  // copy in the 9900 database
  copy9900factoryDb();

  //USE_PMM_CODE-> move to Hw INIT_SVS_supervisor();


  // MH:  I am trying to collect all initializations in a single function
  //      and also to understand the interface, keeping high level here
  flashWriteCount =0;   // Globals init
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
  		//!isEnabledHartTxDriver() &&  // Its is better to test RTS line: isTxEmpty && RTS disable
  		isRxEmpty(&hartUart) &&													// 	Every received Hart character
  		SistemTick125mS < 8 &&													// 	Main loop scanned every 1 sec
  		IS_SYSTEM_EVENT(evHartRcvGapTimeout) ==0 &&			//	Current Hart rx message is broken
  		IS_SYSTEM_EVENT(evHartRcvReplyTimer) ==0 &&			//	Hart message needs a Reply
  		IS_SYSTEM_EVENT(evHartTransactionDone) ==0 &&		//	Hart Command-reply ends and RTS is set to RX mode

  		IS_SYSTEM_EVENT(evHsbRxChar)==0                 //  An arriving character wakes up, but get from input stream all we can get

  		)//  && all conditions that indicates no-event)
  {
    /*!
  	 * stop_oscillator()
  	 * Stops the CPU clock and puts the processor in sleep mode.
  	 * Sleep mode can only be exited by interrupt that restarts the clock.
  	 *
  	 */
  	//_low_power_mode_0();
  	//_low_power_mode_1();
  	//_low_power_mode_3();
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
#define smallBufSize 80
void main()
{
  volatile unsigned int i;
  WORD flashWriteTimer=0,
        Cmds[10],nTotalBytes[10];  // Small debug: list previous Commands and transactions
  WORD  nBytesHartTransaction, nBytesHartRx;  // Not initialized, but allow a few loop scans
  low_power =1;

  initSystem();

  // This a Debug test only
  hartUart.hTxInter.enable();
  hsbUart.hTxInter.enable();
  _enable_interrupt();    //<   Enable interrupts after all hardware and peripherals set
  volatile BYTE ch= 'A', rP=0,LoadSize=1, echo=0;
  i=0;
  CLEARB(TP_PORTOUT, TP1_MASK);     // Indicate we are running
  tEvent systemEvent;
  initHartRxSm();                   // Init Global part, static are intialized at first call
  volatile BYTE sbufs[smallBufSize], j;
  for(j=0;j <smallBufSize; ++j)
  	sbufs[j] =0;
  j=0;
  while(1)													// continuous loop
  {
  	systemEvent = waitForEvent();
  	switch(systemEvent)
  	{
  	case evHsbRxChar:               //  Main loop is too slow for HSB, we need to get all we can for every RX Char (may be more than one)
  	  if(hsbUart.bRxError)          //  Handle error on "Uart" basis not necessary per received char. This is the minimum error handling
  	  {
  	    clearMainUartErrFlags();
  	    resetForNew9900Message();
  	    // This was done inside the RxSm - find a place to do it
#if 0
  	    // Check for errors first, and wait for the next message if any are found
  	      statusReg = MAIN_STAT;
  	      if (statusReg & (UCFE | UCOE | UCPE | UCBRK | UCRXERR))
  	      {
  	        // record the error
  	        ++numMessagesErrored;
  	        // Clear error flags
  	        clearMainUartErrFlags();
  	        // Ignore anything already received
  	        resetForNew9900Message();
  	        // Bail now
  	        return;
  	      }

#endif


  	  }
  	  else
  	  while(!isRxEmpty(&hsbUart))   //  We need a 50mS timer to limit the time since the first time we enter here to the end of message
  	  {
  	    SETB(TP_PORTOUT, TP1_MASK);
  	    hsbReceiver(ch=getcUart(&hsbUart));

  	    if(j < smallBufSize)
  	      sbufs[j++] = ch;
  	    else
  	      _no_operation();
  	    CLEARB(TP_PORTOUT, TP1_MASK);
  	  }
  	  //  	  		putcUart(ch, &hsbUart);
  	  _no_operation();
  	  CLEARB(TP_PORTOUT, TP1_MASK);
  	  break;
  	case evHartRxChar:
  	  	  //SETB(TP_PORTOUT, TP1_MASK);
  	  	  ++nBytesHartRx;       // Count every received char at Hart (loop back doesn't generate and event)
  	  	  // Just test we are receiving a 475 Frame
  	  		hartReceiver(getwUart(&hartUart));

  	  		//CLEARB(TP_PORTOUT, TP1_MASK);
  	  		break;

  	case evHartRcvGapTimeout:
  	  if(hartFrameRcvd ==FALSE)               // Cancel current Hart Command Message, prepare to Rx a new one
  	    initHartRxSm();
  	  HartErrRegister |= GAP_TIMER_EXPIRED;   // Record the Fault
  	  stopGapTimerEvent();                    // Once message cancel is ack, it's ok to turn-off gap check
  	  break;

  	case evHartRcvReplyTimer:
  	  //TOGGLEB(TP_PORTOUT,TP2_MASK);
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
  	      for(i=0;i<9; ++i)
  	      {
  	        Cmds[i]= Cmds[i+1];
  	        nTotalBytes[i] = nTotalBytes[i+1];
  	      }
  	      Cmds[i] = hartCommand;
  	      nBytesHartTransaction = sendHartFrame() + nBytesHartRx;
  	      _no_operation();    // Debug number of Rx
  	      nBytesHartRx =0;  // Reset counter for the next transaciton
  	      nTotalBytes[i] = nBytesHartTransaction;

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
  	case evHartTransactionDone:
  	  //  After the Hart transaction ends we have a silent line time, can last 80mS (Cmd 1,2)
  	  //  ***** Write To Flash *****
  	  if(flashWriteTimer >= 4)   // every 4 secs
  	  {
  	    flashWriteTimer =0;
  	    if(updateNvRam && nBytesHartTransaction <=50  )
  	    {
  	      SETB(TP_PORTOUT,TP3_MASK);
  	      updateNvRam = FALSE;
  	      ++flashWriteCount;
  	      syncNvRam();
  	      CLEARB(TP_PORTOUT,TP3_MASK);
  	    }
  	  }
  	  break;

  	case evHartTxChar:              // Transmission is transparent to user app
  	  _no_operation();
  		break;

  	case evTimerTick:               // System timer event - Get here every 8 x125mS = 1 Sec
  	  //TOGGLEB(TP_PORTOUT,TP2_MASK);
  		SistemTick125mS =0;
  		// Increment the data time stamp and roll it every 24 hours
  		++dataTimeStamp;
  		++flashWriteTimer;

  		// This is the equivalent function provided in hardware.c
      #ifdef QUICK_START
  		// This timer is only incremented when we first start up. If it hits the
  		// timout, it means that the 9900 is disconnected and need to stop HART communication
  		if (FALSE == comm9900started)
  		{
  		  ++comm9900counter; // Increment the 9900 startup timer
  		  if (MAX_9900_TIMEOUT < comm9900counter)
  		  {
  		    // Reset the counter so it checks every few seconds
  		    comm9900counter = 0;
  		    stopHartComm();
  		  }
  		}
  		#endif


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



