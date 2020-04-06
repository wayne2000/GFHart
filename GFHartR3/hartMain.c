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
 *  Major features added to original revision 2 are the following:
 *  - Serial communication drivers for Hart and High speed Serial bus
 *  - Fifo buffers to isolate interrupt service routines from foreground tasks
 *  - Low power operation
 *  \n
 *  Firmware was developed with CCS V5.1\n
 *  Last code is stored at
 *   and there is a remote repository at https://github.com/MarcoHG/GFHart.git
 *
 *
 *
 *  \n
 *  \version 2b
 *  Revision History:
 *  \verbatim
  - Date -    Rev Engineer    Description
  - 03/24/11  0   Vijay Soni  Creation
  - 04/02/11  1   BDM         Add funcations
  - 05/27/11  2   BMD         Clean up code
  - 09/13/12  2a   MH         New code made from above references
  - 12/11/12  2b   MH         Last revision for testing
  - 2/4/13    2c   MH         - RTS pulses during power up, 150 to 600uS
   *
 *  \endverbatim
 *
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
#include "hartcommand_r3.h"
#include "main9900_r3.h"
#include <string.h>
//==============================================================================
//  LOCAL DEFINES
//==============================================================================

//==============================================================================
//  LOCAL PROTOTYPES.
//==============================================================================
void initSystem(void);
void hsbErrorHandler(void);
void pulseTp4(BYTE errCode);
//==============================================================================
//  GLOBAL DATA
//==============================================================================
volatile unsigned int sEvents[1 + (evLastEvent +1)/16];	// Array where events are stored

#ifdef FORCE_FLASH_WRITE
WORD testWriteFlash =0;
#endif

//==============================================================================
//  LOCAL DATA
//==============================================================================
#if 0
  // Testing Active vs. Lowpower
volatile int16u  low_power=0;
BOOLEAN lowPowerModeEnabled= FALSE;
#endif

WORD  flashWriteTimer=0;//        nBytesHartRx;

/*!
 * Monitor HSB activity
 *
 * Monitor the HSB for activity,
 * (These may be promoted to local scope under main() )
 */
volatile int16u  hsbNoActivityTimer=0;
volatile BOOLEAN hsbNoActivityError= FALSE;
volatile BOOLEAN bRequestHsbErrorHandle= FALSE;

volatile WORD hsbWAtchDog;

//HICCUP: flashWriteCount was declared as unsigned int but used as long
unsigned long int flashWriteCount=0;
unsigned long int dataTimeStamp=0;

//==============================================================================
// FUNCTIONS
//==============================================================================

//HICCUP
#include "utilities_r3.h"

/*!
 *  initialize local data structure for the first time, or if the NV memory is ever corrupted.
 *
 *  If the NV memory is corrupt, copy the factory structure into ram, then copy the RAM to the FLASH
 */
void initializeLocalData (void)
{
  int numSegsToErase;
  // Clear the local structure
  memset(&startUpDataLocalNv, 0, sizeof(HART_STARTUP_DATA_NONVOLATILE));
  // Now copy the factory image into RAM
  memcpy(&startUpDataLocalNv, &startUpDataFactoryNv, sizeof(HART_STARTUP_DATA_NONVOLATILE));
  // Now copy in the NV unique device ID
  copyNvDeviceIdToRam();
  // erase the segment of FLASH so it can be reprogrammed
  numSegsToErase = calcNumSegments (sizeof(HART_STARTUP_DATA_NONVOLATILE));
  eraseMainSegment(VALID_SEGMENT_1, (numSegsToErase*MAIN_SEGMENT_SIZE));
  // Copy the local data into NV memory
  copyMemToMainFlash (VALID_SEGMENT_1, ((unsigned char *)&startUpDataLocalNv),
    sizeof(HART_STARTUP_DATA_NONVOLATILE));
}


void initStartUpData()
{
  // Load up the volatile startup data
  // Clear the local structure
  memset(&startUpDataLocalV, 0, sizeof(HART_STARTUP_DATA_VOLATILE));
  // Now copy the factory image into RAM
  memcpy(&startUpDataLocalV, &startUpDataFactoryV, sizeof(HART_STARTUP_DATA_VOLATILE));
  // Load up the nonvolatile startup data
  // Load the startup data from NV memory
  syncToRam(VALID_SEGMENT_1, ((unsigned char *)&startUpDataLocalNv), sizeof(HART_STARTUP_DATA_NONVOLATILE));
  // If the local data structure has bad values, initialize them
  if (GF_MFR_ID != startUpDataLocalNv.ManufacturerIdCode)
  {
    initializeLocalData();
  }
  else
  {
    // Make sure we have the correct Device ID in any case
    verifyDeviceId();
  }
  // Set the COLD START bit for primary & secondary
  setPrimaryStatusBits(FD_STATUS_COLD_START);
  setSecondaryStatusBits(FD_STATUS_COLD_START);
  ++startUpDataLocalV.errorCounter[8];
}



/*!
 * System level initialization
 *
 * Performs an ordered start up of the Hart micro controller, low level hardware first, clock,
 * peripherals and then data base.
 *
 */
void initSystem(void)
{
  initHardware();       //  Initialize clock system, GPIO, timers and peripherals
  initUart(&hartUart);	//  Initialize Hart Uart @1200bps, 8,o,1
  // Hart Starts First
  hartUart.hTxInter.enable();
  hartUart.hRxInter.enable();
  initHartRxSm();           // Init Global part, static vars are initialized at first call

  //	High Speed Bus initialization - RX is enabled some time after OR few Hart transactions
  initUart(&hsbUart);		        // Initialize High Speed Bus Uart @19200bps, 7,o,1
  hsbUart.hTxInter.enable();

  //  MH - Just to be sure on timming
  kickWatchdog();

  // Merging original code = higher level inits
  // copy in the 9900 database
  copy9900factoryDb();

  //USE_PMM_CODE-> move to Hw INIT_SVS_supervisor();


  // MH:  I am trying to collect all initializations in a single function
  //      and also to understand the interface, keeping high level here
  initStartUpData();

}

/*!
 *  Waits for the next event to happen
 *
 *  This is where the main event loop spend its time when there is nothing else to do.
 *  The wait is executed at LPM0 and it takes an interrupt to get it started.\n
 *  Once an interrupt is received, the MCLK starts, and the first thing checked is if any interrupt
 *  that can generate an event is the source. If not, we go back to sleep.\n
 *  If waken up by an event creating interrupt [Note!!! there can be multiple sources at the same time],
 *  they are check them as follows:\n
 *  - Hart events: evHartRxChar, evHartTxChar,  evHartRcvGapTimeout,  evHartRcvReplyTimer, evHartTransactionDone
 *  - High Speed Bus events: evHsbRxChar, evHsbTxChar,  evHsbRcvReplyTimer,  evHsbTxDone, evHsbSlotTimeout
 *  - System Events:  evTimerTick
 *
 *  \sa #tEvent #sEvents
 */
tEvent waitForEvent()
{
  // Wait until something happens
  while(  NO_EVENT() )               // 12/28/12 This MACRO collects all events in Atomic instruction
  {
    /*!
  	 * stop_oscillator()
  	 * Stops the CPU clock and puts the processor in sleep mode.
  	 * Sleep mode can only be exited by interrupt that restarts the clock.
  	 *
  	 */
  	_no_operation();					// Just a Breakpoint
  	//CLEARB(TP_PORTOUT, TP2_MASK);   // LPM indicate going to sleep
#ifdef LOW_POWERMODE_ENABLED
  	if(!hsbActivitySlot)      // Real power save is done when no HSB activity,
  	{
  	  __bis_SR_register(LPM_BITS);    // Enter into indicated Low Power mode
  	  _no_operation();
  	  _no_operation();
  	  _no_operation();  // Allow some debug
  	}
  	// SETB(TP_PORTOUT, TP2_MASK);     // LPM Indicate Active CPU
#endif
  	_no_operation();          // Just a Breakpoint
  }
  //	There is an event, need to find from registered ones
  tEvent event = evNull;
  while(event < evLastEvent)
  if( IS_SYSTEM_EVENT(event))			// This is a macro, providing NZ if TRUE
  {
  	_disable_interrupt();
  		CLEAR_SYSTEM_EVENT(event);
  	_enable_interrupt();
  	return event;                 // return the First event found (lower number attended first)
  }
  else
  	event++;
  return evNull;	                // Never happens, unless unregistered event. Returning NULL is safer (I think)
}

/*1
 * pulseTp4()
 * This function sends a pulse to TP4, duration of pulse is errorCode x 100uS
 *
 */
void pulseTp4(BYTE errorCode)
{
  //  Generate a TP pulse
    BYTE i;
    SETB(TP_PORTOUT, TP4_MASK);       // Rising Edge of error code
    for(i=0; i< errorCode; ++i)       // Delay the Indicated Error code
      _delay_cycles(100);
    CLEARB(TP_PORTOUT, TP4_MASK);    // Falling Edge of error code

}

/*!
 * HSB Error Handler
 *
 *  The function does a cleaning of the serial port, UART status is set to its POR state.
 *  All pointers and fifo are initialized
 *
 * \returns always returns FALSE
 */
void hsbErrorHandler()
{


  //  The only place outside the isr where we access Hsb timer
  stopHsbAttentionTimer();


  // This set the UART to PUC status (interrupts disabled)
  UCA0CTL1 |=    UCSWRST;           // Reset internal SM
  _no_operation();
  UCA0CTL1  &= ~UCSWRST;            // Initialize USCI state machine
  hsbUart.hTxInter.disable();       // just to make sure (debugger doesn't match user manual)

  // Set Buffers to init
  hsbUart.bRxError = FALSE;       //  Clear Error code
  hsbActivitySlot = TRUE;         //  Keep HSB awake at all times

  // Reset Fifos
  // Init internal pointers proper allocated memory, fifo length to zero
  resetFifo(&hsbUart.rxFifo,hsbUart.fifoRxAlloc);   //
  resetFifo(&hsbUart.txFifo,hsbUart.fifoTxAlloc);   //

  //  Internal Uart status
  hsbUart.bRxError = FALSE;
  hsbUart.bNewRxChar = FALSE;
  hsbUart.bUsciTxBufEmpty = TRUE;
  hsbUart.bTxMode = FALSE;
  hsbUart.bRxFifoOverrun = FALSE;

  // Return Interrupts to proper operation
  hsbUart.hTxInter.enable();
  hsbUart.hRxInter.enable();

}

void clock_patch()
{
  do
    {
      UCSCTL7 &= ~(XT2OFFG | XT1LFOFFG | DCOFFG); // Clear XT2,XT1,DCO fault flags
      SFRIFG1 &= ~OFIFG;                          // Clear fault flags
      // Reconfig XT1 as the clk source to FLL every loop XT1, such that above fault osc is referred to XT1
      UCSCTL3 = SELREF_0  |       //  SELREF = 0  FLL reference is XT1 = 32768 Hz, XT1 should start from here
          FLLREFDIV__1;     //  FLLREFDIV=0 FLL reference div-by-1
    } while (SFRIFG1&OFIFG);                      // Test oscillator fault flag

}
/*!
 *    This routine synchronizes the startUpDataLocalNv with flash
 *    If conditions met: updateNvRam, flashWriteTimer and the HSB flashWriteEnable flag is set
 *    the local StartUpdata is synch with flash
 */
void pollSyncNvRam()
{

  if (  updateNvRam && flashWriteTimer >= (FLASH_WRITE_MS/SYSTEM_TICK_MS) && updateNvRam  &&   // Leave 2 secs between continuous writes
      flashWriteEnable)    // This condition tells that HSB is not receiving or transmitting
  {

#ifndef DISABLE_INTERNAL_FLASH_WRITE
    //
    flashWriteTimer =0;
    updateNvRam = FALSE;
    ++flashWriteCount;
    // To take real advantage of skip Hsb response, syncNvRam() should return TRUE if a real flash (erase, write) is performed
    // bBlockThisHsbResponse =syncNvRam();
    syncNvRam();
#endif
  }

}


/////////////////////////////////
#define smallBufSize 80
void main()
{
  volatile unsigned int i;
  volatile WORD hartBeatTick;
  BOOLEAN   hartCommStarted;              //!<  The flag is set to FALSE when stopHartComm() to indicate that Hart has been stopped,
                                          //  every system tick is polled to reestablished when Database is Ok again

  volatile BOOLEAN bBlockThisHsbResponse = FALSE;       // Block response if Flash enter the narrow door

  WORD hostActiveCounter =0;  //!<  This is a watchdog for Hart Message activity

  WORD  comm9900counter =0;

  BYTE hsbRecoverAttempts =0;     // We don't want to fill Astro-Med HDD up
  // see recycle #6

#ifdef HSB_SEQUENCER
  WORD  hartHsbSequenceCtr=0;                 //!< Delay to start Hart first and then Hsb (125mS ticks)
#endif

  ////////////////////////////////////////
  // Indicate a POR in pin TP3
  P1DIR |= TP3_MASK;  // set as out
  TOGGLEB(TP_PORTOUT,TP3_MASK);  // Indicate a Power UP reset sequence
    __delay_cycles(2000);
  TOGGLEB(TP_PORTOUT,TP3_MASK);  // Indicate a Power UP reset sequence
  __delay_cycles(2000);
  //////////////////////////////////

  initSystem();

  // Individual serial interrupts Settings before enter endless loop
  //  HART: RX=Enabled, TX=Enabled
  //  HSB:  RX= Disabled, TX= Enabled
  //  Global Interrupt enable from here

  // Testing HSB
  //!div-n-conq
  // 1 no HSB for now:
  hsbUart.hRxInter.enable();
  // 2 No system timer
  //TBCCTL0 &= ~CCIE;     // TRCCR0 interrupt enabled
  // 3 No signature
  // #undef DEBUG_SIGN_MAINLOOP_TOGGLE
    _enable_interrupt();    //<   Enable interrupts after all hardware and peripherals set
  volatile BYTE ch= 'A', rP=0,LoadSize=1, echo=0;
  i=0;
  CLEARB(TP_PORTOUT, TP1_MASK);     // Indicate we are running
  tEvent systemEvent;
  initHartRxSm();                   // Init Global part, static are intialized at first call
  volatile BYTE bLastRxChar;
  //  Following LOCs are the prerequisites to run using same original sw
  hartCommStarted = TRUE;           // All pre-requisites ready to start communcation with HART
  CLEARB(TP_PORTOUT,TP2_MASK);  // Start with LOW
  WORD loopTimes =0;

  volatile tEvent switchTask;
  //
  ///////////////////////////////////////////

  while(1)													// continuous loop
  {
    systemEvent = waitForEvent();
    kickWatchdog();

#ifdef DEBUG_SIGN_MAINLOOP_TOGGLE
    TOGGLEB(TP_PORTOUT, TP3_MASK);    //  Measure main LOOP scan time - optionaly signed at end of scan
#endif


  	switch(systemEvent)
  	{
  	////////////////////////////////  The HSB responds to Only ONE Event ////////////////////////////////////////////
  	// 1 EVENT //   Command buffer is ready
  	case evHsbRecComplete:
  	  _no_operation();
  	  if(bBlockThisHsbResponse)
  	    bBlockThisHsbResponse = FALSE;    // Only one response blocked per incident
  	  else
  	  {
  	    //  Main loop is too slow for a single HSB. Received data is prepared under ISR
  	    SETB(TP_PORTOUT, TP2_MASK);     // HSB message 3) Send Data to TX buffer
  	    Process9900Command(); // Returns TRUE if valid command
  	    hsbNoActivityTimer =0;
  	    //      hsbErrorHandler(4,TP4_MASK);   // Error code 4= Malformed command
  	  }
  	  break;
  	//
  	//////////////////////////////////// END of HSB Events ///////////////////////////////////////////////

  	/////////////////////////////////////////  HART Events ///////////////////////////////////////////////
  	case evHartRxChar:
  	  while(!isRxEmpty(&hartUart))   // Debug Low power mode == Missing the LRC byte, 17th char in CMD 1 & 2
  	  {
  	    //SETB(TP_PORTOUT, TP1_MASK);
  	    // ++nBytesHartRx;       // Count every received char at Hart (loop back doesn't generate and event)
  	    // Just test we are receiving a 475 Frame
  	    hartReceiver(getwUart(&hartUart));
  	    //CLEARB(TP_PORTOUT, TP1_MASK);
  	  }
  	  break;

  	case evHartRcvGapTimeout:
  	  if(bHartRecvFrameCompleted)   // Just in case a Race condition - ignore this event
  	    break;
  	  // This is an Error: Hart master transmitter exceeds maximum Gap time
  	  //Astro-Med ===>
  	  //  SETB(TP_PORTOUT, TP3_MASK);             // Indicate an GAP timer Error
  	  if(hartFrameRcvd ==FALSE)                   // Cancel current Hart Command Message, prepare to Rx a new one
  	    initHartRxSm();
  	  HartErrRegister |= GAP_TIMER_EXPIRED;   // Record the Fault
  	  //  Lets do some Debug This generates an event in AstroMed
  	  pulseTp4(2);
  	  break;

  	case evHartRcvReplyTimer:
  	  // Process the frame (same as original project)
  	  //SETB(TP_PORTOUT, TP2_MASK);     // Indicate Start of response
  	  if (commandReadyToProcess)
  	  {
  	    // clear the flag
  	    commandReadyToProcess = FALSE;
  	    // Initialize the response buffer
  	    initRespBuffer();
  	    // Process the HART command

  	    if (processHartCommand()  ) //  && !doNotRespond)    // note that doNotRespond is always FALSE as we don;t support CMD_42
  	    {
  	      // see recycle #4

  	      // recycle #7
  	      sendHartFrame();
  	      _no_operation();    // Debug number of Rx

  	      // recycle #5
  	    }
  	    else
  	    { // This command was not for this address or invalid
  	      // Get ready to start another frame
  	      initHartRxSm(); //MH: TODO: Look for side effects on Globals
  	    }
  	  }
  	  // No more need to stopReplyTimerEvent(); as is one shot
  	  //CLEARB(TP_PORTOUT, TP2_MASK);     // Indicate END of response (CPU processing)
  	  break;
  	case evHartTransactionDone:
  	  ////////////////////////////////////////////////////////////////////////////////////////////
  	  //  If HSB has been shutd-down by any error on bus or itself, we need to put it up
  	  if(bRequestHsbErrorHandle)
  	  {
        bRequestHsbErrorHandle = FALSE;
        hsbErrorHandler();             // Error code 1= Sync lost
        pulseTp4(1);
  	    break;  // We allow only ONE system event per Hart frame complete
  	  }
  	  //  MH  = 1/24/13 Logic to Set the hostActive: Any complete message sets the host indicator
  	  hostActive = TRUE;
  	  hostActiveCounter =0; // Keep resting the time-out counter


  	  //////////////////////////////////////////////////////////////////////////////////////////
  	  //
  	  //  Write to Flash is Critical Task - We don't disable interrupts but do test
  	  //  several times that HSB is not about to end reception before writing to Flash
  	  //
  	  //  1/18/13 Hart Test ULA038a - If we don't have a Hart Master with cyclic message, we need
  	  //  to syncNvRam() under another event  user case where there is no  any other
  	  if(updateNvRam)
  	    pollSyncNvRam();

  	  hartBeatTick =0;  // Indicate the presence of a Hart Master Frame
  	  break;
  	//////////////////////////// END of HART Events ///////////////////////////////////////////////

  	////////////////////////////////// SYSTEM EVENTS ///////////////////////////////////////////////
  	//                                                                                            //
  	case evTimerTick:               // System timer event - Get here every 125mS

  	  //  MH Logic to reset hostActive bit 1/24/13
  	  if( hostActiveCounter < HOST_ACTIVE_TIMEOUT && ++hostActiveCounter == HOST_ACTIVE_TIMEOUT)
  	      hostActive = FALSE;


#ifdef FORCE_FLASH_WRITE
  	  ++testWriteFlash;     /// DEBUG
#endif
  	  //  1/18/13 Hart Test ULA038a -
  	  //  If we don't have a Hart Master with cyclic messages, syncNvRam() with a timed event
  	  if(updateNvRam  &&
  	      ++hartBeatTick > HART_CONFIG_CHANGE_SYNC_TICKS )   // MH- For now just 1.5 secs after last Hart message that intends to change memory
  	    pollSyncNvRam();

  	    //  TICKS TIMERS
  	  if (NUMBER_OF_MS_IN_24_HOURS <= (dataTimeStamp +=SYSTEM_TICK_MS) )  //  dataTime stamp (mS) and rolled every 24 Hrs / Hart CMD_9
  	    dataTimeStamp = 0;
  	  ++flashWriteTimer;                                  //  Flash write stress protection
  	  SistemTick125mS =0; // Assumes no task takes more than 125mS w/o CPU attention

  	  //  Check for Oscillator Flag
  	  if(SFRIFG1&OFIFG)
  		do
  		{
  		  UCSCTL7 &= ~(XT1LFOFFG | XT2OFFG| DCOFFG);       //  Clear XT1 and DCO fault flags
  		  SFRIFG1 &= ~OFIFG;                      //  Clear fault flags
  		} while (SFRIFG1&OFIFG);                  //  While oscillator fault flag ==== YES this could be a problem

#ifdef  HSB_SEQUENCER
  	  // Receiving HSB Cmd and acting when ready doesn't need a sequence power-up
  	  //  Hart -> HSB Start sequence
  		if( hartHsbSequenceCtr < HART_HSB_SEQUENCE_DELAY && ++hartHsbSequenceCtr == HART_HSB_SEQUENCE_DELAY)
  		  hsbUart.hRxInter.enable();  // (This is Done only at power up)
#endif

  		//
  		//  Stop HART communication after 4 Sec (MAX_9900_TIMEOUT) with no 9900 communication
  		//  (only POWER-UP sequence, not sure if this needs to be checked every certain time)
  		//  stopHartComm()
  		if ( !comm9900started && ++comm9900counter > MAX_9900_TIMEOUT)  // ==> DEBUG LOW POWER MODE 0)  //// HART_ALONE_LPM, original code:
  		{
  		  comm9900counter = 0;
  		  //  Flags affected by stopHartComm(), original function also disabled Hart with HART_IE &= ~UCRXIE;
  		  //  If we are in the middle of a transmission, we just wait until TxFifo empties by itself
  		  //  hartUart.hTxDriver.disable(); can ABORT current, but no specs
  		  hartUart.hRxInter.disable();  // Disable RX interrupts
  		  hartCommStarted = FALSE;
  		  databaseOk = FALSE;
  		  updateMsgRcvd = FALSE;
  		}
  		///
  		//  This code was part of loop forever and was polled every scan, let's poll it every 125mS for now
      //  Make sure the 9900 database and updates are occurring so that we can begin HART communications
      if ((!hartCommStarted) && updateMsgRcvd && databaseOk)
      {
        // Need more intelligent trap compiler is removing === while(1);   // TRAP  HART
        //  The very first thing sis to enable interrupts and try to Flush RxFifo at End, as we may get
        //  bogus interrupts while the UART was disabled and Hart master sending data
        hartUart.hRxInter.enable();   // Start listening to Hart modem == We need to Flush RxFIFO at the end
        //
        hartCommStarted = TRUE;
        // Send the initial current mode based upon what came out of FLASH
        if (CURRENT_MODE_DISABLE == startUpDataLocalNv.currentMode)
        {
          // Tell the 9900 to go to fixed current mode at 4 mA
          setFixedCurrentMode(4.0);
          setPrimaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
          setSecondaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
        }
        else
        {
          // Tell the 9900 to go to loop reporting current mode
          setFixedCurrentMode(0.0);
          clrPrimaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
          clrSecondaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
        }
#if 0
        // ======  HART is working properly - about 24hrs ====
        //  Eventualy HSB responds late -- removing or analyzing all code with TAG: (38mS?)
        //  Return if HART fails after solving HSB
        //  12/21/12

        //!< \note  Original statement:
        // "Clean out the UART of any errors caused by a Hart Master trying to talk before the unit was ready"
        // We have saved Status and data en the hart double Rx Fifo (status, data). Uart's rx errors were cleared
        //  on the isr, just flush the RxFifo will clear flags.
        while(!isRxEmpty(&hartUart))
          getwUart(&hartUart);        // discard status,data
#endif

      }
      //
      //  Supervisory: Hart-->HSB-->monitor HSB
      //  Monitor HSB activity and request a HSB reset if inactive for 1.125 secs
      //
      // if(++testErrorHandler > 80 &&      hartHsbSequenceCtr == HART_HSB_SEQUENCE_DELAY &&
      if(hsbNoActivityTimer < HSB_NO_ACTIVITY_TIMEOUT  && !bRequestHsbErrorHandle && ++hsbNoActivityTimer >= HSB_NO_ACTIVITY_TIMEOUT
          && hsbRecoverAttempts++ <= 5)  // For Astro-Med storage we limit to 5 tries
      {
        hsbNoActivityTimer=0;           // Keep monitoring
        bRequestHsbErrorHandle = TRUE;  // Request to Flush the HSB serial port after Hart message has been handled
      }
  		break;
  	//                                                                                            //
  	//                                                                                            //
    ///////////////////////////////////// All System EVENTS  ///////////////////////////////////////
  	//
  	case evNull:
  	default:   // evNull or any non enumerated
  	  _no_operation();
  	  break;


  	}
#ifdef  DEBUG_SIGN_MAINLOOP_TOGGLE
  	// Sign the source with TP3
  	switchTask = (systemEvent == evNull) ? evLastEvent : systemEvent;
  	//
  	_disable_interrupt();
  	for(i=0; i< switchTask; ++i)
  	  TOGGLEB(TP_PORTOUT, TP3_MASK); // sign the current event
  	_enable_interrupt();
#endif

  }

}



