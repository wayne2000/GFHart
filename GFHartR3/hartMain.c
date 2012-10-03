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
etEvents etSystemEvent;
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
 *  First I check if it was an incoming character, if so I assume an ETP command and add an EVENT_ETP_RX to the execute list.\n
/// Then I check if it was a telemetry command that completed, if so I add an EVENT_TELEMETRY_COMPLETE to the execute list.\n
/// Then I check if it was the tick timer that timed out, and if so I move all events that are ripe [ready to execute] to the execute list. \n
/// Finally I check if it was an impedance sample ready to AD convert, and if so I add an EVENT_IMPEDANCE_SAMPLED to the execute list.
/// Note!!! that I do this check as the very last one to make sure it gets into the first position of the execute list [items added
/// to the beginning of the list]
 *
 */
/// \n
///
etEvents waitForEvent()
{
  etEvents this;
  while(isRxEmpty(&hartUart));//      p_uart->is_rx_empty()) && all events here
  return this;
}

void main(void)
{
  volatile unsigned int i;
  low_power =1;
  //CLEARB(TP_PORTOUT, TP1_MASK);  // Indicate we are running
  initSystem();
  if(low_power)
  {
    __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3, enable interrupts
    __no_operation();                         // For debugger
  }
  else
    _enable_interrupt();    //<   Enable interrupts after all hardware and peripherals set
  while(1)                                  // continuous loop
  {
    etSystemEvent =waitForEvent();
    switch(etSystemEvent)
    {
    case evHartRxChar:
      break;

    }

    for(i=5000;i>0;i--);                   // Delay
    //P1OUT ^=0x01;
  }
}

