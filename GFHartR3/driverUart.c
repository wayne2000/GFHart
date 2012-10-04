/*!
 *  \file   driverUart.c
 *  \brief  Serial interface for GF Hart implementation
 *  Created on: Sep 20, 2012
 *  \author: MH
 */

//==============================================================================
//  INCLUDES
//==============================================================================
#include "define.h"
#include "msp_port.h"
#include "fifo.h"
#include "hardware.h"
#include "driverUart.h"
//==============================================================================
//  LOCAL DEFINES
//==============================================================================
#define HartRxFifoLen   4
#define HartTxFifoLen   6
#define HsbRxFifoLen    128
#define HsbTxFifoLen    16

// Parity settings
#define PARITY_NONE 0
#define PARITY_EVEN 1
#define PARITY_ODD  2

//==============================================================================
//  LOCAL PROTOTYPES.
//==============================================================================
static void initHartUart(void);
static void initMainUart(void);
extern BYTE hartRxfifoBuffer[], hartTxfifoBuffer[]; // defined bellow
//==============================================================================
//  GLOBAL DATA
//==============================================================================
BOOLEAN     HartRxCharError;            //!< The most current received data has error
BOOLEAN     hartTxFifoEmpty;
BOOLEAN     hartRxFrameError,           //!<  The serial isr receiver has detected errors in present serial frame
            hartNewCharInRxFifo,        //!<  New character has been moved to the Hart receiver fifo
            HartRxFifoError;            //!<  Indicates a problem in the whole Hart Receiver Fifo (overrun)

stSerialStatus HartUartCharStatus;      //!<    the status of UartChar
stSerialStatus SerialStatus;            //!<    a running summary of the UART
//
//  Let the compiler do the pointer setting
stUart
  hartUart =
  {
    HartRxFifoLen, HartTxFifoLen,           // Buffer lengths (defines)
    initHartUart,                           // points to msp430 init
    hartRxfifoBuffer, hartTxfifoBuffer,     // static allocation
    enableHartRxIntr, disableHartRxIntr,    // Rx Interrupt enable/disable inlines
    enableHartTxIntr, disableHartTxIntr,    // Tx Interrupt enable/disable inlines
    isHartTxIntrEnabled,                    // TRUE if Tx is enabled
    hartTxChar,                             // Routine writes to TXBUF when TxFifo & TXBUF empty
    //  485, Tx Control
    enableHartTxDriver, disableHartTxDriver // pointers to void func(void) that enables and disables Tx Driver
                                            // declare both NULL if no function used (rs232, rs422, etc)
  },
  hsbUart;

//==============================================================================
//  LOCAL DATA
//==============================================================================
BYTE hartRxfifoBuffer[HartRxFifoLen];    //!< Allocates static memory for Hart Receiver Buffer
BYTE hartTxfifoBuffer[HartTxFifoLen];    //!< Allocates static memory for Hart Transmitt Buffer

//==============================================================================
// FUNCTIONS
//==============================================================================

/*!
 * \function initUarts()
 * Initialize uarts HART to 1200,odd,8,1 and HSB to 19,200,odd,7,1, rx and tx fifos
 *
 * \param none
 * \return  TRUE if Uart initialization successful
 *
 */


///
///
/// Date Created: Sep 20,2012
/// Author:  MH
/// Description:
///
BOOLEAN initUarts()
{

  //    Configure the UART for Odd parity
  //
  //    Init Hart Fifos: check allocated memory and size
  if( !initFifo(&hartUart.rxFifo,HartRxFifoLen) ||
      !initFifo(&hartUart.txFifo ,HartTxFifoLen) )
    return FALSE;                                 //  Memory Problem

  resetFifo(&hartUart.rxFifo,  hartRxfifoBuffer); //  Set internal pointers
  resetFifo(&hartUart.txFifo,  hartTxfifoBuffer);
  initHartUart();                       //  UART: 1200,8,O,1

  return TRUE;

}

/*!
 * \function  initUart()
 * Initialize Uart members: fifos pointers and setsize, call the Ucsi initilization
 *
 */
///
///
/// Date Created: Sep 20,2012
/// Author:  MH
/// Description:
///
BOOLEAN initUart(stUart *pUart)
{
  // Set Fifos max length
  pUart->rxFifo.maxLength = pUart->nRxBufSize;
  pUart->txFifo.maxLength = pUart->nTxBufSize;
  //
  // Init internal pointers
  resetFifo(&pUart->rxFifo, pUart->fifoRxAlloc);
  resetFifo(&pUart->txFifo, pUart->fifoTxAlloc);
  //
  pUart->bRxError = FALSE;
  pUart->bNewRxChar = FALSE;
  pUart->bRxFifoOverrun = FALSE;
  pUart->bUsciTxBufEmpty = TRUE;
  pUart->bTxDriveEnable = FALSE;
  disableHartTxDriver();
  //
  //  Configure the msp ucsi for odd parity 1200 bps
  pUart->initUcsi(); //
  return TRUE;
}


/*!
 * \function    initHartUart()
 * Initializes the Hart Uart to 1200, 8,o,1 using SMCLK @1.048576 MHz
 *
 */
static void initHartUart(void)
{
  UCA1CTL1 =    UCSWRST;            // Reset internal SM
  UCA1CTL1 |=   UCSSEL_2;           // SMCLK as source

  UCA1BR0 = 0x6A;                   // BR= 1048576/1200 = 873.8133 ~ 874
  UCA1BR1 = 0x03;
  UCA1MCTL =0;                      // No modulation


  //  case PARITY_ODD:
  UCA1CTL0  |= UCPEN;               //  set parity
  UCA1CTL0  &= ~UCPAR;              // Odd

  UCA1CTL1  &= ~UCSWRST;            // Initialize USCI state machine
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: initMainUart()
//
// Description:
//
// Sets up the A0 UART for 19,200 baud
//
// Parameters: int - initial parity setting
//
// Return Type: void.
//
// Implementation notes:
//
// Assumes SMCLK at 1 Mhz
//
///////////////////////////////////////////////////////////////////////////////////////////
static void initMainUart(void)
{
    P3SEL = BIT4 | BIT3;                        // P3.4,3 = USCI_A0 TXD/RXD
    MAIN_CTL1 = UCSWRST;                        // reset the UART
    MAIN_CTL1 |= UCSSEL_2;                      // SMCLK source
    MAIN_BR0 = 3;                               // 1.048 MHz 19,200
    MAIN_BR1 = 0;                               // 1.048 MHz 19,200
    MAIN_MCTL = UCBRF_6 | UCBRS0 | UCOS16;      // Modln UCBRS =1, UCBRF =6, over sampling
    // Set for 7-bit data
    MAIN_CTL0 |= UC7BIT;
    // Odd is default for HART
    // case PARITY_ODD:
    MAIN_CTL0 |= UCPEN;               // setMainOddParity();
    MAIN_CTL0 &= ~UCPAR;
    MAIN_CTL1 &= ~UCSWRST;                      // **Initialize USCI state machine**
}


/*!
 * \function    HartSerialIsr()
 * Handles the Rx/Tx interrupts for Hart
 *
 */
#pragma vector=USCI_A1_VECTOR
__interrupt void hartSerialIsr(void)
{
  _no_operation(); // recommended by TI errata
  //_enable_interrupts();
  volatile BYTE data;
  switch(__even_in_range(UCA1IV,4))
  {
  case 0:
    break;                                  // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    //!MH: REMOVE process the character rxIsr();
    // get the character's status
    if(UCA1STAT & UCRXERR & UCBRK)          //  Any (FE PE OE) error or BREAK detected?
      hartUart.bRxError = TRUE;
    data = UCA1RXBUF;                       //  clear the RX interrupt flag and UCRXERR status flag
    if(!isRxFull(&hartUart))
    {
      putFifo(&hartUart.rxFifo, data);
      hartUart.bNewRxChar = TRUE;           // Signal an Event to main loop
    }
    else
      hartUart.bRxFifoOverrun = TRUE;       // Receiver Fifo overrun!!

    break;
  case 4:                                   // Vector 4 - TXIFG
    // Disable the Tx interrupt
    if(!isTxEmpty(&hartUart))
    {
      UCA1TXBUF = getFifo(&hartUart.txFifo);
      hartTxFifoEmpty = FALSE;
    }
    else
    {
        //TODO:  More informative is after tx is complete
        hartUart.bUsciTxBufEmpty = TRUE;   // Control the Tx isr - Generate a Time-out to release RTS line
        // releaseHartTx()
    }
    break;
  default:
    break;
  }
}

/*!
 * \function  putcUart(BYTE ch, stUart *pUart)
 * Put a character into the output stream
 *
 * If the output stream is full, the routine waits for an empty position. If the ostream
 * and Transmit register are empty, rotuine writes direct to transmit register.
 *
 *
 * \param ch  is the byte to be sent to the output stream
 * \param *pUart is the Uart's fifo
 *
 * \return  TRUE if character goes into the output stream
 */
BOOLEAN putcUart(BYTE ch, stUart *pUart)
{
  BOOLEAN sent = FALSE;
  while (isFull(&pUart->txFifo));       // wait until there is room in the fifo (is it worth to turn OFF power?)
  //  First Tx controls the TxDriver line
  if(pUart->txEnable != NULL &&         // this UART need Tx control
      !pUart->bTxDriveEnable)           // and its Driver is not enabled
  {
    pUart->bTxDriveEnable = TRUE;       // Raise the TxMode Flag
    pUart->txEnable();                  // CAll the actual function
  }
  // Lock the TxFifo
  __disable_interrupt();
  if(pUart->bUsciTxBufEmpty && isEmpty(&pUart->txFifo))
  {
      pUart->txChar(ch);
      pUart->bUsciTxBufEmpty = FALSE;
  }
  else
    sent = putFifo(&pUart->txFifo, ch);   // Write through the buffer
  pUart->bUsciTxBufEmpty = FALSE;         // TXSBUF has a char pending to be shifted out
  // Writting to SBUF clears the TXIF. It will be set again when SBUF moves to shift register
  if(!pUart->isTxIntrEnabled())           // If TxIntr not enabled, enable it for this frame
    pUart->txInterruptEnable();
  __enable_interrupt();

  return sent;
}

//===================================================================================================
///
/// getcUart();
/// Gets a byte from the indicated input stream. If the Rx fifo is empty, it waits here for data (TBD sleep mode??)
/// Interrupts are disabled to lock the RxFifo  (assumes interrpts are enabled)
///  \param none
//  \retval The Oldest BYTE from the RxFifo
///
BYTE getcUart(stUart *pUart)
{
  while(isEmpty(&pUart->rxFifo));        // Just wait here until a character arrives
  //    Critical Zone
  _disable_interrupt();
  BYTE ch = getFifo(&pUart->rxFifo);
  _enable_interrupt();
  return ch;
}


