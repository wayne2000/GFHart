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
#define HartRxFifoLen   64
#define HartTxFifoLen   32
#define HsbRxFifoLen    64
#define HsbTxFifoLen    32

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
    //  Rx Interrupt Handlers
        {enableHartRxIntr,disableHartRxIntr,isHartRxIntrEnabled},
    //  Tx Interrupt Handlers
        {enableHartTxIntr,disableHartTxIntr,isHartTxIntrEnabled},
    //  Tx Driver
        {enableHartTxDriver,disableHartTxDriver,isEnabledHartTxDriver}, // NULL if no function used

    hartTxChar,                             // Routine writes to TXBUF when TxFifo & TXBUF empty

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
  pUart->bHalfDuplex = TRUE;
  pUart->hTxDriver.disable();
  pUart->initUcsi();              //  Configure the msp ucsi for odd parity 1200 bps
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
  static BOOLEAN TxCompleted = FALSE;
  _no_operation(); // recommended by TI errata
  //_enable_interrupts();
  volatile BYTE data, status;
  switch(__even_in_range(UCA1IV,4))
  {
  case 0:
    break;                                  // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    status = UCA1STAT;
    data = UCA1RXBUF;                       // read & clears the RX interrupt flag and UCRXERR status flag
    if(hartUart.bTxMode)                    // Ignore everything but last char
    {
      if(TxCompleted)   // This is the reception of last Txmitted
      {
        hartUart.hTxDriver.disable();             // Disable the Tx Driver
        hartUart.bTxMode = TxCompleted = FALSE;   // Done
      }
    }
    else
    if(status & UCRXERR & UCBRK)            //  Any (FE PE OE) error or BREAK detected?
        hartUart.bRxError = TRUE;           //  ==> power save ==> discard current frame
    else
    if(!isRxFull(&hartUart))                //  put data in input stream if no errors
      hartUart.bNewRxChar = putFifo(&hartUart.rxFifo, data);  // Signal an Event to main loop
    else
      hartUart.bRxFifoOverrun = TRUE;       // Receiver Fifo overrun!!
    break;

  case 4:                                   // Vector 4 - TXIFG
    if(!isTxEmpty(&hartUart))
    {
      UCA1TXBUF = getFifo(&hartUart.txFifo);
      if(hartUart.bHalfDuplex && isTxEmpty(&hartUart))
        TxCompleted = TRUE;             // Signal receiver to
    }
    else
      hartUart.hTxInter.disable();        // Done with interrupts for this frame
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
 * and Transmit register are empty, routine writes direct to transmit register.
 *
 *
 * \param ch  is the byte to be sent to the output stream
 * \param *pUart is the Uart's fifo
 *
 * \return  TRUE if character goes into the output stream or direct to txbuf
 */
BOOLEAN putcUart(BYTE ch, stUart *pUart)
{
  BOOLEAN sent = FALSE;
  while (isFull(&pUart->txFifo));       // wait until there is room in the fifo (is it worth to turn OFF power?)

  // Lock the TxFifo
  __disable_interrupt();
  //  Handle the TxDriver && TxMode if HalfDuplex Mode
  if(pUart->bHalfDuplex)
  {
    pUart->bTxMode = TRUE;
    pUart->hTxDriver.enable();            // enable it now
  }
  sent = putFifo(&pUart->txFifo, ch);     // Write through the buffer
  // Handle the Tx Interrupt control
  if(!pUart->hTxInter.isEnabled())        // If TxIntr not enabled, enable it for this frame
    pUart->hTxInter.enable();
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

// Remove inlines

/*
 * \function disableHartRxInter
 * Disable Hart serial transmit interrupt -
 */
void disableHartRxIntr(void)
{
  UCA1IE &= ~UCRXIE;
}
/*
 * \function enableHartRxInter
 * Enable Hart transmit serial interrupt -
 */
/*inline*/ void enableHartRxIntr(void)
{
  UCA1IE |= UCRXIE;
}
/*
 * \function isHartRxIntrEnabled
 * Enable Hart transmit serial interrupt -
 */
/*inline*/ BOOLEAN isHartRxIntrEnabled(void)
{
  return (UCA1IE & UCRXIE) ? TRUE : FALSE;
}

/*
 * \function disableHartTxInter
 * Disable Hart serial transmit interrupt -
 */
/*inline*/ void disableHartTxIntr(void)
{
  UCA1IE &= ~UCTXIE;
}
/*
 * \function enableHartTxInter
 * Enable Hart transmit serial interrupt -
 */
/*inline*/ void enableHartTxIntr(void)
{
  UCA1IE |= UCTXIE;
}
/*
 * \function isHartTxIntrEnabled
 * Enable Hart transmit serial interrupt -
 */
/*inline*/ BOOLEAN isHartTxIntrEnabled(void)
{
  return (UCA1IE & UCTXIE) ? TRUE : FALSE;
}
//////// TxDriver
/*!
 *  \function disableHartTxDriver()
 *  Put the hart modem in listen mode
 */
/*inline*/ void disableHartTxDriver(void)
{
  HART_UART_TXCTRL_PORTOUT |= HART_UART_TXCTRL_MASK;
}
/*!
 *  \function enableHartTxDriver()
 *  Put the hart modem in talk mode
 */
/*inline*/ void enableHartTxDriver(void)
{
  HART_UART_TXCTRL_PORTOUT &= ~HART_UART_TXCTRL_MASK;
}
/*
 * \function isHartTxIntrEnabled
 * Enable Hart transmit serial interrupt -
 */
/*inline*/ BOOLEAN isEnabledHartTxDriver(void)
{
  return (HART_UART_TXCTRL_PORTOUT & HART_UART_TXCTRL_MASK) ? FALSE : TRUE;  // 1= Disabled
}
/*
 * \function hartTxChar
 * Just an abstraction of the TXSBUF
 */
/*inline*/ void hartTxChar(BYTE ch)
{
  UCA1TXBUF = ch;
}

