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
#include "hardware.h"
#include "driverUart.h"
//==============================================================================
//  LOCAL DEFINES
//==============================================================================
#define HartRxFifoLen   4
#define HartTxFifoLen   4
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
    //  Feed Tx back to RX
        {enableHartLoopBack,disableHartLoopBack,isEnabledHartLoopBack}, // NULL if no function used
    // Writes to TXBUF and clears TXIF
    hartTxChar,

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
  pUart->bTxFlowControl = TRUE;
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

  // Generate an Rx Interrupt when chars have errors
  UCA1CTL1|= UCRXEIE;

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
  _no_operation(); // recommended by TI errata -VJ (I just left Here MH)
  //
  volatile BYTE data, status;
  volatile WORD u =UCA1IV;  // Get the Interrupt source
  switch(u)
  {
  case 0:                                   // Vector 0 - no interrupt
  default:                                  //  or spurious
    break;
  case 2:                                   // Vector 2 - RXIFG
    SETB(TP_PORTOUT, TP1_MASK);
    status = UCA1STAT;
    data = UCA1RXBUF;                       // read & clears the RX interrupt flag and UCRXERR status flag
    //SETB(TP_PORTOUT, TP1_MASK);
    if( hartUart.bTxMode )                  // Loopback interrupt
    {
      if( hartUart.bUsciTxBufEmpty)         // Ignore everything but last char
      {
        //SETB(TP_PORTOUT, TP1_MASK);
        hartUart.hLoopBack.disable();           // Disable loop back
        hartUart.hTxDriver.disable();           // Disable the Tx Driver
        hartUart.bTxMode = FALSE;               // Tx is done
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

    CLEARB(TP_PORTOUT, TP1_MASK);         // signal RxDone
    break;

  case 4:                                   // Vector 4 - TXIFG
    //SETB(TP_PORTOUT, TP1_MASK); CLEARB(TP_PORTOUT, TP1_MASK); // signal for Debuging
    if(!isEmpty(&hartUart.txFifo))
    {
      hartUart.txChar(getFifo(&hartUart.txFifo));
      hartUart.bUsciTxBufEmpty = FALSE;    //  enable "chain" isrs

    }
    else
    {
      // in Half DUplex we reach this point (look oscope carefully) while moving last char
      //volatile BYTE i;      for(i=0; i < 100; ++i)        __no_operation();
      hartUart.bUsciTxBufEmpty = TRUE;
      //CLEARB(TP_PORTOUT, TP1_MASK);
      // Prepare to disable RTS line after Rx complete in next RX isr ->
      //Wrong hartUart.hLoopBack.enable();    // Enabling loopback here is 960 after the start-bit
    }

    break;
  }
}

#pragma vector=USCI_A0_VECTOR
__interrupt void hsbSerialIsr(void)
{
  while(1); // trap
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

  // Disable TxInterrupt only when no chars in TxFifo
  while (isFull(&pUart->txFifo));      // wait until there is room in the fifo TODO: worth turn OFF power?, TXIE always ON
#if 0
  // We are always having TXIE = TRUE
  if(!pUart->hTxInter.isEnabled())     // If TxIntr not enabled, enable it as we don't want infinite loop
  {
    __disable_interrupt();
      pUart->hTxInter.enable();
    __enable_interrupt();                   // Start the Tx or continue with the previous
  }
#endif
  // Lock the TxFifo
  __disable_interrupt();
  //  Handle the TxDriver if Hardware Flow-Control
  if(pUart->bTxFlowControl)
    pUart->hTxDriver.enable();                    // enable it now
  if(pUart->bUsciTxBufEmpty && isTxEmpty(pUart))  // Ask if we write direct to SBUF
  {
    hartUart.hLoopBack.enable();                  // Enable loop back TODO: be sure Rx line is not floating, add pull up in initHardware
    pUart->txChar(ch);                            // this clears TXIF
    pUart->bUsciTxBufEmpty = FALSE;
  }
  else
    sent = putFifo(&pUart->txFifo, ch);           // Write using buffer
  pUart->bTxMode = TRUE;                  // signal the Rx/Tx isr we are in TxMode
  __enable_interrupt();                   // Start the Tx or continue with the previous
  //  A TX ISR will be generated here
  return sent;
}
/*!
 * \function  writeUart(BYTE *pMem, BYTE nBytes, stUart *pUart)
 * Write nBytes bytes from the memory pointed by pMem to the output stream of pUart
 *
 * This function is intended to write a block of data to the output stream. The
 * Tx driver is controlled as indicated by members in Uart structure
 *
 * \param pMem  is the memory location to be sent to the output stream
 * \param nBytes is the amount of bytes to write
 * \param *pUart is a Uart structure whose fifo is written to. Tx control is indicated here
 *
 * \return  amount of bytes actually written
 */
WORD writeUart(BYTE *pMem, WORD nBytes, stUart *pUart)
{
  WORD i, nBytesSent;
#if 0
  if()
  for(i=0; i< nBytes; ++i )
    if(putcUart(*pMem[i], pUart)) ++nBytesSent;
#endif
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

