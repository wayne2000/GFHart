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
#include "hartMain.h"
//==============================================================================
//  LOCAL DEFINES
//==============================================================================
/*  Rx Buffer is just for contention when CPU is busy and avoid overrun     */
#define hartRxFifoLen   8
/* We would like to write the whole message to buffer and go to sleep */
#define hartTxFifoLen   80

#define hsbRxFifoLen    64
#define hsbTxFifoLen    32

// Parity settings
#define PARITY_NONE 0
#define PARITY_EVEN 1
#define PARITY_ODD  2

//==============================================================================
//  LOCAL PROTOTYPES.
//==============================================================================
static void initHartUart(void);
static void initHsbUart(void);
extern BYTE hartRxfifoBuffer[], hartTxfifoBuffer[],
						hsbRxfifoBuffer[], 	hsbTxfifoBuffer[]; 		// defined bellow
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
    hartRxFifoLen, hartTxFifoLen,           // Buffer lengths (defines)
    initHartUart,                           // points to msp430 init
    hartRxfifoBuffer, hartTxfifoBuffer,     // static allocation
    TRUE,																		//	bRtsControl = TRUE

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
  hsbUart =
  {
      hsbRxFifoLen, hsbTxFifoLen,           // Buffer lengths (defines)
      initHsbUart,                          // points to msp430 init
      hsbRxfifoBuffer, hsbTxfifoBuffer,			// static allocation
      FALSE,																//	bRtsControl = TRUE
      //  Rx Interrupt Handlers
          {enableHsbRxIntr,disableHsbRxIntr,isHsbRxIntrEnabled},
      //  Tx Interrupt Handlers
          {enableHsbTxIntr,disableHsbTxIntr,isHsbTxIntrEnabled},
      //  Tx Driver
          {NULL, NULL, NULL}, 							// Hsb has no RTS or TX Driver
      //  Feed Tx back to RX
          {enableHsbLoopBack,disableHsbLoopBack,isEnabledHsbLoopBack}, // NULL if no function used
      // Writes to TXBUF and clears TXIF
          hsbTxChar

    };

//==============================================================================
//  LOCAL DATA
//==============================================================================
BYTE hartRxfifoBuffer[hartRxFifoLen*2];   //!< Allocates static memory for Hart Receiver Buffer (data, status)
BYTE hartTxfifoBuffer[hartTxFifoLen];     //!< Allocates static memory for Hart Transmit Buffer
BYTE hsbRxfifoBuffer[hsbRxFifoLen];				//!< Allocates static memory for High Speed Serial Bus receiver buffer
BYTE hsbTxfifoBuffer[hsbTxFifoLen];				//!< Allocates static memory for High Speed Serial transmit Buffer
//==============================================================================
// FUNCTIONS
//==============================================================================


/*!
 * \function  initUart()
 *
 * Date Created: Sep 20,2012
 * Author:  MH
 * Description:	Initialize Uart members: fifos pointers and setsize, call the Ucsi initilization
 *
 *
 *
 */
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
  //	If Uart requires RTS control provide it here
  if(	pUart->bRtsControl && pUart->hTxDriver.disable != NULL)
  	pUart->hTxDriver.disable();
  pUart->initUcsi();              //  Configure the msp ucsi for odd parity 1200 bps
  return TRUE;
}


/*!
 * \function    initHartUart()
 * Initializes the Hart Uart to 1200, 8,o,1 using
 *  - SMCLK @1.048576 MHz or
 *  -ACLK @32.768KHz
 *
 */
static void initHartUart(void)
{
  UCA1CTL1 =    UCSWRST;            // Reset internal SM
#ifdef  HART_UART_USES_SMCLK
  #ifdef HART_UART_USES_ACLK
    #error Define only one clk source
  #endif
    UCA1CTL1 |=   UCSSEL_2;           // SMCLK as source
    UCA1BR0 = 0x6A;                   // BR= 1048576/1200 = 873.8133 ~ 874
    UCA1BR1 = 0x03;
    UCA1MCTL =0;                      // No modulation
#else
  #ifdef HART_UART_USES_ACLK
    UCA1CTL1 |=   UCSSEL_1;           // ACLK as source ==> This will be the preferred if LPM3
    UCA1BR0 = 0x1B;                   // BR= 32768/1200 = 27.306 = 0x1B + 2/8
    UCA1BR1 = 0x00;
    UCA1MCTL = UCBRS_2;               // Second modulation stage = 2  (.306 * 8 ~aprox 2)
  #else
    #error Define either HART_UART_USES_ACLK or HART_UART_USES_SMCLK for Hart UART
  #endif

#endif


  //  case PARITY_ODD:
  UCA1CTL0  |= UCPEN;               //  set parity
  UCA1CTL0  &= ~UCPAR;              // Odd

  // Generate an Rx Interrupt when chars have errors
  UCA1CTL1|= UCRXEIE;

  UCA1CTL1  &= ~UCSWRST;            // Initialize USCI state machine
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: initHsbUart()
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
static void initHsbUart(void)
{
    P3SEL = BIT4 | BIT3;                        // P3.4,3 = USCI_A0 TXD/RXD
    UCA0CTL1 = UCSWRST;                        // reset the UART
    UCA0CTL1 |= UCSSEL_2;                      // SMCLK source
    UCA0BR0 = 3;                               // 1.048 MHz 19,200
    UCA0BR1 = 0;                               // 1.048 MHz 19,200
    UCA0MCTL = UCBRF_6 | UCBRS0 | UCOS16;      // Modln UCBRS =1, UCBRF =6, over sampling
    // Set for 7-bit data
    UCA0CTL0 |= UC7BIT;
    // Odd is default for HART
    // case PARITY_ODD:
    UCA0CTL0 |= UCPEN;               // setMainOddParity();
    UCA0CTL0 &= ~UCPAR;
    UCA0CTL1 &= ~UCSWRST;                      // **Initialize USCI state machine**
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
  volatile WORD rxword;
  volatile WORD u =UCA1IV;  // Get the Interrupt source
  BYTE status;
  switch(u)
  {
  case 0:                                   // Vector 0 - no interrupt
  default:                                  //  or spurious
    break;
  case 2:                                   // Vector 2 - RXIFG
  	//SETB(TP_PORTOUT, TP1_MASK);
    rxword = (status = UCA1STAT) << 8 | UCA1RXBUF;    // read & clears the RX interrupt flag and UCRXERR status flag
    //SETB(TP_PORTOUT, TP1_MASK);
    if( hartUart.bTxMode )                  // Loopback interrupt
    {
      if( hartUart.bUsciTxBufEmpty)         // Ignore everything but last char
      {
        //SETB(TP_PORTOUT, TP1_MASK);
        hartUart.hLoopBack.disable();           // Disable loop back
        hartUart.hTxDriver.disable();           // Disable the Tx Driver
        hartUart.bTxMode = FALSE;               // Tx is done
        SET_SYSTEM_EVENT(evHartTransactionDone);  // Signal the end of command-reply transaction
      }
    }
    else
    if(status & UCRXERR & UCBRK)            //  Any (FE PE OE) error or BREAK detected?
        hartUart.bRxError = TRUE;           //  ==> power save ==> discard current frame
    else
    if(!isRxFull(&hartUart))                //  put data in input stream if no errors
    {
    	hartUart.bNewRxChar = putwFifo(&hartUart.rxFifo, rxword);  // Signal an Event to main loop
    	SET_SYSTEM_EVENT(evHartRxChar);
    }

    else
      hartUart.bRxFifoOverrun = TRUE;       // Receiver Fifo overrun!!

    //CLEARB(TP_PORTOUT, TP1_MASK);         // signal RxDone
    break;

  case 4:                                   // Vector 4 - TXIFG
    //SETB(TP_PORTOUT, TP1_MASK); CLEARB(TP_PORTOUT, TP1_MASK); // signal for Debuging
    SET_SYSTEM_EVENT(evHartTxChar);         // Send an event to main loop, to send next char
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
  _low_power_mode_off_on_exit();
}

#pragma vector=USCI_A0_VECTOR
__interrupt void hsbSerialIsr(void)
{
	_no_operation(); // recommended by TI errata -VJ (I just left Here MH)
	//
	volatile BYTE rxbyte;
	volatile WORD u =UCA0IV;  // Get the Interrupt source
	BYTE status;
	switch(u)
	{
		case 0:                                   // Vector 0 - no interrupt
	  default:                                  //  or spurious
	    break;
	  case 2:                                   // Vector 2 - RXIFG
	    status = UCA0STAT;
	    rxbyte = UCA0RXBUF;    // read & clears the RX interrupt flag and UCRXERR status flag
	    if( hsbUart.bTxMode )                  // Loopback interrupt
	    {
	      if( hsbUart.bUsciTxBufEmpty)         // Ignore everything but last char
	      {
	        hsbUart.hLoopBack.disable();           // Disable loop back
	        hsbUart.bTxMode = FALSE;               // Tx is done
	        SET_SYSTEM_EVENT(evHsbTransactionDone);  // Signal the end of command-reply transaction
	      }
	    }
	    else
	    if(status & UCRXERR & UCBRK)            //  Any (FE PE OE) error or BREAK detected?
	        hsbUart.bRxError = TRUE;           	//  ==> power save ==> discard current frame
	    else
	    if(!isRxFull(&hsbUart))                	//  put data in input stream if no errors
	    {
	    	hsbUart.bNewRxChar = putFifo(&hsbUart.rxFifo, rxbyte);  // Signal an Event to main loop
	    	SET_SYSTEM_EVENT(evHsbRxChar);
	    }

	    else
	      hsbUart.bRxFifoOverrun = TRUE;       // Receiver Fifo overrun!!

	    break;

	  case 4:                                   // Vector 4 - TXIFG
	    SET_SYSTEM_EVENT(evHsbTxChar);         // Send an event to main loop, to send next char
	    if(!isEmpty(&hsbUart.txFifo))
	    {
	      hsbUart.txChar(getFifo(&hsbUart.txFifo));
	      hsbUart.bUsciTxBufEmpty = FALSE;    //  enable "chain" isrs

	    }
	    else
	    {
	      // in Half DUplex we reach this point (look oscope carefully) while moving last char
	      //volatile BYTE i;      for(i=0; i < 100; ++i)        __no_operation();
	      hsbUart.bUsciTxBufEmpty = TRUE;
	      //CLEARB(TP_PORTOUT, TP1_MASK);
	      // Prepare to disable RTS line after Rx complete in next RX isr ->
	      //Wrong hsbUart.hLoopBack.enable();    // Enabling loopback here is 960 after the start-bit
	    }
	    break;
	  }
	  _low_power_mode_off_on_exit();
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
  if(pUart->bRtsControl)
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

//===================================================================================================
///
/// getcUart();
/// Gets a byte from the indicated input stream. If the Rx fifo is empty, it waits here for data (TBD sleep mode??)
/// Interrupts are disabled to lock the RxFifo  (assumes interrupts are always enabled)
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
//===================================================================================================
///
/// getwUart();
/// Gets a word from the indicated input stream. If the Rx fifo is empty, it waits here for data (TBD sleep mode??)
/// Interrupts are disabled to lock the RxFifo  (assumes interrupts are always enabled)
///  \param none
//  \retval The Oldest BYTE from the RxFifo
///
WORD getwUart(stUart *pUart)
{
  while(isEmpty(&pUart->rxFifo));        // Just wait here until a character arrives
  //    Critical Zone
  _disable_interrupt();
  WORD u = getwFifo(&pUart->rxFifo);
  _enable_interrupt();
  return u;
}
// Remove inlines

