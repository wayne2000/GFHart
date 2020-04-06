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
#include "main9900_r3.h"
//==============================================================================
//  LOCAL DEFINES
//==============================================================================
/*!
 *   Rx buffer is just for contention when CPU is busy and to avoid overrun
 */
#define hartRxFifoLen   8
/*!
 * We would like to write the whole message to buffer and go to sleep
 */
#define hartTxFifoLen   80
/*!
 *  Max Rx data is 67 chars for the 50mS time slot plus gap and tolerance (+10%)
 */
#define hsbRxFifoLen    80
/*!
 *  Max Tx data is 32 chars for the 50mS time slot plus gap and tolerance (+10%)
 */
#define hsbTxFifoLen    32


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

extern volatile WORD iTx9900CmdBuf;         //!<  Index for transmit the received command in the RX ISR

stSerialStatus HartUartCharStatus;      //!<    the status of UartChar
stSerialStatus SerialStatus;            //!<    a running summary of the UART
//
//  Let the compiler do the pointer setting
/*!
 *  hartUart is the Uart instance for Hart communication
 */
stUart  hartUart =
  {
    hartRxFifoLen,                          //!< Rx Buffer lengths (defines)
    hartTxFifoLen,                          //!< Tx Buffer lengths (defines)
    initHartUart,                           //!< points to msp430 init
    hartRxfifoBuffer,                       //!< static allocation for the Rx Fifo
    hartTxfifoBuffer,                       //!< static allocation  for the Tx Fifo
    TRUE,																		//!<	bRtsControl = TRUE

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

  };

/*!
 *  hsbUart is the Uart instance for High Speed Bus communication
 */
stUart  hsbUart =
  {
      hsbRxFifoLen,                         //!< Rx Buffer lengths (defines)
      hsbTxFifoLen,                         //!< Tx Buffer lengths (defines)
      initHsbUart,                          //!< points to msp430 init
      hsbRxfifoBuffer,                      //!< static allocation for the Rx Fifo
      hsbTxfifoBuffer,			                //!< static allocation for the Tx Fifo
      FALSE,																//	bRtsControl = FALSE, we don't have a TxDriver
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

volatile BOOLEAN  hsbActivitySlot = TRUE;             //!<  Indicates HSB active or preparing to RX frame, No low power while this is TRUE
volatile BOOLEAN  flashWriteEnable = FALSE;           //!<  Indicates to the main loop that it is the best time to write to Flash

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
 *  \fn     BOOLEAN initUart(stUart *pUart)
 *  \brief  Initialize the indicated Uart
 *
 *  Init members: fifos pointers and set buffer size, call the Ucsi initilization to set baud rate, parity, clk source
 *  If the uart requires RTS control, the disable() function is called (constructor)
 *  \param  pUart points to the uart structure
 *
 * Date Created: Sep 20,2012
 * Author:  MH
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
  pUart->bUsciTxBufEmpty = TRUE;
  pUart->bTxMode = FALSE;
  pUart->bRxFifoOverrun = FALSE;
  pUart->bTxDriveEnable = FALSE;

  //	If Uart requires RTS control provide it here
  if(	pUart->bRtsControl && pUart->hTxDriver.disable != NULL)
  	pUart->hTxDriver.disable();
  pUart->initUcsi();              //  Configure the msp ucsi for odd parity 1200 bps
  return TRUE;
}


/*!
 * \fn    initHartUart()
 * \brief Initializes the Hart Uart to 1200, 8,o,1 using
 *  - SMCLK @1.048576 MHz  (if HART_UART_USES_SMCLK defined)
 *  - ACLK @32.768KHz (if defined HART_UART_USES_ACLK == Product will use this option)
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

  //  Generate an Rx Interrupt when chars have errors
  UCA1CTL1|= UCRXEIE;
  //
  ///////////////

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
 * \fn    hartSerialIsr()
 * \brief Handles the Rx/Tx interrupts for Hart
 *
 * This is the Rx/Tx interrupt for the Hart at USCI_A1_VECTOR (46)
 *
 */
#pragma vector=USCI_A1_VECTOR
__interrupt void hartSerialIsr(void)
{
  _no_operation();      // recommended by TI errata -VJ (I just left Here MH)
  static volatile WORD rxword;
  static volatile WORD u ;
  static BYTE status;
  // see recycle #2

  u =UCA1IV;            // Get the Interrupt source

  switch(u)
  {
  case 0:                                   // Vector 0 - no interrupt
  default:                                  //  or spurious
    break;
  case 2:                                   // Vector 2 - RXIFG


  	rxword = (status = UCA1STAT) << 8 | UCA1RXBUF;    // read & clears the RX interrupt flag and UCRXERR status flag
    if( hartUart.bTxMode )                  // Loopback interrupt
    {
      if( hartUart.bUsciTxBufEmpty)         // Ignore everything but last char
      {
        hartUart.hLoopBack.disable();             // Disable loop back
        hartUart.hTxDriver.disable();             // Disable the Tx Driver
        hartUart.bTxMode = FALSE;                 // Tx is done
        SET_SYSTEM_EVENT(evHartTransactionDone);  // Signal the end of command-reply transaction
        // see recycle #3

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        //  12/27/12 -  Releasing the RTS line makes RX lines goes from low to hi after some time (4mS observed)
        //  ON-A519 Modem says 4 consecutive carrier clock cycles - 1.8 to 3.3mS (depending on 1200 or 2200Hz)
        //  This RX transition is detected as a bad reception or good one, depending on the transition time of RX.
        //  After TX is done, we need to clean the RX to avoid receiving the extra char, easiest way is to use
        //  the UCSWRST software reset bit to init uart internal state machine. This affects also the TX, but it
        //  safe. The RX being low is not detected, as msp430 will looks for edges to establish a start bit.
        UCA1CTL1 |=    UCSWRST;           // Reset internal SM
        _no_operation();
        UCA1CTL1  &= ~UCSWRST;            // Initialize USCI state machine
        // Need to manually enable interrupts again
        hartUart.hRxInter.enable();
        hartUart.hTxInter.enable();
        //
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        CLEARB(TP_PORTOUT, TP1_MASK);             // Indicate Hart ends (move bellow to test the Uart reset)

        //  Flash Write sync
        flashWriteEnable = hsbActivitySlot ? FALSE : TRUE;  // The combination of HSB and Hart conditions to write to Flash

      }
    }
    else
    {
      // Uart is in Recvd Mode
      kickHartGapTimer();                       //  kick the Gap timer as soon as received the 11th bit
      //  1/15/2013 We read everything (error included) and take decisions at later on as response depends on error location
      if(!isRxFull(&hartUart))                //  put data in input stream if no errors
      {
        if(status & UCRXERR || status & UCBRK)    //  Any (FE PE OE) error?  or ==== NO BREAK detected==== 12/26/12
        {
          hartUart.bRxError = TRUE;               //  ==> power save ==> discard current frame
          //TOGGLEB(TP_PORTOUT, TP3_MASK);        //  catch errors: errors observed when shorting/disconnecting Hart, no errors on protocol
        }
        hartUart.bNewRxChar = putwFifo(&hartUart.rxFifo, rxword);  // Signal an Event to main loop
        SET_SYSTEM_EVENT(evHartRxChar);
        // see recycle #1
      }
      else
        hartUart.bRxFifoOverrun = TRUE;       // Receiver Fifo overrun!!
    }
    break;

  case 4:                                   // Vector 4 - TXIFG
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
      // Prepare to disable RTS line after Rx complete in next RX isr ->
      //Wrong hartUart.hLoopBack.enable();    // Enabling loopback here is 960 after the start-bit
    }

    break;
  }
#ifdef LOW_POWERMODE_ENABLED
  _bic_SR_register_on_exit(LPM_BITS);
  _no_operation();    //
  _no_operation();
  _no_operation();

#endif
}

/*!
 * \fn    hsbSerialIsr()   USCI_A0_VECTOR (56)
 * Handles the Rx/Tx interrupts for High Speed Bus
 *
 */
#pragma vector=USCI_A0_VECTOR
__interrupt void hsbSerialIsr(void)
{
	_no_operation(); // recommended by TI errata -VJ (I just left Here MH)
	//
	volatile BYTE rxbyte;
	volatile BYTE status;
	static volatile BOOLEAN hsbMsgInProgress = FALSE;
	static volatile BYTE bLastRxChar;
	static volatile WORD i9900CmdBuf;       // This is the local Index

	BOOLEAN volatile rxError = FALSE;     //  Take same action for Rx error at the end
	volatile WORD u =UCA0IV;              //  Get the Interrupt source
	switch(u)
	{
		case 0:                                   // Vector 0 - no interrupt
	  default:                                  //  or spurious
	    break;
	  case 2:                                   // Vector 2 - RXIFG
	    status = UCA0STAT;
	    rxbyte = UCA0RXBUF;                     // read & clears the RX interrupt flag and UCRXERR status flag
	    //
	    if( hsbUart.bTxMode )                   // Loopback interrupt
	    {
	      if( hsbUart.bUsciTxBufEmpty)          // Ignore everything but last char
	      {
	        hsbUart.hLoopBack.disable();        // Disable loop back
	        hsbUart.bTxMode = FALSE;            // Tx is done
	        hsbUart.hRxInter.disable();         //  ===== THIS comes from TX ====
	        //  HSB has finished here
	        hsbActivitySlot = FALSE;            //  HSB for Hart has ended, allow to sleep and ignore any traffic until wake-up
	        hsbMsgInProgress = FALSE;           //  Everything has been shifted out
	        CLEARB(TP_PORTOUT,TP2_MASK);        // HSB message 4) End of HSB message

	      }
	    }
	    else
	    if(status & UCRXERR || status & UCBRK )   // Any (FE PE OE) error or BREAK error ?
	      rxError = TRUE;
	    else
	    {
	      // Code for reception of a character UART-error free:
	      if(hsbMsgInProgress)
	      {
	        //  Bare bone reception is done here
	        if(rxbyte == HART_MSG_END)
	        {
	          SET_SYSTEM_EVENT(evHsbRecComplete);
	          CLEARB(TP_PORTOUT, TP2_MASK);         // HSB message 2) All characters in buffer
	          hsbMsgInProgress = FALSE;
	          //  hsbUart.hRxInter.disable();       //  We can't do this here - need to disable at TX shift out

	        }
	        else  // Keep storing rx chars
	          if(i9900CmdBuf < MAX_9900_CMD_SIZE)
	            sz9900CmdBuffer[i9900CmdBuf++] = rxbyte;
	          else
	            rxError = TRUE;   //  command buffer overrun
	      }
	      else
	      if( bLastRxChar == ATTENTION && rxbyte == HART_ADDRESS )  // We get the start of a $H
	      {
	        startHsbAttentionTimer();       // This will Enable RXIE again before Attention arrives
	        SETB(TP_PORTOUT, TP2_MASK);     // HSB message 1) Start detected $H
	        hsbMsgInProgress = TRUE;
	        //  To be compatible with what Process9900Command() requires first two be '$','H'
	        sz9900CmdBuffer[0] = ATTENTION;
	        i9900CmdBuf=1;
	        sz9900CmdBuffer[i9900CmdBuf++] = HART_ADDRESS;
	      }
	      bLastRxChar = rxbyte;   // The sequence of last two chars is the signature
	    }
	    break;

	  case 4:                                   // Vector 4 - TXIFG
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
	      // Prepare to disable RTS line after Rx complete in next RX isr ->
	      //Wrong hsbUart.hLoopBack.enable();    // Enabling loopback here is 960 after the start-bit
	    }
	    break;
	} // end switch
	//

	// All Hsb UART error actions are taken here - We allow TX to send its previous (Abort not implemented)
	//  -- 12/28/12 Added a Flag to skip the RXBUF wrong data after sleeping
	if(rxError && !hsbUart.bTxMode)
	{
	  // TOGGLEB(TP_PORTOUT, TP3_MASK);        //  catch errors: Errors are detected when shorting HSB bus, no errors due protocol handling
	  hsbUart.bRxError = TRUE;              //  signal the error
	  bLastRxChar =0;                       //  Reset start sequence
	  hsbActivitySlot = TRUE;               //  HSB do not sleep, listen everything

	  // In error condition This is my basic Failure State Recover
	  //  1) UART should be listening but buffers set to init conditions
	  i9900CmdBuf =0;
	  // Loog for the $H again
	  hsbMsgInProgress = FALSE;             //  Reset the simple two-state machine
	}

#ifdef LOW_POWERMODE_ENABLED
	_bic_SR_register_on_exit(LPM_BITS);
	_no_operation();    //
	_no_operation();
	_no_operation();
#endif
}

/*!
 * \fn  putcUart(BYTE ch, stUart *pUart)
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
  while (isFull(&pUart->txFifo));      // (38mS?) wait until there is room in the fifo TODO: worth turn OFF power?, TXIE always ON
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
    pUart->hLoopBack.enable();                  // Enable loop back TODO: be sure Rx line is not floating, add pull up in initHardware --Remove a BUG
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
 * \fn  BYTE getcUart(stUart *pUart)
 * \brief Gets a byte from the indicated input stream.
 *
 *  Gets a byte from the indicated input stream. If the Rx fifo is empty, it waits here for data (TBD sleep mode??)\n
 *  Interrupts are disabled to lock the RxFifo  (assumes interrupts are always enabled)
 *
 *  \param  pUart pointer to the uart
 *  \retval Oldest BYTE from the RxFifo
 *
 */
BYTE getcUart(stUart *pUart)
{
  while(isEmpty(&pUart->rxFifo));        // (38mS?) Just wait here until a character arrives
  //    Critical Zone
  _disable_interrupt();
  BYTE ch = getFifo(&pUart->rxFifo);
  _enable_interrupt();
  return ch;
}
/*!
 * \fn  WORD getwUart(stUart *pUart)
 * \brief Gets a WORD from the indicated input stream.
 *
 *  Gets a word from the indicated input stream. If the Rx fifo is empty, it waits here for data (TBD sleep mode??)\n
 *  Interrupts are disabled to lock the RxFifo  (assumes interrupts are always enabled)
 *
 *  \param  pUart pointer to the uart
 *  \retval Oldest WORD from the RxFifo
 *  \sa putwUart()
 *
 */
WORD getwUart(stUart *pUart)
{
  while(isEmpty(&pUart->rxFifo));        // (38mS?) Just wait here until a character arrives
  //    Critical Zone
  _disable_interrupt();
  WORD u = getwFifo(&pUart->rxFifo);
  _enable_interrupt();
  return u;
}
// Remove inlines

