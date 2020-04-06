/*
 * driverHuart.h
 *
 *  Created on: Sep 20, 2012
 *      Author: Marco.HenryGin
 */


#ifndef DRIVERHUART_H_
#define DRIVERHUART_H_
/*************************************************************************
  *   $INCLUDES
*************************************************************************/
#include "fifo.h"
#include "define.h"
/*************************************************************************
  *   $DEFINES
*************************************************************************/


///
/// the following are counters used for debugging serial communications */
///
typedef struct
{
  unsigned long TxChar;
  unsigned long RxChar;
  unsigned long FramingError;
  unsigned long ParityError;
  unsigned long OverrunError;
  unsigned long RxError;
} stSerialStatus;

typedef enum
{
  CharAvailable = 0x00,         /* character available */
  NoChar = 0x01,                /* no character available */
  ModbusSilentInterval = 0x02,  /* an RTU framing character found (not used in present project)*/
  BreakDetect = 0x10,
  OverrunError = 0x20,
  ParityError = 0x40,
  FramingError = 0x80
} stUartStatus;

/*!
 *  Provide a object-like artifact to abstract hardware uart
 *
 *
 */
typedef struct
{
  // Initialized Members - Mostly constants
  const WORD nRxBufSize, nTxBufSize;    //!< Internal buffer sizes
  void (*initUcsi)(void);               //!< Function to initialize msp Uart
  int8u  *fifoRxAlloc, *fifoTxAlloc;    //!< Pointers to static memory allocation
  const	BOOLEAN bRtsControl;           	//!< TRUE if Tx Requires hardware control \sa Tx handler functions

  const stFuncCtrl
              hRxInter, hTxInter,       //!<  Rx and Tx Interrupts handlers           //TODO: interrupt Enable/Disable is not used
              hTxDriver,                //!<  Tx driver (RTS) (off when bit ==1)
              hLoopBack;                //!<  Feed Tx back to Rx

  void (*txChar)(BYTE);                 //!<  Writes direct to TXBUF to start chained TX isrs

  // Dynamic members
  stFifo rxFifo, txFifo;                // Input and Output streams

  // Status flags
  volatile BOOLEAN
              bRxError,                 //!< A global error in Rx = cancel current Frame
              bNewRxChar,               //!< New Char has arrived

              bUsciTxBufEmpty,          //!< Status of msp430 Serial buffer (write direct to txsbuf)
              bTxMode,                  //!< When Half Duplex, ignore Rx characters but last one
              bRxFifoOverrun,           //!< Fifo overrun indicator
              bTxDriveEnable;           //!< Indicates TxMode in Half-Duplex, TRUE in Full-Duplex
} stUart;

/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/
BOOLEAN putcUart(BYTE ch, stUart *pUart);   //!<  Put a BYTE into output stream
//
//	Two Implementations for getting data from output stream, use the one
//	that matches the one used on RXISR (i.e putFifo or putwFifo)
BYTE getcUart(stUart *pUart);               //  Get a BYTE from the input stream
WORD getwUart(stUart *pUart);								//  Get a WORD from the output stream
BOOLEAN initUart(stUart *pUart);            //  Initialize the indicated Uart




/*************************************************************************
  *   $GLOBAL VARIABLES
*************************************************************************/
///
/// Global Data
///
extern BOOLEAN  hartRxFrameError,           //!<  The serial isr receiver has detected errors in present serial frame
                hartNewCharInRxFifo,        //!<  New character has been moved to the Hart receiver fifo
                hartRxFifoError,            //!<  Indicates a problem in the whole Hart Receiver Fifo (overrun)
                hartTxFifoEmpty;            //!<  The Hart Transmitter Fifo has no more bytes to send (shift reg may still be sending)
extern stUart   hartUart,
                hsbUart;

/*!
 *  hsbActivitySlot
 *  This flag indicates that HSB is in the traffic slot for the hart module,
 *  It is TRUE when Hsb attention timer prepares the reception by enabling the RXIE and FALSE after Hsb has send it last TX bit
 *  While this condition is TRUE no Low power mode is allowed
 */
extern volatile BOOLEAN   hsbActivitySlot;          //!<  This flag indicates the Flash write window opportunity for HSB
/*!
 *  flashWriteEnable
 *  This flag is evaluated at the end of Hart frame and takes into account the HSB flash write slot.
 *  Test this flag (Read Only) in the main loop before writing to flash.
 *
 */
extern volatile BOOLEAN   flashWriteEnable;

/*!
 *  iTx9900CmdBuf
 *  This a global version of the reception command index. Once the Hsb receiver ISR ends receiving data, it load its
 *  internal pointer to the global value. That way the internal state can be reset
 */
extern volatile WORD iTx9900CmdBuf;         //!<  This flag indicates the Flash write window opportunity for HSB


/*************************************************************************
  *   $INLINE FUNCTIONS
*************************************************************************/

//=========================================INLINES===============================================

/*
 * \fn isRxEmpty
 * Returns TRUE if the input stream is empty
 */
inline BOOLEAN isRxEmpty(stUart *pUart)
{

  return isEmpty(&(pUart->rxFifo));
}

/*
 * \fn isRxFull
 * Returns TRUE if the input stream is full
 */
inline BOOLEAN isRxFull(stUart *pUart)
{
  return isFull(&(pUart->rxFifo));
}

/*
 * \fn isTxEmpty
 * Returns TRUE if the output stream is empty
 */
inline BOOLEAN isTxEmpty(stUart *pUart)
{
  return isEmpty(&(pUart->txFifo));
}

/*
 * \fn isTxFull
 * Returns TRUE if the output stream is full
 */
inline BOOLEAN isTxFull(stUart *pUart)
{
  return isFull(&(pUart->txFifo));
}

/////////// Hart Rx, Tx, RxMode //////////////
/*
 * \fn disableHartRxInter
 * Disable Hart serial transmit interrupt -
 */
inline void disableHartRxIntr(void)
{
  UCA1IE &= ~UCRXIE;
}
/*
 * \fn enableHartRxInter
 * Enable Hart transmit serial interrupt -
 */
inline void enableHartRxIntr(void)
{
  UCA1IE |= UCRXIE;
}
/*
 * \fn isHartRxIntrEnabled
 * Enable Hart transmit serial interrupt -
 */
inline BOOLEAN isHartRxIntrEnabled(void)
{
  return (UCA1IE & UCRXIE) ? TRUE : FALSE;
}

/*
 * \fn disableHartTxInter
 * Disable Hart serial transmit interrupt -
 */
inline void disableHartTxIntr(void)
{
  UCA1IE &= ~UCTXIE;
}
/*
 * \fn enableHartTxInter
 * Enable Hart transmit serial interrupt -
 */
inline void enableHartTxIntr(void)
{
  UCA1IE |= UCTXIE;
}
/*
 * \fn isHartTxIntrEnabled
 * Enable Hart transmit serial interrupt -
 */
inline BOOLEAN isHartTxIntrEnabled(void)
{
  return (UCA1IE & UCTXIE) ? TRUE : FALSE;
}
//////// TxDriver
/*!
 *  \fn disableHartTxDriver()
 *  Put the hart modem in listen mode
 */
inline void disableHartTxDriver(void)
{
  HART_UART_TXCTRL_PORTOUT |= HART_UART_TXCTRL_MASK;
}
/*!
 *  \fn enableHartTxDriver()
 *  Put the hart modem in talk mode
 */
inline void enableHartTxDriver(void)
{
  HART_UART_TXCTRL_PORTOUT &= ~HART_UART_TXCTRL_MASK;
}
/*
 * \fn isHartTxIntrEnabled
 * Enable Hart transmit serial interrupt -
 */
inline BOOLEAN isEnabledHartTxDriver(void)
{
  return (HART_UART_TXCTRL_PORTOUT & HART_UART_TXCTRL_MASK) ? FALSE : TRUE;  // 1= Disabled
}
/*
 * \fn hartTxChar
 * Just an abstraction of the TXSBUF
 */
inline void hartTxChar(BYTE ch)
{
  UCA1TXBUF = ch;
}
//////// Tx/Rx Loopback
/*!
 *  \fn disableHartLoopBack()
 *  Remove internal connection between RX and TX line
 */
inline void disableHartLoopBack(void)
{
  UCA1STAT &= ~UCLISTEN;
}
/*!
 *  \fn enableHartLoopBack()
 *  Connect internally RX and TX line
 */
inline void enableHartLoopBack(void)
{
  UCA1STAT |= UCLISTEN;
}
/*
 * \fn isEnabledHartLoopBack
 * Get the status of internal TX and RX loopback
 */
inline BOOLEAN isEnabledHartLoopBack(void)
{
  return (UCA1STAT & UCLISTEN) ? TRUE : FALSE;
}
///===
/////////// HSB- High Speed Bus Rx, Tx //////////////
/*
 * \fn disableHsbRxInter
 * Disable Hsb serial transmit interrupt -
 */
inline void disableHsbRxIntr(void)
{
  UCA0IE &= ~UCRXIE;
}
/*
 * \fn enableHsbRxInter
 * Enable Hsb transmit serial interrupt -
 */
inline void enableHsbRxIntr(void)
{
  UCA0IE |= UCRXIE;
}
/*
 * \fn isHsbRxIntrEnabled
 * Enable Hsb transmit serial interrupt -
 */
inline BOOLEAN isHsbRxIntrEnabled(void)
{
  return (UCA0IE & UCRXIE) ? TRUE : FALSE;
}

/*
 * \fn disableHsbTxInter
 * Disable Hsb serial transmit interrupt -
 */
inline void disableHsbTxIntr(void)
{
  UCA0IE &= ~UCTXIE;
}
/*
 * \fn enableHsbTxInter
 * Enable Hsb transmit serial interrupt -
 */
inline void enableHsbTxIntr(void)
{
  UCA0IE |= UCTXIE;
}
/*
 * \fn isHsbTxIntrEnabled
 * Enable Hsb transmit serial interrupt -
 */
inline BOOLEAN isHsbTxIntrEnabled(void)
{
  return (UCA0IE & UCTXIE) ? TRUE : FALSE;
}
//////// Tx/Rx Loopback
/*!
 *  \fn disableHsbLoopBack()
 *  Remove internal connection between RX and TX line
 */
inline void disableHsbLoopBack(void)
{
  UCA0STAT &= ~UCLISTEN;
}
/*!
 *  \fn enableHsbLoopBack()
 *  Connect internally RX and TX line
 */
inline void enableHsbLoopBack(void)
{
  UCA0STAT |= UCLISTEN;
}
/*
 * \fn isEnabledHsbLoopBack
 * Get the status of internal TX and RX loopback
 */
inline BOOLEAN isEnabledHsbLoopBack(void)
{
  return (UCA0STAT & UCLISTEN) ? TRUE : FALSE;
}
/*
 * \fn hsbTxChar
 * Just an abstraction of the TXSBUF
 */
inline void hsbTxChar(BYTE ch)
{
  UCA0TXBUF = ch;
}

#endif /* DRIVERHUART_H_ */
