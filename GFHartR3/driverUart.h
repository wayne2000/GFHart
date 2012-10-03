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


typedef struct
{
  const WORD nRxBufSize, nTxBufSize;
  void (*initUcsi)(void);
  unsigned char *fifoRxAlloc, *fifoTxAlloc;
  stFifo rxFifo, txFifo;


} stUart;



/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/
BOOLEAN initUarts();                        //!< Initialize Hart 1200,odd,8,1 and Hsb uart 19,200,odd,7,1 test memory
BOOLEAN putcUart(BYTE ch, stUart *pUart);   //!<  Put a BYTE into output stream
BYTE getcUart(stUart *pUart);               //!<  Get a BYTE from the input stream
BOOLEAN initUart(stUart *pUart);            //!<  Initialize the indicated Uart
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
extern stUart   hartUart;
/*************************************************************************
  *   $INLINE FUNCTIONS
*************************************************************************/

//=========================================INLINES===============================================

inline BOOLEAN isRxEmpty(stUart *pUart)
{
  return isEmpty(&hartUart.rxFifo);
}
inline BOOLEAN isRxFull(stUart *pUart)
{
  return isFull(&hartUart.rxFifo);
}
//
inline BOOLEAN isTxEmpty(stUart *pUart)
{
  return isEmpty(&hartUart.txFifo);
}
inline BOOLEAN isTxFull(stUart *pUart)
{
  return isFull(&hartUart.txFifo);
}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: disableTxIntr()
//
// Description:
//
// Disables the UART transmit interrupt
//
// Parameters: void
//
// Return Type: void.
//
// Implementation notes:
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////
inline void disableTxIntr(void)
{
  HART_IE &= ~UCTXIE;                // disable USCI_A0 RX interrupt
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: enableTxIntr()
//
// Description:
//
// Enables the UART transmit interrupt
//
// Parameters: void
//
// Return Type: void.
//
// Implementation notes:
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////
inline void enableTxIntr(void)
{
  HART_IE |= UCTXIE;                // Enable USCI_A0 RX interrupt
}



#endif /* DRIVERHUART_H_ */
