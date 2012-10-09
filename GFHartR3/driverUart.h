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


typedef struct
{
  // Initialized Members - Mostly constants
  const WORD nRxBufSize, nTxBufSize;    //!< Internal buffer sizes
  void (*initUcsi)(void);               //!< Function to initialize msp Uart
  int8u  *fifoRxAlloc, *fifoTxAlloc;    //!< Pointers to static memory allocation

  const stFuncCtrl
              hRxInter, hTxInter,       //!<  Rx and Tx Interrupts handlers
              hTxDriver;                //!<  Tx driver (RTS) (off when bit ==1)

  void (*txChar)(BYTE);                 //!< Bypass Fifo to speed up transmitt

  // Dynamic members
  stFifo rxFifo, txFifo;                // Input and Output streams

  // Status flags
  volatile BOOLEAN
              bRxError,                 //!< A global error in Rx = cancel current Frame
              bNewRxChar,               //!< New Char has arrived
              bHalfDuplex,              //!< Half-full duplex mode
              bUsciTxBufEmpty,          //!< Status of msp430 Serial buffer (write direct to txsbuf)
              bTxMode,                  //!< When Half Duplex, ignore Rx characters but last one
              bRxFifoOverrun,           //!< Fifo overrun indicator
              bTxDriveEnable;           //!< Indicates TxMode in Half-Duplex, TRUE in Full-Duplex
} stUart;

/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/
BOOLEAN initUarts();                        //!< Initialize Hart 1200,odd,8,1 and Hsb uart 19,200,odd,7,1 test memory
BOOLEAN putcUart(BYTE ch, stUart *pUart);   //!<  Put a BYTE into output stream
BYTE getcUart(stUart *pUart);               //!<  Get a BYTE from the input stream
BOOLEAN initUart(stUart *pUart);            //!<  Initialize the indicated Uart

extern void enableHartRxIntr(void);
extern void disableHartRxIntr(void);
extern BOOLEAN isHartRxIntrEnabled(void);

extern void enableHartTxDriver(void);
extern void disableHartTxDriver(void);
extern BOOLEAN isHartTxIntrEnabled(void);

extern void enableHartTxIntr(void);
extern void disableHartTxIntr(void);
extern BOOLEAN isEnabledHartTxDriver(void);
extern void hartTxChar(BYTE ch);



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

/*************************************************************************
  *   $INLINE FUNCTIONS
*************************************************************************/

//=========================================INLINES===============================================

/*
 * \function isRxEmpty
 * Returns TRUE if the input stream is empty
 */
inline BOOLEAN isRxEmpty(stUart *pUart)
{
  return isEmpty(&hartUart.rxFifo);
}

/*
 * \function isRxFull
 * Returns TRUE if the input stream is full
 */
inline BOOLEAN isRxFull(stUart *pUart)
{
  return isFull(&hartUart.rxFifo);
}

/*
 * \function isTxEmpty
 * Returns TRUE if the output stream is empty
 */
inline BOOLEAN isTxEmpty(stUart *pUart)
{
  return isEmpty(&hartUart.txFifo);
}

/*
 * \function isTxFull
 * Returns TRUE if the output stream is full
 */
inline BOOLEAN isTxFull(stUart *pUart)
{
  return isFull(&hartUart.txFifo);
}

/////////// Hart Rx, Tx, RxMode //////////////
/*
 * \function disableHartRxInter
 * Disable Hart serial transmit interrupt -
 */


#endif /* DRIVERHUART_H_ */
