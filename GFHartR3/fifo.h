/*!
 * \file    fifo.h
 *  \brief  Provides basic FIFO operations for both byte and word fifos
 *
 *  Created on: Oct 1, 2012
 *      Author: MH
 */

#ifndef FIFO_H_
#define FIFO_H_
/*************************************************************************
  *   $INCLUDES
*************************************************************************/
#include "define.h"
/*************************************************************************
  *   $DEFINES
*************************************************************************/
///
/// Define a general Fifo structure
///
typedef struct
{
  BYTE *buffer;                     //!<    Data storage
  WORD  readIndex, writeIndex;      //!<    Separate indexes for accessing the buffer
  WORD  maxLength, currentLength;   //!<    Defines the maximum element length  and current size
} stFifo;


/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/
// Basic Fifo operations
BYTE getFifo(stFifo *pFifo);
BOOLEAN putFifo(stFifo *pFifo, BYTE data);

WORD getwFifo(stFifo *pFifo);
BOOLEAN putwFifo(stFifo *pFifo, WORD data);

void resetFifo(stFifo *pFifo, BYTE *pBuffer);
BOOLEAN initFifo(stFifo *pFifo, WORD byteSize);

/*************************************************************************
  *   $GLOBAL VARIABLES
*************************************************************************/

/*************************************************************************
  *   $INLINE FUNCTIONS
*************************************************************************/
inline BOOLEAN isFull(stFifo *pFifo)
{
  return (pFifo->currentLength == pFifo->maxLength);
}
//===================================================================================================
inline BOOLEAN isEmpty(stFifo *pFifo)
{
  return (pFifo->currentLength == 0);
}


#endif /* FIFO_H_ */
