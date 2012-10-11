/*
 * fifo.c
 *
 *  Created on: Oct 1, 2012
 *      Author: marco henry
 */
//==============================================================================
//  INCLUDES
//==============================================================================
#include "fifo.h"
#include "string.h"
//==============================================================================
//  LOCAL DEFINES
//==============================================================================
//==============================================================================
//  LOCAL PROTOTYPES.
//==============================================================================
//==============================================================================
//  GLOBAL DATA
//==============================================================================
//==============================================================================
//  LOCAL DATA
//==============================================================================
//==============================================================================
// FUNCTIONS
//==============================================================================
///
/// resetFifo()
/// Initialize all indexes to zero, point to buffer memory location
///
/// \param pFifo This is the fifo we want to reset
/// \param pBuffer The memory block where the fifo is located
/// \return none
/// \sa Rev3
///
void resetFifo(stFifo *pFifo, BYTE *pBuffer)
{
  pFifo->buffer = pBuffer;
  pFifo->readIndex =0;
  pFifo->writeIndex =0;
  pFifo->currentLength =0;
}

///
/// putFifoFifo()
/// put the element to the indicated Fifo
///
/// \param data is the element to the fifo
/// \param pFifo a pointer to the Fifo
/// \return true if element gets into, FALSE if Fifo is full
/// \sa Rev3
///
BOOLEAN putFifo(stFifo *pFifo, BYTE data)
{
  if (pFifo->currentLength < pFifo->maxLength)
  {
    pFifo->buffer[pFifo->writeIndex++] = data;
    if(pFifo->writeIndex >= pFifo->maxLength)
      pFifo->writeIndex = 0;
    pFifo->currentLength++;
    return TRUE;
  }
  else
    return FALSE;
}

///
/// getFifo()
/// get the oldest BYTE element from the indicated Fifo
///
/// \param pFifo a pointer to the Fifo
/// \return the element. Zero if no element (test before calling)
/// \sa Rev3
///
BYTE getFifo(stFifo *pFifo)
{
  if (pFifo->currentLength != 0)
  {
    BYTE tmp = pFifo->buffer[pFifo->readIndex++];
    if (pFifo->readIndex >= pFifo->maxLength)
      pFifo->readIndex = 0;
    pFifo->currentLength--;
    return tmp;
  }
  else
    return '\0';

}
///
/// getWFifo()
/// get the oldest WORD element from the indicated Fifo
///
/// \param pFifo a pointer to the Fifo
/// \return the oldest WORD. Zero if no element (test before calling)
/// \sa Rev3
///
WORD getWFifo(stFifo *pFifo)
{
  if (pFifo->currentLength != 0)
  {
    WORD *pW = (WORD *)pFifo->buffer;
    WORD tmp = pW[pFifo->readIndex++];

    if (pFifo->readIndex >= pFifo->maxLength)
      pFifo->readIndex = 0;
    pFifo->currentLength--;
    return tmp;
  }
  else
    return '\0';

}
///
/// putWFifo()
/// put the WORD to the indicated Fifo
///
/// \param data is a WORD element to the fifo
/// \param pFifo a pointer to the Fifo
/// \return true if element gets into, FALSE if Fifo is full
/// \sa Rev3
///
BOOLEAN putWFifo(stFifo *pFifo, WORD data)
{
  if (pFifo->currentLength < pFifo->maxLength)
  {
    ((WORD *)pFifo->buffer)[pFifo->writeIndex++] = data;
    if(pFifo->writeIndex >= pFifo->maxLength)
      pFifo->writeIndex = 0;
    pFifo->currentLength++;
    return TRUE;
  }
  else
    return FALSE;
}


