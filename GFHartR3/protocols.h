/*
 * protocols.h
 *
 *  Created on: Sep 19, 2012
 *      Author: Marco.HenryGin
 */

#ifndef PROTOCOLS_H_
#define PROTOCOLS_H_
/*************************************************************************
  *   $INCLUDES
*************************************************************************/
#include "hartr3.h"
/*************************************************************************
  *   $DEFINES
*************************************************************************/
#define XMIT_PREAMBLE_BYTES 8


// 9900 response, max size 15 bytes
#define MAX_9900_RESP_SIZE 15
// Every so many update messages, we need to remind the
// 9900 if it is in a fixed current state
#define UPDATE_REMINDER_COUNT   20


/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/
// HART Frame Handlers
void hartReceiver(WORD data);
WORD sendHartFrame (void);
//
void initHartRxSm(void);
void initRespBuffer(void);
//void rtsRcv(void);

/*************************************************************************
  *   $GLOBAL VARIABLES
*************************************************************************/
// exported HART timer variables
extern unsigned char szHartCmd [];        // The HART Command buffer
extern unsigned char szHartResp [];       // The HART response buffer
extern unsigned int respBufferSize;         // The size of the response buffer
extern int rcvBroadcastAddr;                // broadcas error received flag


// Message Counters
extern unsigned long xmtMsgCounter;
extern unsigned long errMsgCounter;
extern unsigned long numMsgProcessed;
extern unsigned long numMsgUnableToProcess;
extern unsigned long numMsgReadyToProcess;
extern unsigned char lastCharRcvd;  //!<    Last character pulled from Hart UART fifo
extern unsigned char command;       //!<    MH: Hart received command informaiton

// long address flag
extern int longAddressFlag;
// address byte index
extern unsigned char addressStartIdx;
extern unsigned char hartFrameRcvd;         //!< This Flag indicates the reception of a valid Hart Message
extern unsigned char addressValid;
extern int parityErr;
extern int overrunErr;
extern int rcvLrcError;                     // Received LRC error flag
extern WORD hartDataCount;                  //!< The number of data field bytes
extern BYTE expectedByteCnt;                //!< The received byte count, to know when we're done

// Command ready for processing flag
extern int commandReadyToProcess;
extern unsigned char hartCommand;

// flags to make sure that the loop value does not
// get reported if an update is in progress
extern unsigned char updateRequestSent;


// The HART error register
extern unsigned int HartErrRegister;

/*************************************************************************
  *   $INLINE FUNCTIONS
*************************************************************************/



#endif /* PROTOCOLS_H_ */
