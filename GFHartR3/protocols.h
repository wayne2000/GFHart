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


// Defines
#define HART_PREAMBLE       0xFF
#define MIN_PREAMBLE_BYTES  2
#define MAX_PREAMBLE_BYTES  30
#define XMIT_PREAMBLE_BYTES 8
// SOM delimiter defines
#define BACK 0x01
#define STX 0x02
#define ACK 0x06
#define LONG_ADDR_MASK 0x80
#define FRAME_MASK 0x07
#define EXP_FRAME_MASK  0x60
#define LONG_ADDR_SIZE 5
#define SHORT_ADDR_SIZE 1
#define LONG_COUNT_OFFSET 7
#define SHORT_COUNT_OFFSET 3

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
void sendHartFrame (void);
//
void initHartRxSm(void);
void initRespBuffer(void);
void rtsRcv(void);

/*************************************************************************
  *   $GLOBAL VARIABLES
*************************************************************************/
// exported HART timer variables
extern unsigned char szHartCmd [];        // The HART Command buffer
extern unsigned char szHartResp [];       // The HART response buffer
extern unsigned int respBufferSize;         // The size of the response buffer
extern int rcvBroadcastAddr;                // broadcas error received flag

extern long dataTimeStamp;                  // Timer added for command 9
extern float lastRequestedCurrentValue;     // The laast commanded current from command 40

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
extern int8u loopMode;

// Device Variable Status for PV
extern unsigned char PVvariableStatus;


// The HART error register
extern unsigned int HartErrRegister;

/*************************************************************************
  *   $INLINE FUNCTIONS
*************************************************************************/





#endif /* PROTOCOLS_H_ */
