/*!
 *  protocols.h
 *
 *  Created on: Sep 19, 2012
 *      Author: Marco.HenryGin
 */

#ifndef PROTOCOLS_H_
#define PROTOCOLS_H_
/*************************************************************************
  *   $INCLUDES
*************************************************************************/
#include "hart_r3.h"
/*************************************************************************
  *   $DEFINES
*************************************************************************/
#define XMIT_PREAMBLE_BYTES 8


// 9900 response, max size 15 bytes
#define MAX_9900_RESP_SIZE 15



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
//  Utilities
extern unsigned long int flashWriteCount;

// exported HART timer variables
extern unsigned char szHartCmd [];        // The HART Command buffer
extern unsigned char szHartResp [];       // The HART response buffer
extern unsigned int respBufferSize;         // The size of the response buffer
extern int rcvBroadcastAddr;                // broadcas error received flag

extern unsigned long int dataTimeStamp;     // Timer added for command 9  (type corrected 12/5/12 )
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
/*!
 * Indicates a valid command frame has been received
 *
 * This flag is set in the main loop, when the Hart Rx state machine finds that
 * the LRC is correct and a valid address for this module
 */
extern unsigned char hartFrameRcvd;
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
