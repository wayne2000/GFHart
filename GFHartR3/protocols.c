/*
 * protocols.c
 *
 *  Created on: Sep 19, 2012
 *      Author: Marco.HenryGin
 */
//==============================================================================
// INCLUDES
//==============================================================================
#include <msp430f5528.h>
#include <string.h>
#include "define.h"
#include "msp_port.h"
#include "protocols.h"
#include "driverUart.h"
#include "hartr3.h"
#include "main9900r3.h"
#include "hardware.h"
#// done include "merge.h"
#include "utilitiesr3.h"

//==============================================================================
//  LOCAL DEFINES
//==============================================================================
// Other size definitions
#define MAX_HART_XMIT_BUF_SIZE 267
#define MAX_RCV_BYTE_COUNT 267
#define MAX_HART_DATA_SIZE 255

// Defines
#define HART_PREAMBLE       0xFF
#define MIN_PREAMBLE_BYTES  2
#define MAX_PREAMBLE_BYTES  30

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






//==============================================================================
//  LOCAL PROTOTYPES.
//==============================================================================
static int isAddressValid(void);
//==============================================================================
//  GLOBAL DATA
//==============================================================================

///
/// command information
///
unsigned char hartCommand = 0xff;
unsigned char addressValid = FALSE;
unsigned char hartFrameRcvd;                //!< Flag is set after a successfully Lrc and address is for us
WORD hartDataCount;                         //!< The number of data field bytes
BYTE expectedByteCnt;                       //!< The received byte count, to know when we're done
int longAddressFlag = FALSE;              //!< long address flag
unsigned char addressStartIdx = 0;        //!< address byte index
int commandReadyToProcess = FALSE;        //!< Is the command ready to process?

unsigned long errMsgCounter = 0;          //!< Message Counters
unsigned long numMsgProcessed = 0;
unsigned long numMsgReadyToProcess = 0;
unsigned long numMsgUnableToProcess = 0;

// Parity & overrun error flags
int parityErr;
int overrunErr;
int rcvLrcError;                            //!< Did the LRC compute OK

unsigned int respBufferSize;                //!< size of the response buffer

unsigned int HartErrRegister = NO_HART_ERRORS;  //!< The HART error register
unsigned int respXmitIndex = 0;             //!< The index of the next response byte to transmit
float lastRequestedCurrentValue = 0.0;      //!< The last commanded current value from command 40 is here
int checkCarrierDetect (void);              //!< other system prototypes

unsigned long xmtMsgCounter = 0;            //!< MH: counts Hart messages at some point in SM
unsigned char szHartResp [MAX_HART_XMIT_BUF_SIZE];  //!< start w/preambles
unsigned char szHartCmd [MAX_RCV_BYTE_COUNT];       //!< Rcvd message buffer (start w/addr byte)

//==============================================================================
//  LOCAL DATA
//==============================================================================

// detect compile error static unsigned char * pRespBuffer = NULL;       //!< Pointer to the response buffer
static BOOLEAN  bInitHartSm = FALSE;



//==============================================================================
// FUNCTIONS
//==============================================================================
///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: hartReceiverSm()
//
// Description:
//
// The actual work done in the receive ISR. Checks errors, puts the received character
// into the RX queue
//
// Parameters: void
//
// Return Type: void.
//
// Implementation notes:
//
// Checks the receive error flags, as well as checks to make sure the character
// got written to the RX queue successfully
//
///////////////////////////////////////////////////////////////////////////////////////////

// Diagnostic RX error count array
unsigned int ErrReport[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// Index 0 = the number of times the rcv funct is called
// Index 1 = the number of BREAK characters processed
// Index 2 = the number of FRAMING errors
// Index 3 = the number of OVERFLOW errors
// Index 4 = the number of PARITY errors
// Index 5 = the number of excess preambles
// Index 6 = the number of insufficient preambles
// Index 7 = the number of STX errors
// Index 8 = the number of bad byte errors
// Index 9 = the number of bad LRC
// Index 10 = the number of default/idle cases
// Index 11 = the number of stored characters
// Index 12 = the number of characters used to calculate LRC
// Index 13 = the number of times the address wasn't valid
// Index 14 = SPARE


/*!
 * \function    initHartRxSm()
 *
 * This function perform the global init of Hart Receiver state machine. Ideally this
 * functionshould be implemented inside the state function, but the spread of globals
 * makes this very hard
 *
 * Implementation notes:
 * Here the global variables that were used by previous state machine are reset.
 * Other variables are initalized internally in the internal init state.
 * The signal bInitHartSm is set to perform the remainding intialization inside
 * the function, which is performed when the new character event is captured at main loop
 *
 * This function partially replaces prepareToRxFrame(), rest is done at its context
 */
void initHartRxSm(void)
{
  // Hart Reception Results - Start reply
  hartFrameRcvd = FALSE;
  commandReadyToProcess = FALSE;
  //  Status Report
  hostActive = FALSE;
  // Used in processHartCommand(), executeCommand()
  hartCommand = 0xfe;               // Make the command invalid

  // Hart Error or Status Register
  HartErrRegister = RCV_BAD_LRC;    // Clear the HART error register, except for the assumed LRC error

  // Used to calculate the Address in Hart Command message -
  addressStartIdx = 0;          // hart.c::isAddressValid()
  longAddressFlag = FALSE;
  addressValid = FALSE;         // hartCommand.c::processHartCommand ()

  // Data section of command message. First member is used everywhere
  hartDataCount = 0;            //!< The number of data field bytes
  expectedByteCnt = 0;          //!< The received byte count, to know when we're done


  // Sttus of current Cmd Message - used on processHartCommand()
  rcvLrcError = TRUE;         // Assume an error until the LRC is OK
  overrunErr = FALSE;
  parityErr = FALSE;

  // Signal the Hart Receiver State MAchine to do the rest
  bInitHartSm = TRUE;

}


/*!
 * 	hartReceiver()
 * 	Implement the Hart Receiver state machine
 * 	\param data       Receiver character (lo byte) and its status (Hi byte)
 *
 * 	\returns 	the result of the hart building
 * 	- hrsIdle					the internal SM is waiting for preamble
 * 	- hrsValidFrame		Valid frame with the address of this module has been received
 * 	- hrsDone					a valid frame and at least one extra byte has been received (current idl
 * 	- hrsErrorDump		an internal error has been detected - rest of message dumped
 * 	- hrsErrorReport  message wt this point contain errors, mus reply with a status
 * 	- hrsBusy
 *
 * 	 Implementation notes:
 * 	 The routine is called everytime a new char has arrived at Hart receiver.
 * 	 The message is built through several internal states. At power Up (or on request) the state is eRccPrepareToRx, when
 * 	 called it will set all pointers and data to prepare a new reception.
 *
 * 	 ValidFrame will transition to
 * 	 Done and Error will end
 */
void hartReceiver(WORD data)   //===> BOOLEAN HartReceiverSm() Called every time a HartRxChar event is detected
{
  /// HART receive state machine
  typedef enum
  {
    eRcvInit,                 //!<  Set the initial counters and pointer in position
    //  Follwoing states are the same as described in Documentation
    eRcvSom,                  //!<  Start of Message  - Idle waiting for Start Delimiter
    eRcvAddr,                 //!<
    eRcvCmd,
    eRcvByteCount,
    eRcvData,
    eRcvLrc,
    eRcvXtra
  } eRcvState;
  unsigned char nextByte;
  unsigned char statusReg;

  // Here come the list of Statics
  static BYTE expectedAddrByteCnt;              //!< the number of address bytes expected
  static eRcvState ePresentRcvState = eRcvSom;

  static BYTE calcLrc;
  static unsigned char rcvAddrCount;            //!< The number of address bytes received
  static unsigned char rcvByteCount;            //!< The total number of received bytes, starting with the SOM
  static WORD preambleByteCount;                //!< the number of preamble bytes received
  static unsigned char totalRcvByteCount;

  // Hart Receiver State Machine Initialization - perform before increment error counters
  if(bInitHartSm ) // Init is a pseudo state -> eRcvSom
  {
    bInitHartSm = FALSE;  // Initialization done
    // locals
    expectedByteCnt = calcLrc = rcvByteCount = rcvAddrCount = totalRcvByteCount = 0;
    preambleByteCount = 0;
    //pRespBuffer = szHartResp;     // Set the transmit pointer back to the beginning of the buffer
    //  stop (if running) the Reply timer
    stopReplyTimerEvent();
    ePresentRcvState = eRcvSom;   // Set the state machine to look for the start of message
  }
  //  Process Received Character
  nextByte = data;            //  !MH:--> HART_RXBUF;
  statusReg = data >>8;       //  debugging HART_STAT;
  kickHartRecTimers();        //  kick the Gap and Response timers, they will generate wake-up events to main loop
  //
  ++ErrReport[0];
  hostActive = TRUE;      //  If we are receiving characters, the host is active
  //
  // Check for errors before calling the state machine, since any comm error resets the state machine
  //
  //  BRK:  Break Detected (all data, parity and stop bits are low)
  if (statusReg & UCBRK)      //
    ++ErrReport[1];           // Status was cleared when happened, just report here
  //  FE:   Frame error (low stop bit)
  if (statusReg & UCFE)
  {
    HartErrRegister |= RCV_FRAMING_ERROR;
    ++ErrReport[2];
    ++startUpDataLocalV.errorCounter[0];
  }
  //  OE: Buffer overrun (previous rx overwritten)
  if (statusReg & UCOE)
  {
    // if we're still receiving preamble bytes, just clear the OE flag otherwise, bail on the reception
    if (eRcvSom != ePresentRcvState)
    {
      overrunErr = TRUE;
      HartErrRegister |= BUFFER_OVERFLOW;
      ++ErrReport[3];
      ++startUpDataLocalV.errorCounter[2];
    }
    else
      ++startUpDataLocalV.errorCounter[14];

  }
  //  PE: Parity Error
  if (statusReg & UCPE)
  {
    //  if the parity error occurs after the command byte, we will respond with a tx error message.
    //  Otherwise, just ignore the message
    parityErr = TRUE;
    HartErrRegister |= RCV_PARITY_ERROR;
    ++ErrReport[4];
    ++startUpDataLocalV.errorCounter[1];
  }
  // increment the total byte count
  totalRcvByteCount++;
  // The receive state machine: What we do depends on the current state
  switch (ePresentRcvState)
  {
  case eRcvSom:
    // A parity or framing error here is fatal, so check
    if ((statusReg & UCPE) || (statusReg & UCFE))
    {
      ++startUpDataLocalV.errorCounter[10];
      // We are not going to respond, so we will wait until the next message starts

      // 1) prepareToRxFrame();
      initHartRxSm();
      return;
    }
    // is it a preamble character?
    if (HART_PREAMBLE == nextByte)
    {
      //intrDcd = TRUE;
      // increment the preamble byte count
      ++preambleByteCount;
      // Check for too many preamble characters
      if (MAX_PREAMBLE_BYTES < preambleByteCount)
      {
        // Set the HART error register
        HartErrRegister |= EXCESS_PREAMBLE;
        // Set the state machine to Idle, because we're not processing any more bytes on this frame - TO BE VERIFIED
        //  2) prepareToRxFrame();
        initHartRxSm();
        ++ErrReport[5];
        ++startUpDataLocalV.errorCounter[4];
      }
      // Do not store the character
      return;
    }
    else
    if (STX == (nextByte & (FRAME_MASK | EXP_FRAME_MASK)))
    {
      if (MIN_PREAMBLE_BYTES > preambleByteCount)
      {
        // Set the HART error register
        HartErrRegister |= INSUFFICIENT_PREAMBLE;
        // If we haven't seen enough preamble bytes, we are not going to respond at all, so set the state machine to idle
        //  3)prepareToRxFrame();
        initHartRxSm();
        errMsgCounter++;
        ++ErrReport[6];
        ++startUpDataLocalV.errorCounter[3];
        return;  // If < 2 preambles, do nothing & return
      }
      // How many address bytes to expect?
      expectedAddrByteCnt = (nextByte & LONG_ADDR_MASK) ? LONG_ADDR_SIZE : SHORT_ADDR_SIZE;
      addressStartIdx = rcvByteCount+1; // the first address byte is the next character
      // is this a long address?
      longAddressFlag = (nextByte & LONG_ADDR_MASK) ? TRUE : FALSE;
      // change the state
      ePresentRcvState = eRcvAddr;
      // Start Checking time between chars as a valid frame has started
      startGapTimerEvent();

    }
    else
    {
      // Set the HART error register
      HartErrRegister |= STX_ERROR;
      // Increment the error counters
      ++ErrReport[7];
      errMsgCounter++;
      ++startUpDataLocalV.errorCounter[5];
      // Something's wrong. Just set the preamble count back to zero and start over after recording the error diagnostics
      //  4) prepareToRxFrame();
      initHartRxSm();
      return;
    }
    break;
  case eRcvAddr:
    // A parity, overrun or framing error here is fatal, so check
    if ((statusReg & UCPE) || (statusReg & UCFE) || (statusReg & UCOE))
    {
      // We are not going to respond, so we will wait until the next message starts
      // 5) prepareToRxFrame();
      initHartRxSm();
      ++startUpDataLocalV.errorCounter[11];
      return;
    }
    // increment the count of the address bytes rcvd
    ++rcvAddrCount;
    if (rcvAddrCount == expectedAddrByteCnt)
    {
      // Store the last byte of the address here to
      // make sure that isAddressValid() will work correctly. Do NOT
      // increment rcvByteCount!!
      szHartCmd[rcvByteCount] = nextByte;
      // Check to see if the address is for us. If not, start
      // looking for the beginning of the next message
      addressValid = isAddressValid();
      if (!addressValid)
      {
        ++ErrReport[13];
        ++startUpDataLocalV.errorCounter[15];
        // x) prepareToRxFrame(); - BMD removed. Do not start to look for a new frame until the current one is completely done
        // MH: Agree with BMD
        return;
      }
      // we're done with address, move to command
      ePresentRcvState = eRcvCmd;
    }
    break;
  case eRcvCmd:
    // ready to receive byte count
    ePresentRcvState = eRcvByteCount;
    // signal that the command byte has been received, and we have to process the command
    commandReadyToProcess = TRUE;
    numMsgReadyToProcess++;
    // Now that we have to respond, set the error register. We'll clear it later if all is OK
    HartErrRegister |= RCV_BAD_LRC;
    break;
  case eRcvByteCount:
    // A parity, overrun or framing error here is fatal, so check
    if ((statusReg & UCPE) || (statusReg & UCFE) || (statusReg & UCOE))
    {
      ++startUpDataLocalV.errorCounter[12];
      // We are not going to respond, so we will wait until the next message starts
      // 6) prepareToRxFrame();
      initHartRxSm();
      return;
    }
    expectedByteCnt = nextByte;
    if (expectedByteCnt > MAX_HART_DATA_SIZE) //MH This never happens, as left value is byte and right is 255
    {
      HartErrRegister |= RCV_BAD_BYTE_COUNT;
      // 7) prepareToRxFrame();
      initHartRxSm();
      ++ErrReport[8];
      errMsgCounter++;
      ++startUpDataLocalV.errorCounter[7];
    }
    else
      if (0 == expectedByteCnt)
        ePresentRcvState = eRcvLrc;
      else
      {
        hartDataCount = 0;
        ePresentRcvState = eRcvData;
      }
    break;
  case eRcvData:
    ++hartDataCount;
    // Are we done?
    if (hartDataCount == expectedByteCnt)
      ePresentRcvState = eRcvLrc;
    break;
  case eRcvLrc:
    // We are using the single hartFrameRcvd flag for Gap/Reply
    stopGapTimerEvent();      // extras are don't cares for hart command message
    startReplyTimerEvent();   // Now we have enough data to send a Reply

    if (calcLrc == nextByte)
    {
      HartErrRegister &= ~RCV_BAD_LRC;  // Clear the bad CRC error
      rcvLrcError = FALSE;              // process the command
    }
    else
    {
      rcvLrcError = TRUE;
      HartErrRegister |= RCV_BAD_LRC;
      ++ErrReport[9];
      ++startUpDataLocalV.errorCounter[6];
    }
    // If the address is for us, pick up extra characters
    // If the message isn't for us, start looking for a new message asap
    if (addressValid)
    {
      hartFrameRcvd = TRUE;
      ePresentRcvState = eRcvXtra;
    }
    else      // 8) prepareToRxFrame();
      initHartRxSm();
    break;
  case eRcvXtra:
    HartErrRegister |= EXTRA_CHAR_RCVD;
    //#ifdef STORE_EXTRA_CHARS
    if (FALSE == checkCarrierDetect())
      HartErrRegister |= EXTRA_CHAR_RCVD;
    break;
    //#else#endif
  default:
    ++ErrReport[10];
    // 9) prepareToRxFrame();   // Get ready for the next message
    initHartRxSm();
    return;               // Do not store the character
  }
  // Here we build the Hart Command Buffer & calc LRC - Not idle, or msg cancelled
  if (MAX_RCV_BYTE_COUNT > rcvByteCount)  // Make sure we don't overrun the buffer
  {
    ++ErrReport[11];
    szHartCmd[rcvByteCount] = nextByte;
    ++rcvByteCount;
  }
  calcLrc ^= nextByte;  // calculateLrc(nextByte);
  ++ErrReport[12];
}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: checkCarrierDetect()
//
// Description:
//
// Checks the state of the CD pin. Return TRUE if the carrier is detected, FALSE otherwise
//
// Parameters: void
//
// Return Type: int
//
// Implementation notes:
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////
int checkCarrierDetect (void)
{
    int rtnVal;
    // read port 1
    int port1value;

    port1value = P1IN;
    // mask the value of the port
    rtnVal = (port1value & BIT2) ? TRUE : FALSE;
    return rtnVal;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: isAddressValid()
//
// Description:
//
// Verify that the address is either a broadcast address or is an exact match
// to the poll address in the database. Returns TRUE if either conditions
// met, FALSE otherwise
//
// Parameters:
//     unsigned char * pCommand:  pointer to the HART command to check the address of.
//
// Return Type: unsigned int.
//
// Implementation notes:
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////
// HART 7 compliant
static int isAddressValid(void)
{
  union
  {
    unsigned int i;
    unsigned char b[2];
  } myDevId;
  int index;
  // Mask the Primary bit out of the poll address
  unsigned char pollAddress = szHartCmd[addressStartIdx] & ~(PRIMARY_MASTER | BURST_MODE_BIT);
  // Now capture if it is from the primary or secondary master
  startUpDataLocalV.fromPrimary = (szHartCmd[addressStartIdx] & PRIMARY_MASTER) ? TRUE : FALSE;
  int isValid = FALSE;

  // Set the broadcast flag to FALSE
  rcvBroadcastAddr = FALSE;
  // Remove the Burst mode bit
  szHartCmd[addressStartIdx] &= ~BURST_MODE_BIT;
  if (longAddressFlag)
  {
    // We only need to compare to the lower 14 bits
    myDevId.i = startUpDataLocalV.expandedDevType & EXT_DEV_TYPE_ADDR_MASK;
    // Now XOR out the recieved address for each
    myDevId.b[1] ^= szHartCmd[addressStartIdx] & POLL_ADDR_MASK;
    myDevId.b[0] ^= szHartCmd[addressStartIdx+1];
    // If the ID is 0, check for a unique address
    if (!myDevId.i)
    {
      // Now compare the last 3 bytes of address
      if (!(memcmp(&szHartCmd[addressStartIdx+2], &startUpDataLocalNv.DeviceID, 3)))
      {
        isValid = TRUE;
      }
    }
    // If the address is not a unique address for me,
    // look for the broadcast address
    if (!isValid)
    {
      // Check to see if it is a broadcast. We have to check the first byte
      // separately, since it may have the primary master bit set.
      // Mask out the primary master bit of the first byte
      if (0 != (szHartCmd[addressStartIdx] & ~PRIMARY_MASTER))
      {
        return isValid;
      }
      // Now check the following 4 bytes
      for (index = 1; index < LONG_ADDR_SIZE; ++index)
      {
        // the rest of the bytes are 0 if this is a broadcast address
        if(0 != szHartCmd[addressStartIdx+index])
        {
          return isValid;
        }
      }
      rcvBroadcastAddr = TRUE;
      isValid = TRUE;
    }
  }
  else  // short Polling address for Cmd0?
  {
    if (startUpDataLocalNv.PollingAddress == pollAddress)
    {
      isValid = TRUE;
    }
  }
  return isValid;
}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: initRespBuffer()
//
// Description:
//
// Set up the initial response buffer
//
// Parameters: void
//
// Return Type: void.
//
// Implementation notes:
//      In addition to setting up the beginning of the response buffer, this function also
//      captures some information used in later procesing
//
///////////////////////////////////////////////////////////////////////////////////////////
void initRespBuffer(void)
{
  int i, addrSize;

  // Delimiter - Mask out expansion bytes & physical layer type bits
  szHartResp[0] = ACK | (szHartCmd[0] & (STX | LONG_ADDR_MASK));
  // Copy Address field
  addrSize = (szHartCmd[0] & LONG_ADDR_MASK) ? LONG_ADDR_SIZE : SHORT_ADDR_SIZE;
  for (i = 1; i <= addrSize; ++i)
  {
    szHartResp[i] = szHartCmd[i];
  }
  // Now remove the burst mode bit
  szHartResp[1] &= ~BURST_MODE_BIT;
  // Command byte
  hartCommand = szHartResp[i] = szHartCmd[i];
  // set frame offset for building the command to the next position
  respBufferSize = i + 1;
}
#if 0
///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: rtsRcv()
//
// Description:
//
// Set the RTS line high (receive mode)
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
/* inline */ void rtsRcv(void)
{
    P4OUT |= BIT0;
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: sendHartFrame()
//
// Description:
//
// Starts the transmit of the completed response
//
// Parameters: void
//
// Return Type:  Number of bytes sent to output stream
//
// Implementation notes:
//  pRespBuffer
//
//
///////////////////////////////////////////////////////////////////////////////////////////
WORD sendHartFrame (void)
{

  WORD i;
  volatile BYTE calcLrc =0;
  _no_operation();	// debug point
  //  Send preambles
  for(i=0; i < XMIT_PREAMBLE_BYTES; ++i)
    putcUart(HART_PREAMBLE, &hartUart);// write the character to Hart outstream

  // Send the response buffer
  WORD nTotal = (szHartResp[0] & LONG_ADDR_MASK) ?    \
      (szHartResp[LONG_COUNT_OFFSET] + LONG_COUNT_OFFSET) :   \
      (szHartResp[SHORT_COUNT_OFFSET] + SHORT_COUNT_OFFSET);
  for(i=0; i<= nTotal; ++i)	// nTotal+1 iterations because need to include nData byte itself
  {
    calcLrc ^= szHartResp[i];              // Calculate the LRC
    putcUart(szHartResp[i], &hartUart);    // Transmit the character
  }
  // Send calculated Lrc
  putcUart(calcLrc, &hartUart);
  // count the transmitted message
  xmtMsgCounter++;
  // Clear the appropriate cold start bit
  if (startUpDataLocalV.fromPrimary)

    clrPrimaryStatusBits(FD_STATUS_COLD_START);
  else
    clrSecondaryStatusBits(FD_STATUS_COLD_START);
  //TODO: Need to investigate all this section in Hart
  //  If cmdReset is set and we are here, we should not respond again until we emerge from the reset,
  //  so set the do not respondflag
  if (TRUE == cmdReset)
    doNotRespond = TRUE;
  // Get ready for next frame
  // 10) prepareToRxFrame();
  initHartRxSm();
  return nTotal +2 + XMIT_PREAMBLE_BYTES; //  Frame total size = Frame + 1 + LRC + Preambles
}



