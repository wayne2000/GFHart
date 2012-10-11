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
#include "hart.h"
#include "Main9900.h"

//==============================================================================
//  LOCAL DEFINES
//==============================================================================
// Other size definitions
#define MAX_HART_XMIT_BUF_SIZE 267
#define MAX_RCV_BYTE_COUNT 267
#define MAX_HART_DATA_SIZE 255

//==============================================================================
//  LOCAL PROTOTYPES.
//==============================================================================
//==============================================================================
//  GLOBAL DATA
//==============================================================================
HART_TIMER dllTimer;                    //!< Provides Gap and Response Hart timing
HART_TIMER mainMsgTimer;
///
/// command information
///
unsigned char hartCommand = 0xff;
unsigned char addressValid = FALSE;


int longAddressFlag = FALSE;              //!< long address flag
unsigned char addressStartIdx = 0;        //!< address byte index
int commandReadyToProcess = FALSE;        //!< Is the command ready to process?

unsigned long errMsgCounter = 0;          //!< Message Counters
unsigned long numMsgProcessed = 0;
unsigned long numMsgReadyToProcess = 0;
unsigned long numMsgUnableToProcess = 0;

// Parity & overrun error flags
int parityErr = FALSE;
int overrunErr = FALSE;

unsigned char hartFrameRcvd;                //!< Flag is set after a successfully Lrc and address is for us
eXmitState ePresentXmitState = eXmitIdle;   //!< Set the State machines to idle state
unsigned char szLrc = 0;                    //!< calculated LRC byte
unsigned int hartDataCount = 0;             //!< The number of data field bytes

unsigned char expectedAddrByteCnt = 0;      //!< the number of address bytes expected
unsigned char expectedByteCnt = 0;          //!< The received byte count, to know when we're done
unsigned int respBufferSize;                //!< size of the response buffer
int rcvLrcError = FALSE;                    //!< Did the LRC compute OK
unsigned int HartErrRegister = NO_HART_ERRORS;  //!< The HART error register
unsigned int respXmitIndex = 0;             //!< The index of the next response byte to transmit
float lastRequestedCurrentValue = 0.0;      //!< The last commanded current value from command 40 is here
int checkCarrierDetect (void);              //!< other system prototypes

unsigned long xmtMsgCounter = 0;            //!< MH: counts Hart messages at some point in SM
unsigned char szHartResp [MAX_HART_XMIT_BUF_SIZE];  //!< start w/preambles
unsigned char lastCharRcvd = 0;

unsigned char szHartCmd [MAX_RCV_BYTE_COUNT];       //!< Rcvd message buffer (start w/addr byte)

//==============================================================================
//  LOCAL DATA
//==============================================================================
static unsigned char rcvAddrCount = 0;             //!< The number of address bytes received
static unsigned char rcvByteCount = 0;             //!< The total number of received bytes, starting with the SOM
static int preambleByteCount = 0;                  //!< the number of preamble bytes received
static eRcvState ePresentRcvState = eRcvSom;
static unsigned char totalRcvByteCount;
static unsigned char * pRespBuffer = NULL;       //!< Pointer to the response buffer

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

///
/// hartReceiverSm()
///
/// Implements the Hart receiver state machine.
/// (start with the minimal: message building)
///
void hartReceiverSm (void)   //===> BOOLEAN HartReceiverSm() Called every time a HartRxChar event is detected
{
    unsigned char nextByte;
    unsigned char statusReg;

    ++ErrReport[0];
    // If we are receiving characters, the host is active
    hostActive = TRUE;
    // restart the timer
    startDllTimer();
    // Check for errors before calling the state machine, since any comm error
    // resets the state machine
    statusReg = 0;  // MH for now let's assume no erors - Need to move down this debugging HART_STAT;
    if (statusReg & UCBRK)      //!MH:   Break Detected (all data, parity and stop bits are low)
    {
        // Just clear the status register, since this break is expected
        HART_STAT &= ~(UCBRK | UCFE | UCOE | UCPE | UCRXERR);
        ++ErrReport[1];
        // return;
    }
    // Now capture the character
    nextByte = getByteHart();   //!MH:--> HART_RXBUF;
    if (statusReg & UCFE)       //!MH:  Frame error (low stop bit)
    {
        HartErrRegister |= RCV_FRAMING_ERROR;
        ++ErrReport[2];
        ++startUpDataLocalV.errorCounter[0];
    }
    if (statusReg & UCOE)       //!MH:  Buffer overrun (previous rx overwritten)
    {
        // if we're still receiving preamble bytes, just clear the OE flag
        // otherwise, bail on the reception
        if (eRcvSom != ePresentRcvState)
        {
            overrunErr = TRUE;
            HartErrRegister |= BUFFER_OVERFLOW;
            ++ErrReport[3];
            ++startUpDataLocalV.errorCounter[2];
        }
        else
        {
            HART_STAT &= ~UCOE;
            ++startUpDataLocalV.errorCounter[14];
        }
    }
    if (statusReg & UCPE)       //MH:   Parity Error
    {
        // if the parity error occurs after the command byte, we will respond
        // with a tx error message. Otherwise, just ignore the message
        parityErr = TRUE;
        HartErrRegister |= RCV_PARITY_ERROR;
        ++ErrReport[4];
        ++startUpDataLocalV.errorCounter[1];
    }

    lastCharRcvd = nextByte;

    // increment the total byte count
    totalRcvByteCount++;
    // The receive state machine
    // What we do depends on the current state
    switch (ePresentRcvState)
    {
    case eRcvSom:
        // A parity or framing error here is fatal, so check
        if ((statusReg & UCPE) || (statusReg & UCFE))
        {
            ++startUpDataLocalV.errorCounter[10];
            // We are not going to respond, so we will wait until the next message starts
            prepareToRxFrame();
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
                // Set the state machine to Idle, because we're not processing
                // any more bytes on this frame - TO BE VERIFIED
                prepareToRxFrame();
                ++ErrReport[5];
                ++startUpDataLocalV.errorCounter[4];
            }
            // Do not store the character
            return;
        }
        else if (STX == (nextByte & (FRAME_MASK | EXP_FRAME_MASK)))
        {
            if (MIN_PREAMBLE_BYTES > preambleByteCount)
            {
                // Set the HART error register
                HartErrRegister |= INSUFFICIENT_PREAMBLE;
                // If we haven't seen enough preamble bytes, we are not
                // going to respond at all, so set the state machine to
                // idle
                prepareToRxFrame();
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
        }
        else
        {
            // Set the HART error register
            HartErrRegister |= STX_ERROR;
            // Increment the error counters
            ++ErrReport[7];
            errMsgCounter++;
            ++startUpDataLocalV.errorCounter[5];
            // Something's wrong. Just set the preamble count back to zero and
            // start over after recording the error diagnostics
            prepareToRxFrame();
            return;
        }
        break;
    case eRcvAddr:
        // A parity, overrun or framing error here is fatal, so check
        if ((statusReg & UCPE) || (statusReg & UCFE) || (statusReg & UCOE))
        {
            // We are not going to respond, so we will wait until the next message starts
            prepareToRxFrame();
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
                //prepareToRxFrame(); - BMD removed. Do not start to look
                // for a new frame until the current one is completely done
                return;
            }
            // we're done with address, move to command
            ePresentRcvState = eRcvCmd;
        }
        break;
    case eRcvCmd:
        // ready to receive byte count
        ePresentRcvState = eRcvByteCount;
        // signal that the command byte has been received, and we have to
        // process the command
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
            prepareToRxFrame();
            return;
        }
        expectedByteCnt = nextByte;
        if (expectedByteCnt > MAX_HART_DATA_SIZE)
        {
            HartErrRegister |= RCV_BAD_BYTE_COUNT;
            prepareToRxFrame();
            ++ErrReport[8];
            errMsgCounter++;
            ++startUpDataLocalV.errorCounter[7];
        }
        else if (0 == expectedByteCnt)
        {
            ePresentRcvState = eRcvLrc;
        }
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
        {
            ePresentRcvState = eRcvLrc;
        }
        break;
    case eRcvLrc:
        if (szLrc == nextByte)
        {
            // Clear the bad CRC error
            HartErrRegister &= ~RCV_BAD_LRC;
            // process the command
            rcvLrcError = FALSE;
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
        else
        {
            prepareToRxFrame();
        }
        break;
    case eRcvXtra:
        HartErrRegister |= EXTRA_CHAR_RCVD;
//#ifdef STORE_EXTRA_CHARS
        if (FALSE == checkCarrierDetect())
        {
            HartErrRegister |= EXTRA_CHAR_RCVD;
        }
        break;
//#else#endif
    default:
        ++ErrReport[10];

        // Get ready for the next message
        prepareToRxFrame();
        // Do not store the character
        return;
    }
    // Make sure we don't overrun the buffer
    if (MAX_RCV_BYTE_COUNT > rcvByteCount)
    {
        ++ErrReport[11];
        // build the command buffer if we're not idle
        szHartCmd[rcvByteCount] = nextByte;
        ++rcvByteCount;
    }
    // calc the LRC
    calculateLrc(nextByte);
    ++ErrReport[12];
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: hartTransmitterSm()
//
// Description:
//
// Hart Transmitter State machine controls the sending of the Hart response message
//
// Parameters: void
//
// Return Type: void.
//
// Implementation notes:
//
// Puts the next character in the TX queue into the UART TX buffer. If no characters
// remain in the queue, the RTS signal is lowered, and the TX interrupt is disabled.
//
///////////////////////////////////////////////////////////////////////////////////////////
void hartTransmitterSm(void)
{

    switch (ePresentXmitState)
    {
    case eXmitPreamble:
        // Send a preamble character
        putByteHart(HART_PREAMBLE); // write the character to Hart outstream
        if (0 == --respXmitIndex)
        {
            ePresentXmitState = eXmitAck;
        }
        // Enable the transmit interrupt
        enableTxIntr();
        return;
    case eXmitAck:
        respXmitIndex = (szHartResp[0] & LONG_ADDR_MASK) ? (szHartResp[LONG_COUNT_OFFSET] +
            LONG_COUNT_OFFSET) : (szHartResp[SHORT_COUNT_OFFSET] + SHORT_COUNT_OFFSET);
        ePresentXmitState = eXmitCtrlData;
        break;
    case eXmitCtrlData:
        if (0 == --respXmitIndex)
        {
            ePresentXmitState = eXmitLrc;
        }
        break;
    case eXmitLrc:
      putByteHart(szLrc);
      ePresentXmitState = eXmitDone;
        // Enable the transmit interrupt
        enableTxIntr();
        return;
    case eXmitDone:
    case eXmitIdle:
    default:
        // wait for the last character to leave the shift register
        while (HART_STAT & UCBUSY);
        // Drop RTS
        rtsRcv();
        // count the transmitted message
        xmtMsgCounter++;
        // Clear the appropriate cold start bit
        if (startUpDataLocalV.fromPrimary)
        {
            clrPrimaryStatusBits(FD_STATUS_COLD_START);
        }
        else
        {
            clrSecondaryStatusBits(FD_STATUS_COLD_START);
        }
        // If cmdReset is set and we are here, we should not respond
        // again until we emerge from the reset, so set the do not respond
        // flag
        if (TRUE == cmdReset)
        {
            doNotRespond = TRUE;
        }

        // Get ready for next frame
        prepareToRxFrame();
        // Set the xmit state to idle
        ePresentXmitState = eXmitIdle;
        return;
    }
    if (eXmitPreamble != ePresentXmitState)
    {
    // Calculate the LRC
    calculateLrc(*pRespBuffer);
    // Transmit the character
    putByteHart(*pRespBuffer );
    // Move to the next character
    pRespBuffer++;
    // re-enable the transmit interrupt
    enableTxIntr();
    }

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
// Function Name: prepareToRxFrame()
//
// Description:
//
// Set up the receive state machine, counters, and flags to receive a new frame
//
// Parameters: void
//
// Return Type: void.
//
// Implementation notes:
//
//
///////////////////////////////////////////////////////////////////////////////////////////
void prepareToRxFrame(void)
{
  // Sort by scope (Globals first)
  hartFrameRcvd = FALSE;
  szLrc = 0;                    // Set the LRC to 0
  hartCommand = 0xfe;               // Make the command invalid
  hostActive = FALSE;
  // Clear the HART error register, except for the assumed LRC error
  HartErrRegister = RCV_BAD_LRC;

  // Set all the counters to 0
  hartDataCount = 0;
  expectedByteCnt = 0;
  expectedAddrByteCnt = 0;
  addressStartIdx = 0;
  respXmitIndex = 0;
  // set the flags to false
  longAddressFlag = FALSE;
  rcvLrcError = TRUE; // Assume an error until the LRC is OK
  commandReadyToProcess = FALSE;
  addressValid = FALSE;
  overrunErr = FALSE;
  parityErr = FALSE;

  // locals
  rcvByteCount = 0;
  rcvAddrCount = 0;
  preambleByteCount = 0;
  totalRcvByteCount = 0;
  ePresentRcvState = eRcvSom;   // Set the state machine to look for the start of message
  // Set the transmit pointer back to the beginning of the buffer
   pRespBuffer = szHartResp;




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
