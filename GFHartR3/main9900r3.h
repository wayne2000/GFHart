#ifndef MAIN9900_H_
#define MAIN9900_H_

///////////////////////////////////////////////////////////////////////////////////////////
//
// Header File Name:  Main9900.h
//
// Description:
//
// This module provides HART to 9900 main interface. Functions are implemented in 
// the file, Main9900.c
//
// Revision History:
// Date    Rev.   Engineer     Description
// -------- ----- ------------ --------------
// 06/29/11  0    Vijay Soni    Creation
//
//
///////////////////////////////////////////////////////////////////////////////////////////

/*************************************************************************
  *   $INCLUDES
*************************************************************************/
#include "define.h"
/*************************************************************************
  *   $DEFINES
*************************************************************************/
// Loop Modes
#define LOOP_OPERATIONAL    0
#define LOOP_FIXED_CURRENT  1

// Sensor update rate from the 9900
#define SENSOR_UPDATE_TIME  150     /*  can be as fast as once every 150 mS */

/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/
/*************************************************************************
  *   $GLOBAL VARIABLES
*************************************************************************/
// Buffer for the commands from the 9900
extern unsigned char sz9900CmdBuffer [];
// Buffer for the response to the 9900
extern unsigned char sz9900RespBuffer [];
// Do we have a host actively communicating?
extern int hostActive;
///
/// flags to make sure that the loop value does not get reported if an update is in progress
///
extern unsigned char updateInProgress;
extern unsigned char updateRequestSent;
extern int8u loopMode;
extern unsigned char PVvariableStatus;      //!< Device Variable Status for PV



// Every so many update messages, we need to remind the
// 9900 if it is in a fixed current state
#define UPDATE_REMINDER_COUNT	20

/////////////////////////////////////////
// 
// 9900 Utilities
//
/////////////////////////////////////////


typedef struct st9900database
{
	int16u DATABASE_LENGTH;
	char SERIAL_NUMBER [10];
	char MODEL_STR [10];
	char SW_VER_NUM [6];
	fp32 LOOP_SET_LOW_LIMIT;
	fp32 LOOP_SET_HIGH_LIMIT;
	fp32 LOOP_SETPOINT_4MA;
	fp32 LOOP_SETPOINT_20MA;
	fp32 LOOP_ADJ_4MA;
	fp32 LOOP_ADJ_20MA;
	int8u LOOP_ERROR_VAL;
	int8u LOOP_MODE;
	int8u MEASUREMENT_TYPE;
	char GF9900_MS_PARAMETER_REVISION;
	int8u Hart_Dev_Var_Class;
	int8u UnitsPrimaryVar;
	int8u UnitsSecondaryVar;
	int8u Pad;  // Just to be even
	int16u checksum;
} DATABASE_9900;


// For quick updates, union the database with a byte array. Just declare this
// as the database so the utilities will work
typedef union u9900database
{
	int8u bytes[sizeof(DATABASE_9900)];
	DATABASE_9900 db;
} U_DATABASE_9900;

// exported database
extern U_DATABASE_9900 u9900Database;
extern const DATABASE_9900 factory9900db;
extern int8u updateDelay;

// 9900 utilities





// 9900 Command Message Indicies
#define CMD_ATTN_IDX 	0					// The attention character is always first
#define CMD_ADDR_IDX 	CMD_ATTN_IDX+1		// Followed by the address
#define CMD_CMD_IDX 	CMD_ADDR_IDX+1		// Followed by the command
#define CMD_1ST_SEP_IDX	CMD_CMD_IDX+1		// Followed by the first separator
#define CMD_FIRST_DATA	CMD_1ST_SEP_IDX+1	// Followed by the first data
// 9900 Response message indicies
#define RSP_ADDR_IDX	0					// The return address is first
#define RSP_1ST_SEP_IDX	RSP_ADDR_IDX+1		// Followed by a separator
#define RSP_REQ_IDX		RSP_1ST_SEP_IDX+1	// Followed by a request
#define RSP_2ND_SEP_IDX	RSP_REQ_IDX+1		// Followed by a separator
#define RSP_STATUS_IDX	RSP_2ND_SEP_IDX+1	// Followed by status

#define ACK_NACK_CR_IDX	RSP_REQ_IDX+1	// Index for ACK/NAK response



// HART --> 9900 Response Status
#define RESP_GOOD_NO_ACTIVE_HOST	'0'
#define RESP_ACTIVE_HOST			'1'
#define RESP_HOST_ERROR				'2'
#define RESP_NO_OR_BAD_DB			'3'
// HART --> 9900 Response Requests
#define RESP_REQ_NO_REQ					'0'
#define RESP_REQ_SAVE_AND_RESTART_LOOP	'1'
#define RESP_REQ_CHANGE_4MA_POINT		'2'
#define RESP_REQ_CHANGE_20MA_POINT		'3'
#define RESP_REQ_CHANGE_4MA_ADJ			'4'
#define RESP_REQ_CHANGE_20MA_ADJ		'5'
#define RESP_REQ_CHANGE_SET_FIXED		'6'
#define RESP_REQ_CHANGE_RESUME_NO_SAVE	'7'


// Database Load message status
#define DB_EXPECT_MORE_DATA	'0'
#define DB_LAST_MESSAGE		'1'

// Database Load message indices
#define DB_ADDR_START_IDX	CMD_FIRST_DATA			// The DB offset is the first data
#define DB_BYTE_COUNT_IDX	DB_ADDR_START_IDX+3		// 2 bytes + separator
#define DB_STATUS_IDX		DB_BYTE_COUNT_IDX+3		// 2 bytes + separator
#define DB_FIRST_DATA_IDX	DB_STATUS_IDX+2			// 1 byte + separator

// Message constants
#define MIN_9900_CMD_SIZE	6	// The poll message is at least 6 characters

// prototypes
void Process9900Command(void);
void Process9900Poll(void);
void Process9900Update(void);
void Process9900DatabaseLoad(void);
void Nack9900Msg(void);
void Ack9900Msg(void);
int16u Calc9900DbChecksum(void);

extern int mainMsgInProgress;
extern int mainMsgReadyToProcess;
extern int mainMsgCharactersReceived;

// Trim command flags
extern unsigned char setToMinValue;
extern unsigned char setToMaxValue;

// Main UART Prototypes
void rxMainIsr (void);
void txMainIsr (void);
void clearMainUartErrFlags(void);
void enableMainRcvIntr(void);
void disableMainRcvIntr(void);
void enableMainTxIntr(void);
void disableMainTxIntr(void);
void setMainOddParity(void);
void setMainEvenParity(void);
void setMainNoParity(void);
void stopMainMsgTimer(void);
void init9900Timers(void);
void resetForNew9900Message(void);
void startMainXmit (void);

// Utility to set fixed current mode
unsigned char setFixedCurrentMode(float);
// Trim the PV to 0
//unsigned char trimPvToZero(void);
// Trim loop current
unsigned char trimLoopCurrentZero(float);
unsigned char trimLoopCurrentGain(float);
void setUpperRangeVal(void);
void setLowerRangeVal(void);
void setBothRangeVals(float upper, float lower);
void convertFloatToAscii(float, unsigned char *);
void copy9900factoryDb(void);
void updatePVstatus(void);


/*************************************************************************
  *   $INLINE FUNCTIONS
*************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: CalculateDatabaseChecksum()
//
// Description:
//
//      Calculates the checksum of the 9900 database.
//
// Parameters: void
//
//
// Return Type: int16u - the calculated chaecksum.
//
// Implementation notes:
//
//
///////////////////////////////////////////////////////////////////////////////////////////
inline int16u CalculateDatabaseChecksum(void)
{
    int iIdx;
    int16u calcCkSum = 0;

    for (iIdx = 0; iIdx < sizeof(DATABASE_9900); ++iIdx)
    {
        calcCkSum += u9900Database.bytes[iIdx];
    }
    return calcCkSum;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: HexAsciiToByte()
//
// Description:
//
//      converts a two-byte to hexadecimal string to it's numerical equivalent. A variant of atoh().
//
// Parameters:
//          int8u * inString - the input two-byte string
//          int8u * outByte - pointer to the output byte
//
// Return Type: int - TRUE if the input was a valid hex number, FALSE otherwise.
//
// Implementation notes:
//
//
///////////////////////////////////////////////////////////////////////////////////////////
inline int HexAsciiToByte (int8u* inString, int8u* outByte)
{
    int Success = FALSE;

    // Deal with the high nibble first
    int8u temp = inString[0];
    // If the character is NOT a valid hex character, bail. Only uppercase alpha is valid
    if ((temp < '0') || (temp > 'F') || ((temp > '9') && (temp < 'A')))
    {
        return Success;
    }
    // it's valid, so convert it
    temp = (temp > 0x39) ? ((temp - 7) - 0x30) : (temp - 0x30);
    // shift to make room for the low nibble
    *outByte = (temp << 4);
    temp = inString[1];
    // If the character is NOT a valid hex character, bail. Only uppercase alpha is valid
    if ((temp < '0') || (temp > 'F') || ((temp > '9') && (temp < 'A')))
    {
        return Success;
    }
    // it's valid, so convert it
    temp = (temp > 0x39) ? ((temp - 7) - 0x30) : (temp - 0x30);
    *outByte += temp;
    //. We succeeded, so return success
    Success = TRUE;
    return Success;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: ByteToHexAscii()
//
// Description:
//
//      converts a byte to the ASCII hex string. A variant of itoa().
//
// Parameters:
//          int8u inByte - the input byte to convert
//          int8u * outString - the output two-byte string
//
// Return Type: void.
//
// Implementation notes:
//
//      outString MUST point to a two-byte buffer
//
///////////////////////////////////////////////////////////////////////////////////////////
inline void ByteToHexAscii (int8u inByte, int8u * outString)
{
    // Convert the high nibble
    int8u temp = (inByte >> 4) | 0x30;
    temp = (temp > 0x39) ? (temp + 7) : temp;
    *outString = temp;
    // convert the low nibble
    temp = (inByte & 0x0F) | 0x30;
    temp = (temp > 0x39) ? (temp + 7) : temp;
    *(outString+1) = temp;
}

#endif /*MAIN9900_H_*/
