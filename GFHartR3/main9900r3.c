/*!
 *  \file   main9900r3.c
 *  \brief  Command for handling the High Speed serial Bus to 9900
 *  Recoded on: Nov 6, 2012
 *  \author: MH
 *
 *  Revision History:
 *  Date    Rev.   Engineer     Description
 *  -------- ----- ------------ --------------
 *  04/01/11  0    Vijay Soni    Creation
 *
 */

//=======================
// INCLUDES
//=======================
#include <msp430f5528.h>
#include <string.h>
#include "hardware.h"
#include "hartr3.h"
#include "main9900r3.h"

//========================
//  LOCAL DEFINES
//========================
//================================
//  LOCAL PROTOTYPES.
//================================
//========================
//  GLOBAL DATA
//========================
//========================
//  LOCAL DATA
//========================

//==============================================================================
// FUNCTIONS
//==============================================================================

// Diagnostics
unsigned long numMessagesRcvd = 0;
unsigned long numMessagesErrored = 0;

// 9900 Database
U_DATABASE_9900 u9900Database;
// flags for determining status
// database loaded OK flag
int databaseOk = FALSE;
// Do we have a host actively communicating?
int hostActive = FALSE;
// Is the host in an comm error condition?
int hostError = FALSE;
// The most recent comm status from the 9900
int8u lastCommStatus = POLL_LAST_REQ_GOOD;
// the most recent variable status from the 9900
int8u lastVarStatus = UPDATE_STATUS_INIT;
int8u varStatus = UPDATE_STATUS_GOOD;
// Buffer for the commands from the 9900
unsigned char sz9900CmdBuffer [MAX_9900_CMD_SIZE];
// Buffer for the response to the 9900
unsigned char sz9900RespBuffer [MAX_9900_RESP_SIZE];
// the size of the response
unsigned int responseSize = 0;
// Transmitted character counter
unsigned int numMainXmitChars = 0;
int mainMsgInProgress = FALSE;
int mainMsgReadyToProcess = FALSE;
int mainMsgCharactersReceived = 0;
// A flag that indicates that an update
// message has been received from the 9900
int updateMsgRcvd = FALSE;
// If the update is delayed, set this flag
int8u updateDelay = FALSE;
// A request is made by the HART master
int8u request = RESP_REQ_SAVE_AND_RESTART_LOOP;
// There is usually a value for a request
float requestValue = 0.0;
float rangeRequestUpper = 0.0;
// Loop mode is either operational, fixed 4, or fixed 20
int8u loopMode = LOOP_OPERATIONAL;
// We have to queue requests for responses 1 and 3 
int8u queueResp1 = FALSE;
int8u queueResp3 = FALSE;
// If we go to constant current mode, we need to see whether a save
// is required when the loop goes operational again
int8u saveRequested = FALSE;
// flags to make sure that the loop value does not
// get reported if an update is in progress
unsigned char updateInProgress = FALSE;
unsigned char updateRequestSent = FALSE;
// Trim command flags
unsigned char setToMinValue = FALSE;
unsigned char setToMaxValue = FALSE;
// Transmit flag. Set when transmit is active. Used to insure that the 
// transmit & rcv ISRs don't run simultaneously
unsigned char transmitMode = FALSE;
// Add a counter to remind the 9900 of the current mode every 256 update messages
unsigned char modeUpdateCount = 0;
// Device Variable Status for PV. initialize to BAD, constant
unsigned char PVvariableStatus = VAR_STATUS_BAD | LIM_STATUS_CONST;
// The timer for signaling HART is update messages don't occur
unsigned long UpdateMsgTimeout = 0;

// local prototype
void killMainTransmit(void);

#ifdef QUICK_START
// Flag and counter to make sure the 9900 has communicated
unsigned char comm9900started = FALSE;
unsigned char checkCurrentMode = FALSE;
unsigned int comm9900counter = 0;
unsigned char currentMsgSent = NO_CURRENT_MESSAGE_SENT;
#endif



// 9900 Factory database
const DATABASE_9900 factory9900db =
{
	62,		// DB Length
	"4640500111",	// serial number
	"3-9900-1X ",	// Model string
	"10-04a",	// SW REV
	0.0,		// LOOP_SET_LOW_LIMIT
	15.0,		// LOOP_SET_HIGH_LIMIT
	0.0,		// LOOP_SETPOINT_4MA
	14.0,		// LOOP_SETPOINT_20MA
	4.0,		// LOOP_ADJ_4MA
	20.0,		// LOOP_ADJ_20MA
	1,	// LOOP_ERROR_VAL
	0,	// LOOP_MODE;
	2,	// MEASUREMENT_TYPE;
	'A',	// GF9900_MS_PARAMETER_REVISION;
	'Q',	// Hart_Dev_Var_Class;
	0x3b,	// UnitsPrimaryVar;
	0x20,	// UnitsSecondaryVar;
	0,	// Pad;  // Just to be even
	0x0972	// checksum;
};



///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: Process9900Command()
//
// Description:
//
//     Processes the command from the 9900 UART, sends response 
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
void Process9900Command(void)
{
	// First, make sure the command is for the HART modem.
	// First character must be an ATTENTION character, followed 
	// by the HART_COMMAND character. If not, then return.
	if ((ATTENTION != sz9900CmdBuffer[CMD_ATTN_IDX]) || 
		(HART_ADDRESS != sz9900CmdBuffer[CMD_ADDR_IDX]))
	{
		// either it isn't a command or it's not for the HART module
		return;
	}
	// The command type is the 3rd character in the command
	switch (sz9900CmdBuffer[CMD_CMD_IDX])
	{
	case HART_POLL:
		Process9900Poll();
		break;
	case HART_UPDATE:
#ifdef QUICK_START
		// if this is the first update message, check the current mode, and set .	
		if ((FALSE == comm9900started) || (UPDATE_REMINDER_COUNT <= modeUpdateCount)) 
		{
			// reset the update reminder count
			modeUpdateCount = 0;
			if (databaseOk)
			{
				// Now check the flash to make sure the current mode is
				// set correctly
				if (CURRENT_MODE_DISABLE == startUpDataLocalNv.currentMode)
				{
					// Tell the 9900 to go to fixed current mode at 4 mA
					setFixedCurrentMode(4.0);
					setPrimaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
					setSecondaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
					currentMsgSent = FIXED_CURRENT_MESSAGE_SENT;
				}
				else // NOT in multidrop mode
				{
					// If the loop reporting is not operational, we want to make sure
					// the 9900 is reminded about the fixed current output
					if (LOOP_OPERATIONAL == loopMode)
					{
						// Tell the 9900 to go to loop reporting current mode
						setFixedCurrentMode(0.0);
						clrPrimaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
						clrSecondaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
						currentMsgSent = LOOP_CURRENT_MESSAGE_SENT;
					}
					else
					{
						// Tell the 9900 to go to fixed current mode at 
						// the last requested command value
						setFixedCurrentMode(lastRequestedCurrentValue);
						setPrimaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
						setSecondaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
					}
				}
				// Set the 9900 comm flag true so that the timeout will not occur
				comm9900started = TRUE; 
			}
		}
#endif	
		Process9900Update();
		modeUpdateCount++;
		break;
	case HART_DB_LOAD:
		Process9900DatabaseLoad();
		break;
	default:
		// We have no idea what the message is, but it has a <CR>, so NACK it
		Nack9900Msg();
		break;
	}	
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: Process9900Poll()
//
// Description:
//
//     Processes the poll command from the 9900 UART, sends response 
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
void Process9900Poll(void)
{
	// Set the response size to 0 to start
	responseSize = 0;
	// capture the last comm status
	lastCommStatus = sz9900CmdBuffer[CMD_FIRST_DATA];
	// determine the status to reply with
	int8u status;
	if (databaseOk)
	{
		if (hostActive)
		{
			if (hostError)
			{
				status = RESP_HOST_ERROR;
			}
			else
			{
				status = RESP_ACTIVE_HOST;
			}
		}
		else
		{
			status = RESP_GOOD_NO_ACTIVE_HOST;
		}
	}
	else
	{
		status = RESP_NO_OR_BAD_DB;
	}
	// Build the poll response
	sz9900RespBuffer[RSP_ADDR_IDX] = HART_ADDRESS;
	++responseSize;
	sz9900RespBuffer[RSP_ADDR_IDX+1] = HART_SEPARATOR;
	++responseSize;
	sz9900RespBuffer[RSP_REQ_IDX] = request;
	++responseSize;
	sz9900RespBuffer[RSP_REQ_IDX+1] = HART_SEPARATOR;
	++responseSize;
	sz9900RespBuffer[RSP_STATUS_IDX] = status;
	++responseSize;
	if (RESP_REQ_NO_REQ == request)
	{
		sz9900RespBuffer[RSP_STATUS_IDX+1] = HART_MSG_END;
		++responseSize;
	}
	else
	{
		// Put in the separator
		sz9900RespBuffer[responseSize] = HART_SEPARATOR;
		++responseSize;
		// Send the value back
		convertFloatToAscii(requestValue, &(sz9900RespBuffer[responseSize]));
		responseSize += 8;
		// Carriage return
		sz9900RespBuffer[responseSize] = HART_MSG_END;
		++responseSize;
		// Now, reset the request to none, 1, or 3
		if (queueResp1)
		{
			queueResp1 = FALSE;
			request = RESP_REQ_SAVE_AND_RESTART_LOOP;
			requestValue = 0.0;
		}
		else if (queueResp3)
		{
			queueResp3 = FALSE;
			queueResp1 = TRUE;
			request = RESP_REQ_CHANGE_20MA_POINT;
			requestValue = rangeRequestUpper;
		}
		else
		{
			request = RESP_REQ_NO_REQ;
		}
	}
	// Signal that a loop change request has been sent
	if (TRUE == updateInProgress)
	{
		updateRequestSent = TRUE;
	}
	// Load the transmit buffer & send
	startMainXmit();	
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: Process9900Update()
//
// Description:
//
//     Processes the update command from the 9900 UART, sends response 
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
void Process9900Update(void)
{
	fp32 tempVal;
	int index;
	int success;
	int8u tempChar;
	responseSize = 0;
	// Work through the message, upadating everything
	// grab the PV as bytes
	for (index = 0; index < 4; ++index)
	{
		success = HexAsciiToByte(sz9900CmdBuffer+UPDATE_PV_START_INDEX + (2*index), &tempChar);
		if (success)
		{
			tempVal.byteVal[index] = tempChar;
		}
		else
		{
			// respond with a NACK
			Nack9900Msg();
			// no sense going further, return
			return;
		}
	}
	// If we're here, it means we picked up the PV, so write the new value
	PVvalue = tempVal.floatVal;
	// the next 4 bytes are the secondary value, which may or may not be real.
	for (index = 0; index < 4; ++index)
	{
		success = HexAsciiToByte(sz9900CmdBuffer+UPDATE_SV_START_INDEX + (2*index), &tempChar);
		if (success)
		{
			tempVal.byteVal[index] = tempChar;
		}
		else
		{
			// respond with a NACK
			Nack9900Msg();
			// no sense going further, return
			return;
		}
	}
	// If we're here, it means we picked up the SV, so write the new value
	SVvalue = tempVal.floatVal;
	// the next 4 bytes are the loop current value.
	for (index = 0; index < 4; ++index)
	{
		success = HexAsciiToByte(sz9900CmdBuffer+UPDATE_MA4_20_START_INDEX + (2*index), &tempChar);
		if (success)
		{
			tempVal.byteVal[index] = tempChar;
		}
		else
		{
			// respond with a NACK
			Nack9900Msg();
			// no sense going further, return
			return;
		}
	}
	// If we're here, it means we picked up the loop current, so write the new value
	ma4_20 = tempVal.floatVal;
	// Now the var status
	varStatus = sz9900CmdBuffer[UPDATE_VAR_STATUS_INDEX];
	// Per +GF+, set the "PV out of range" bit when the status is anything other
	// than good from the 9900
	if (UPDATE_STATUS_GOOD == varStatus)
	{
		clrPrimaryStatusBits(FD_STATUS_PV_OUT_OF_LIMITS);
		clrSecondaryStatusBits(FD_STATUS_PV_OUT_OF_LIMITS);
		startUpDataLocalV.StandardStatus0 &= ~SS0_HARDWARE_PROBLEM;
		if (varStatus != lastVarStatus)
		{ 
			clrPrimaryMoreAvailable();
			clrSecondaryMoreAvailable();
		}
	}
	else
	{
		setPrimaryStatusBits(FD_STATUS_PV_OUT_OF_LIMITS);
		setSecondaryStatusBits(FD_STATUS_PV_OUT_OF_LIMITS);
		startUpDataLocalV.StandardStatus0 |= SS0_HARDWARE_PROBLEM;
		if (varStatus != lastVarStatus)
		{ 
			setPrimaryMoreAvailable();
			setSecondaryMoreAvailable();
		}
	}
	// Save the var status for the next update
	lastVarStatus = varStatus;
	// Now save the comm status
	lastCommStatus = sz9900CmdBuffer[UPDATE_COMM_STATUS_INDEX];
	// If we're here, we can build a normal response
	// determine the status to reply with
	// If both update flags are set, clear them
	if ((TRUE == updateInProgress) && (TRUE == updateRequestSent) && 
		(POLL_LAST_REQ_GOOD == lastCommStatus))
	{
		updateRequestSent = FALSE;
		updateInProgress = FALSE;
	}
	int8u status;
	if (databaseOk)
	{
		if (hostActive)
		{
			if (hostError)
			{
				status = RESP_HOST_ERROR;
			}
			else
			{
				status = RESP_ACTIVE_HOST;
			}
		}
		else
		{
			status = RESP_GOOD_NO_ACTIVE_HOST;
		}
	}
	else
	{
		status = RESP_NO_OR_BAD_DB;
	}
	// Build the update response
	sz9900RespBuffer[RSP_ADDR_IDX] = HART_ADDRESS;
	++responseSize;
	sz9900RespBuffer[RSP_ADDR_IDX+1] = HART_SEPARATOR;
	++responseSize;
	sz9900RespBuffer[RSP_REQ_IDX] = request;
	++responseSize;
	sz9900RespBuffer[RSP_REQ_IDX+1] = HART_SEPARATOR;
	++responseSize;
	sz9900RespBuffer[RSP_STATUS_IDX] = status;
	++responseSize;
	if (RESP_REQ_NO_REQ == request)
	{
		sz9900RespBuffer[RSP_STATUS_IDX+1] = HART_MSG_END;
		++responseSize;
	}
	else
	{
		// Put in the request
		sz9900RespBuffer[responseSize] = HART_SEPARATOR;
		++responseSize;
		// Send the value back
		convertFloatToAscii(requestValue, &(sz9900RespBuffer[responseSize]));
		responseSize += 8;
		// Carriage return
		sz9900RespBuffer[responseSize] = HART_MSG_END;
		++responseSize;
		// Now, reset the request to none, 1, or 3
		if (queueResp1)
		{
			queueResp1 = FALSE;
			request = RESP_REQ_SAVE_AND_RESTART_LOOP;
			requestValue = 0.0;
		}
		else if (queueResp3)
		{
			queueResp3 = FALSE;
			queueResp1 = TRUE;
			request = RESP_REQ_CHANGE_20MA_POINT;
			requestValue = rangeRequestUpper;
		}
		else
		{
			request = RESP_REQ_NO_REQ;
		}
	}
	// Signal that a loop change request has been sent
	if (TRUE == updateInProgress)
	{
		updateRequestSent = TRUE;
	}
	// Load the transmit buffer & send
	startMainXmit();	
	// Set the flag TRUE so HART communications can begin
	updateMsgRcvd = TRUE;
	// Calculate the status for HART
	updatePVstatus();
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: Process9900DatabaseLoad()
//
// Description:
//
//     Processes the database load command from the 9900 UART, sends response 
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
void Process9900DatabaseLoad(void)
{
	int success;
	int8u index;
	int8u numDbBytesSent;
	int8u startAddress;
	int8u currentData;
	int8u msgStatus;
	
	// Process the request to load the database
	// Grab the starting offset into the DB
	success = HexAsciiToByte((sz9900CmdBuffer+DB_ADDR_START_IDX), &startAddress);
	if (!success)
	{
		// Set the flag to indicate there's a DB problem
		databaseOk = FALSE;
		// respond with a NACK
		Nack9900Msg();
		// no sense going further, return
		return;
	}
	// grab the number of bytes being transmitted
	success = HexAsciiToByte((sz9900CmdBuffer+DB_BYTE_COUNT_IDX), &numDbBytesSent);
	if (!success)
	{
		// Set the flag to indicate there's a DB problem
		databaseOk = FALSE;
		// respond with a NACK
		Nack9900Msg();
		// no sense going further, return
		return;
	}
	// Now pull off the status - is this the last message?
	msgStatus = sz9900CmdBuffer[DB_STATUS_IDX];
	// Check to make sure the status is valid, bail if it isn't
	if (!((DB_EXPECT_MORE_DATA == msgStatus) || (DB_LAST_MESSAGE == msgStatus)))
	{
		// Set the flag to indicate there's a DB problem
		databaseOk = FALSE;
		// respond with a NACK
		Nack9900Msg();
		// no sense going further, return
		return;
	}
	// Now convert the data, one character pair at a time
	for (index = 0; index < numDbBytesSent; ++index)
	{
		success = HexAsciiToByte((sz9900CmdBuffer+DB_FIRST_DATA_IDX+(2*index)), &currentData);
		if (!success)
		{
			// Set the flag to indicate there's a DB problem
			databaseOk = FALSE;
			// respond with a NACK
			Nack9900Msg();
			// no sense going further, return
			return;
		}
		// write to the DB
		u9900Database.bytes[startAddress+index] = currentData;
	}
	if (DB_LAST_MESSAGE == msgStatus)
	{
		// Compare the calculated checksum to the downloaded checksum
		if (Calc9900DbChecksum() == u9900Database.db.checksum)
		{
			// Set the flag to indicate we're good
			databaseOk = TRUE;
			// Respond with and ACK
			Ack9900Msg();
			// Now update the default sensor type
			UpdateSensorType();
		}
		else
		{
			// Set the flag to indicate there's a DB problem
			databaseOk = FALSE;
			// respond with a NACK
			Nack9900Msg();
		}
	}
	else
	{
		// Just respond with an ACK
		Ack9900Msg();
	}
}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: Nack9900Msg()
//
// Description:
//
//     Sends 9900 NACK response 
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
void Nack9900Msg(void)
{
	responseSize = 0;
	// Build the NACK response
	sz9900RespBuffer[RSP_ADDR_IDX] = HART_ADDRESS;
	++responseSize;
	sz9900RespBuffer[RSP_ADDR_IDX+1] = HART_SEPARATOR;
	++responseSize;
	sz9900RespBuffer[RSP_REQ_IDX] = HART_NACK;
	++responseSize;
	sz9900RespBuffer[ACK_NACK_CR_IDX] = HART_MSG_END;
	++responseSize;
	// Load the transmit buffer & send
	startMainXmit();	
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: Ack9900Msg()
//
// Description:
//
//     Sends a 9900 ACK response 
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
void Ack9900Msg(void)
{
	responseSize = 0;
	// Build the ACK response
	sz9900RespBuffer[RSP_ADDR_IDX] = HART_ADDRESS;
	++responseSize;
	sz9900RespBuffer[RSP_ADDR_IDX+1] = HART_SEPARATOR;
	++responseSize;
	sz9900RespBuffer[RSP_REQ_IDX] = HART_ACK;
	++responseSize;
	sz9900RespBuffer[ACK_NACK_CR_IDX] = HART_MSG_END;
	++responseSize;
	// Load the transmit buffer & send
	startMainXmit();	
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: Calc9900DbChecksum()
//
// Description:
//
//     Calculates the checksum on the 9900 database 
//
// Parameters: void
//
// Return Type: int16u - the calculated checksum.
//
// Implementation notes:
//
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
int16u Calc9900DbChecksum(void)
{
	int index;
	int16u calcChksum = 0;
	// Now just add up all the bytes, except the checksum at the end
	for (index = 0; index < sizeof(DATABASE_9900)-2; ++index)
	{ 
		calcChksum += (int16u)(u9900Database.bytes[index]);
	}
	return calcChksum;
}

/******************************************************
 * 
 * A0 (Main) UART Functions
 * 
 ******************************************************/

///////////////////////////////////////////////////////////////////////////////////////////
//
// \function hsbReceiver  supersedes rxMainIsr()
//
// Description:
//
// The actual work done in the receive ISR. Checks errors, puts the received character 
// into the main command buffer
//
// Parameters: void
//
// Return Type: void.
//
// Implementation notes:
//
// Checks the receive error flags, as well as checks to make sure the character 
// is addressed to the HART board
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void hsbReceiver (BYTE tempChar)
{
	unsigned char statusReg;
	
#if 0	
	// Check the transmit flag to make sure the unit hasn't lost gotten lost. Kill the 
	// transmit process if it is
	if (TRUE == transmitMode)
	{
		killMainTransmit();
	}
#endif	
	// Record the message arrival
	++numMessagesRcvd;
	//  Error handling at the call, note that we take the error in the UART and
	//  not in the received (i.e. we don't care in reporting error)

	// Only store the character if there is a main message in progress for US
	if (mainMsgInProgress)
	{
		// save the character
		sz9900CmdBuffer[mainMsgCharactersReceived] = tempChar;		
		// increment the count
		++mainMsgCharactersReceived;
	}
	// Look for the start of a message, the end of a message, or the HART address
	// If this is the start of a message, set up for receipt
	if (ATTENTION == tempChar)
	{
		// start the receive timer
		// TODO: need another timer ====> startMainMsgTimer();
		// clear the ready to process flag
		mainMsgReadyToProcess = FALSE;
		// store the character in the first location
		sz9900CmdBuffer[0] = tempChar;		
		// set the receive count to 1
		mainMsgCharactersReceived = 1;
		// Set the flag to store what's coming in
		mainMsgInProgress = TRUE;
	}
	// We've received 2 characters and the new one is not an 'H', the message
	// isn't for the HART board, so clear the flag indicating a message is in
	// progress, and wait for the next message to begin
	if ((2 == mainMsgCharactersReceived) && (HART_ADDRESS != tempChar)) 
	{
		resetForNew9900Message();
	}
	// finally, look for the message end <CR>. There had to be at least 5 characters received to 
	// be a valid message
	if ((MIN_9900_CMD_SIZE <= mainMsgCharactersReceived) && 
		(HART_MSG_END == tempChar) && mainMsgInProgress)
	{
		// stop the main message timer
		stopMainMsgTimer();
		// Shut off the receive flag
		mainMsgInProgress = FALSE;
		// process the message
		mainMsgReadyToProcess = TRUE;
		// Shut off the receive interrupt until the received message is handled
		disableMainRcvIntr();
	}
}

/*!
 *  \function sendHsbReply()   == supersedes =>txMainIsr()
 *  Description:
 *  This function transmits through the hsb uart output stream the hsb reply message stored in sz9900RespBuffer[]
 *
 *  Parameters: The original function uses globals to communicate TBD
 *  Return Type: void.
 *  */
void sendHsbReply(void)
{
#if 0
	// See if we are done first
	if (responseSize <= numMainXmitChars)
	{
	  // Disable the transmit interrupt
		disableMainTxIntr();
		// wait for the UART to not be busy
		putcUart(&hsbUart); //  TODO replaced  while(MAIN_STAT & UCBUSY);
		// turn the transmit mode flag off
		transmitMode = FALSE;
	}
	else
	{
		// Load the UART with the next character
		MAIN_TXBUF = sz9900RespBuffer[numMainXmitChars];
		// increment the index
		numMainXmitChars++;
		// reenable the TX interrupt
		enableMainTxIntr();
	}
#endif

}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: clearMainUartErrFlags()
//
// Description:
//
// Clears any error flags that were set in reception. 
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
void clearMainUartErrFlags(void)
{
  // I need to write an equivalent function
#if 0

	unsigned char temp;
	// Read the RX buffer to make sure UCOE & UCRXERR are cleared
	temp = MAIN_RXBUF;
	// Force the other errors to be cleared.
	MAIN_STAT &= ~(UCFE | UCPE | UCBRK | UCRXERR | UCOE);
	// verify that the flags ARE clear
	if (MAIN_STAT & (UCFE | UCPE | UCBRK | UCRXERR | UCOE))
	{
		temp = MAIN_STAT;
	}
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: stopMainMsgTimer()
//
// Description:
//
// Stops the main UART timer
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
void stopMainMsgTimer(void)
{
	//mainMsgTimer.onFlag = FALSE;
}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: incrementMainMsgTimer()
//
// Description:
//
// Increments the Main UART timer
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
void incrementMainMsgTimer(void)
{
	// Always increment the update message timeout
	++UpdateMsgTimeout;
	//TODO  Need to write another timer
#if 0
	if (MAXIMUM_UPDATE_INTERVAL < UpdateMsgTimeout)
	{
		PVvariableStatus = VAR_STATUS_BAD;
	}
	if (TRUE == mainMsgTimer.onFlag)
	{
		// increment the count
		++mainMsgTimer.count;
		// check to see if the timer has expired
		if (TIMEOUT_9900 < mainMsgTimer.count)
		{
			// Instead of a flag, simply set up for the next 9900 message
			resetForNew9900Message();
		}
	}
#endif

}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: init9900Timers()
//
// Description:
//
// Sets up the Main 9900 UART timer
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
void init9900Timers(void)
{
	// initialize the timer to a known state
	//TODO: implement a hardware timer on hardware.c initHartTimer(&mainMsgTimer);
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: startMainXmit()
//
// Description:
//
// Adds characters to the main transmit queue. Returns TRUE if there's room to accept them all,
// FALSE otherwise. It also signals the UART to start transmitting
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void startMainXmit (void)
{
	// Make sure we actually have something to transmit
	if (0 == responseSize)
	{ 
		return;
	}
	// turn on the transmit mode
	transmitMode = TRUE;
	// Set the transmit counter to the first character to transmit
	numMainXmitChars = 0;
	// Enable the TX interrupt
	enableMainTxIntr();
	// Transmit the first character
	sendHsbReply();
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: resetForNew9900Message()
//
// Description:
//
// Sets up to start receiving a new message after a timeout
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
void resetForNew9900Message(void)
{
	// stop the timer
	stopMainMsgTimer();
	// Make sure the flags are clear
	mainMsgInProgress = FALSE;
	mainMsgReadyToProcess = FALSE;
	mainMsgCharactersReceived = 0;
	// make sure the characters can be received
	enableMainRcvIntr();  
	responseSize = 0;
}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: setFixedCurrentMode()
//
// Description:
//
// 		Utility to set fixed current mode
//
// Parameters: void
//
// Return Type: unsigned char:
//						0 = RESP_SUCCESS,
//						3 = PASSED_PARM_TOO_LARGE
//						4 = PASSED_PARM_TOO_SMALL
//
// Implementation notes:
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
unsigned char setFixedCurrentMode(float cmdValue)
{
	unsigned char resp = RESP_SUCCESS;
	// if we are here, we are in the correct loop mode, so see if we can adjust
	if (0.00 == cmdValue)
	{
		// Shut off the 9900 interrupts
		disableMainRcvIntr();
		disableMainTxIntr();
		// the request is 7 to leave the fixed current state
		loopMode = LOOP_OPERATIONAL;
		request = (FALSE == saveRequested) ? RESP_REQ_CHANGE_RESUME_NO_SAVE : RESP_REQ_SAVE_AND_RESTART_LOOP;
		saveRequested = FALSE;  
		requestValue = cmdValue;
		// Clear the trim flags
		setToMinValue = FALSE;
		setToMaxValue = FALSE;
		// re-enable the interrupts
		if (TRUE == transmitMode)
		{
			enableMainTxIntr();
		}
		enableMainRcvIntr();
	}
	else
	{
		// limit check first
		if (cmdValue < ADJ_4MA_LIMIT_MIN)
		{ 
			resp = PASSED_PARM_TOO_SMALL;
		}
		if (cmdValue > ADJ_20MA_LIMIT_MAX)
		{ 
			resp = PASSED_PARM_TOO_LARGE;
		}
		// store the value and mode
		if (!resp)
		{
			// Shut off the 9900 interrupts
			disableMainRcvIntr();
			disableMainTxIntr();
			loopMode = LOOP_FIXED_CURRENT;
			requestValue = cmdValue;
			request = RESP_REQ_CHANGE_SET_FIXED;
			setToMinValue = (4.0 == cmdValue) ? TRUE : FALSE;
			setToMaxValue = (20.0 == cmdValue) ? TRUE : FALSE;			
			// re-enable the interrupts
			if (TRUE == transmitMode)
			{
				enableMainTxIntr();
			}
			enableMainRcvIntr();
		}
	}
	return resp;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: trimLoopCurrentZero()
//
// Description:
//		Trims the loop current to minimum 
//
// Parameters: float - measured loop current level
//
// Return Type: unsigned char:
//						0 = RESP_SUCCESS,
//						3 = PASSED_PARM_TOO_LARGE
//						4 = PASSED_PARM_TOO_SMALL
//						9 = INCORRECT_LOOP_MODE,
//
// Implementation notes:
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
unsigned char trimLoopCurrentZero(float level)
{
	unsigned char resp = (LOOP_OPERATIONAL == loopMode) ? INCORRECT_LOOP_MODE : RESP_SUCCESS;
	// If the mode is wrong, just bail now
	if (resp)
	{
		return resp;
	}
	// Now verify the request is in limits
	if (level > ADJ_4MA_LIMIT_MAX)
	{
		resp = PASSED_PARM_TOO_LARGE;
	}
	else if (level < ADJ_4MA_LIMIT_MIN)
	{
		resp = PASSED_PARM_TOO_SMALL;
	}
	if (!resp)
	{
		// Shut off the 9900 interrupts
		disableMainRcvIntr();
		disableMainTxIntr();
		// We are OK so make the request to the 9900
		requestValue = level;
		request = RESP_REQ_CHANGE_4MA_ADJ;
		// Queue the request to save & restart the loop
		saveRequested = TRUE;
		// re-enable the interrupts
		if (TRUE == transmitMode)
		{
			enableMainTxIntr();
		}
		enableMainRcvIntr();
	}
	return resp;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: trimLoopCurrentGain()
//
// Description:
//		Trims the loop current gain 
//
// Parameters: float - measured loop current level
//
// Return Type: unsigned char:
//						0 = RESP_SUCCESS,
//						3 = PASSED_PARM_TOO_LARGE
//						4 = PASSED_PARM_TOO_SMALL
//						9 = INCORRECT_LOOP_MODE,
//
// Implementation notes:
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
unsigned char trimLoopCurrentGain(float level)
{
	unsigned char resp = (LOOP_OPERATIONAL == loopMode) ? INCORRECT_LOOP_MODE : RESP_SUCCESS;
	// If the mode is wrong, just bail now
	if (resp)
	{
		return resp;
	}
	// Now verify the request is in limits
	if (level > ADJ_20MA_LIMIT_MAX)
	{
		resp = PASSED_PARM_TOO_LARGE;
	}
	else if (level < ADJ_20MA_LIMIT_MIN)
	{
		resp = PASSED_PARM_TOO_SMALL;
	}
	if (!resp)
	{
		// Shut off the 9900 interrupts
		disableMainRcvIntr();
		disableMainTxIntr();
		// We are OK so make the request to the 9900
		requestValue = level;
		request = RESP_REQ_CHANGE_20MA_ADJ;
		// Queue the request to save & restart the loop
		saveRequested = TRUE;
		// re-enable the interrupts
		if (TRUE == transmitMode)
		{
			enableMainTxIntr();
		}
		enableMainRcvIntr();
	}
	return resp;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: setUpperRangeVal()
//
// Description:
//		Sets the upper range value (span) to the present PV 
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void setUpperRangeVal(void)
{
	// Shut off the 9900 interrupts
	disableMainRcvIntr();
	disableMainTxIntr();
	// Update the local DB with the requested values so 
	// the modem can respond quickly
	u9900Database.db.LOOP_SET_HIGH_LIMIT.floatVal = PVvalue;
	// The present PV is the request value
	requestValue = PVvalue;
	request = RESP_REQ_CHANGE_20MA_POINT;
	// Queue the request for response 1
	queueResp1 = TRUE;
	// re-enable the interrupts
	if (TRUE == transmitMode)
	{
		enableMainTxIntr();
	}
	enableMainRcvIntr();
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: setLowerRangeVal()
//
// Description:
//		Sets the lower range value to the present PV 
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void setLowerRangeVal(void)
{
	// Shut off the 9900 interrupts
	disableMainRcvIntr();
	disableMainTxIntr();
	// Update the local DB with the requested value so 
	// the modem can respond quickly
	u9900Database.db.LOOP_SET_LOW_LIMIT.floatVal = PVvalue;
	// The present PV is the request value
	requestValue = PVvalue;
	request = RESP_REQ_CHANGE_4MA_POINT;
	// Queue the request for response 1
	queueResp1 = TRUE;
	// re-enable the interrupts
	if (TRUE == transmitMode)
	{
		enableMainTxIntr();
	}
	enableMainRcvIntr();
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: setBothRangeVals()
//
// Description:
//		Sets the lower range value and queues the request for the upper 
//
// Parameters: float - upper range value
//			   float - lower range value 
//
// Return Type: void
//
// Implementation notes:
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void setBothRangeVals(float upper, float lower)
{
	// Shut off the 9900 interrupts
	disableMainRcvIntr();
	disableMainTxIntr();
	// Update the local DB with the requested values so 
	// the modem can respond quickly
	u9900Database.db.LOOP_SET_HIGH_LIMIT.floatVal = upper;
	u9900Database.db.LOOP_SET_LOW_LIMIT.floatVal = lower;
	// The present PV is the request value
	requestValue = lower;
	request = RESP_REQ_CHANGE_4MA_POINT;
	// Queue the request for response 3
	rangeRequestUpper = upper;
	queueResp3 = TRUE;
	// re-enable the interrupts
	if (TRUE == transmitMode)
	{
		enableMainTxIntr();
	}
	enableMainRcvIntr();
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: convertFloatToAscii()
//
// Description:
//		converts a float into an eight-byte ASCII string (little-endian)
//
// Parameters: float - the value to convert
//			   unsigned char * - the 8-byte result buffer
//
// Return Type: void
//
// Implementation notes:
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void convertFloatToAscii(float value, unsigned char * respBuffer)
{
	fp32 val;
	
	val.floatVal = value;
	int count;
	
	for (count = 0; count < 4; ++count)
	{
		ByteToHexAscii(val.byteVal[count], respBuffer+(2*count));
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: copy9900factoryDb()
//
// Description:
//
// copy the factory database to the local
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
void copy9900factoryDb(void)
{
	memcpy(&u9900Database, &factory9900db, sizeof(DATABASE_9900));
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: killMainTransmit()
//
// Description:
//
// kills the transmit process
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
void killMainTransmit(void)
{
	// disable the Tx interrupt
	disableMainTxIntr();
	// turn the transmit mode flag off
	transmitMode = FALSE;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: updatePVstatus()
//
// Description:
//
// updates the device variable status of PV
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
void updatePVstatus(void)
{
	// Decide the PV status based upon the value of the variable status
	switch(varStatus)
	{
	case UPDATE_STATUS_GOOD:
		PVvariableStatus = VAR_STATUS_GOOD;
		break;
	case UPDATE_STATUS_CHECK_SENSOR_BAD_VALUE:
		// Check to see where the limits are
		PVvariableStatus = VAR_STATUS_BAD;
		if (PVvalue >= u9900Database.db.LOOP_SET_HIGH_LIMIT.floatVal)
		{
			PVvariableStatus |= LIM_STATUS_HIGH;
		}
		if (PVvalue <= u9900Database.db.LOOP_SET_LOW_LIMIT.floatVal)
		{
			PVvariableStatus |= LIM_STATUS_LOW;
		}
		break;
	case UPDATE_STATUS_SENSOR_NOT_PRESENT:
	case UPDATE_STATUS_SENSOR_UNDEFINED:
	case UPDATE_STATUS_WRONG_SENSOR_FOR_TYPE:
	default:
		PVvariableStatus = VAR_STATUS_BAD;
		break;
	}
	// Per +GF+, set the "PV out of range" bit when the status is anything other
	// than good from the 9900
	if (UPDATE_STATUS_GOOD == varStatus)
	{
		clrPrimaryStatusBits(FD_STATUS_PV_OUT_OF_LIMITS);
		clrSecondaryStatusBits(FD_STATUS_PV_OUT_OF_LIMITS);
		startUpDataLocalV.StandardStatus0 &= ~SS0_HARDWARE_PROBLEM;
		if (varStatus != lastVarStatus)
		{ 
			clrPrimaryMoreAvailable();
			clrSecondaryMoreAvailable();
		}
	}
	else
	{
		setPrimaryStatusBits(FD_STATUS_PV_OUT_OF_LIMITS);
		setSecondaryStatusBits(FD_STATUS_PV_OUT_OF_LIMITS);
		startUpDataLocalV.StandardStatus0 |= SS0_HARDWARE_PROBLEM;
		if (varStatus != lastVarStatus)
		{ 
			setPrimaryMoreAvailable();
			setSecondaryMoreAvailable();
		}
	}
	// Save the var status for the next update
	lastVarStatus = varStatus;	
	// reset the update timer
	UpdateMsgTimeout = 0;
}

