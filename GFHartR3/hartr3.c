///////////////////////////////////////////////////////////////////////////////////////////
//
// Module Name:  hart.c
//
// Functional Unit: 
//
// Description:
//
// This module provides HART interface. 
//
// Exported Interfaces:
//
// hart.h - This interface file includes all exported interfaces
//               from this module.
//
// Document Reference(s):
//
// PDD-xxxx
//
// Implementation Notes:
//
// 
//
// Revision History:
// Date    Rev.   Engineer     Description
// -------- ----- ------------ --------------
// 04/01/11  0    Vijay Soni    Creation
//
// 04/02/11  1    Patel    	Add funcations
//
//
///////////////////////////////////////////////////////////////////////////////////////////
//==============================================================================
//  INCLUDES
//==============================================================================
#include <msp430f5528.h>
#include <string.h>
#include "hardware.h"
#include "protocols.h"
#include "utilitiesr3.h"
#include "main9900r3.h"
#include "hartr3.h"

//==============================================================================
//  LOCAL DEFINES
//==============================================================================

//==============================================================================
//  LOCAL PROTOTYPES.
//==============================================================================
//==============================================================================
//  GLOBAL DATA
//==============================================================================
unsigned char updateNvRam;      //!< A flag so NV ram is only updated in the main loop
//==============================================================================
//  LOCAL DATA
//==============================================================================

//==============================================================================
// FUNCTIONS
//==============================================================================

///!MH NEED TO ORGANIZE VARIABLE & DEFINE SCOPES

// The process variables
float PVvalue = 0.0;
float SVvalue = 0.0;
float ma4_20 = 0.0;
float savedLoopCurrent = 0.0;
float reportingLoopCurrent = 0.0;


// If the device is busy, we need to respond appropriately
unsigned char deviceBusyFlag = FALSE;
// Flags to indicate a HART command has been received
unsigned char cmdSyncToRam = FALSE;
unsigned char cmdSyncToFlash = FALSE;
unsigned char cmdReset = FALSE;
unsigned char doNotRespond = FALSE;

// The programmed Device ID must be set in a different area of FLASH
// and copied into the local data on initialization
const unsigned char * pNvDevId = NV_DEVICE_ID_LOCATION;


// The startup data
HART_STARTUP_DATA_NONVOLATILE startUpDataLocalNv;
HART_STARTUP_DATA_VOLATILE startUpDataLocalV;

const HART_STARTUP_DATA_NONVOLATILE startUpDataFactoryNv =
{
	//"+GF+ Signet 9900\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",		 	// Name
	//FLOW_TYPE, 					// default sensor
	//FALSE,						// from primary
	0,							// primary status
	0,							// secondary status
	//EXPANDED_DEV_TYPE,			// expanded device type
	//XMIT_PREAMBLE_BYTES,		// min M -> S preambles
	//HART_MAJ_REV,				// HART rev
	//DEVICE_REVISION,			// device revision
	//SOFTWARE_REVISION,			// sw revision
	//HARDWARE_REVISION,			// hw revision
	//SIGNAL_CODE,				// physical signal code
	//FLAGS,						// flags
	{0x00, 0x00, 0x01},			// default device ID
	//XMIT_PREAMBLE_BYTES,		// min S -> M preambles
	//MAX_DEV_VARS,				// Maximum device variables
	CURRENT_CONFIG_CNT,			// configuration change counter
	//EXT_FLD_STATUS,				// extended field device status
	GF_MFR_ID,					// manufacturer ID
	//GF_MFR_ID,					// private label distributor code
	//DEVICE_PROFILE,				// Device Profile
	{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},  // Long tag is all '?'
	{0x20,0x33,0xcd,0x36,0x08,0x20},	// tag = "HCOMM   "
	{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},	// tag descriptor - all '?'
	{1,1,0x64},					// date
	0,							// polling address
	{0x97,0x11,0x6B},			// final assembly = "9900395d"
	{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},	// HART message - all '?'
	//0,							// Alert Status
	//0,							// Error Status
	//{0,0,0,0,0,0},				// Device Specific Status
	//0,							// Device Op Mode Status
	//0,							// Std Status 0
 	CURRENT_MODE_ENABLE  //,		// Current Mode
 	//{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}		// Error Counts
 };

const HART_STARTUP_DATA_VOLATILE startUpDataFactoryV =
{
	"+GF+ Signet 9900\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",		 	// Name
	FLOW_TYPE, 					// default sensor
	FALSE,						// from primary
	//0,							// primary status
	//0,							// secondary status
	EXPANDED_DEV_TYPE,			// expanded device type
	XMIT_PREAMBLE_BYTES,		// min M -> S preambles
	HART_MAJ_REV,				// HART rev
	DEVICE_REVISION,			// device revision
	SOFTWARE_REVISION,			// sw revision
	HARDWARE_REVISION,			// hw revision
	SIGNAL_CODE,				// physical signal code
	FLAGS,						// flags
	//{0x00, 0x00, 0x01},			// default device ID
	XMIT_PREAMBLE_BYTES,		// min S -> M preambles
	MAX_DEV_VARS,				// Maximum device variables
	//CURRENT_CONFIG_CNT,			// configuration change counter
	EXT_FLD_STATUS,				// extended field device status
	//GF_MFR_ID,					// manufacturer ID
	GF_MFR_ID,					// private label distributor code
	DEVICE_PROFILE,				// Device Profile
	//{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // Long tag
	//{0x22,0x00,0xCF,0x34,0xD7,0xDF},	// tag
	//{0,0,0,0,0,0,0,0,0,0,0,0},	// tag descriptor
	//{1,1,0x64},					// date
	//0,							// polling address
	//{0x00,0x00,0x01},			// final assembly
	//{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},	// HART message
	//0,							// Alert Status
	//0,							// Error Status
	{0,0,0,0,0,0},				// Device Specific Status
	0,							// Device Op Mode Status
	0,							// Std Status 0
 	//CURRENT_MODE_ENABLE,		// Current Mode
 	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}		// Error Counts
 };

void incrementConfigCount(void)
{
	startUpDataLocalNv.configChangeCount++;
	// Set up to write RAM to FLASH
	updateNvRam = TRUE;					
}

void setPrimaryMasterChg(void)
{
	startUpDataLocalNv.Primary_status |= FD_STATUS_CONFIG_CHANGED;
	// Set up to write RAM to FLASH
	updateNvRam = TRUE;					
}

void setSecondaryMasterChg(void)
{
	startUpDataLocalNv.Secondary_status |= FD_STATUS_CONFIG_CHANGED;
	// Set up to write RAM to FLASH
	updateNvRam = TRUE;					
}

void clrPrimaryMasterChg(void)
{
	startUpDataLocalNv.Primary_status &= ~FD_STATUS_CONFIG_CHANGED;
	// Set up to write RAM to FLASH
	updateNvRam = TRUE;					
}

void clrSecondaryMasterChg(void)
{
	startUpDataLocalNv.Secondary_status &= ~FD_STATUS_CONFIG_CHANGED;
	// Set up to write RAM to FLASH
	updateNvRam = TRUE;					
}

void setPrimaryStatusBits(unsigned char bits)
{
	startUpDataLocalNv.Primary_status |= bits;
	// Set up to write RAM to FLASH
	updateNvRam = TRUE;					
}

void setSecondaryStatusBits(unsigned char bits)
{
	startUpDataLocalNv.Secondary_status |= bits;
	// Set up to write RAM to FLASH
	updateNvRam = TRUE;					
}

void clrPrimaryStatusBits(unsigned char bits)
{
	startUpDataLocalNv.Primary_status &= ~bits;
	// Set up to write RAM to FLASH
	updateNvRam = TRUE;					
}

void clrSecondaryStatusBits(unsigned char bits)
{
	startUpDataLocalNv.Secondary_status &= ~bits;
	// Set up to write RAM to FLASH
	updateNvRam = TRUE;					
}

void syncNvRam(void)
{
	// Set busy flag
	deviceBusyFlag = TRUE;
	syncToFlash(VALID_SEGMENT_1, ((unsigned char *)&startUpDataLocalNv), 
		sizeof(HART_STARTUP_DATA_NONVOLATILE));
	// clear busy flag
	deviceBusyFlag = FALSE;
}

void setPrimaryMoreAvailable(void)
{
	startUpDataLocalNv.Primary_status |= FD_STATUS_MORE_STATUS_AVAIL;
	// Set up to write RAM to FLASH
	updateNvRam = TRUE;					
}

void setSecondaryMoreAvailable(void)
{
	startUpDataLocalNv.Secondary_status |= FD_STATUS_MORE_STATUS_AVAIL;
	// Set up to write RAM to FLASH
	updateNvRam = TRUE;					
}

void clrPrimaryMoreAvailable(void)
{
	startUpDataLocalNv.Primary_status &= ~FD_STATUS_MORE_STATUS_AVAIL;
	// Set up to write RAM to FLASH
	updateNvRam = TRUE;					
}

void clrSecondaryMoreAvailable(void)
{
	startUpDataLocalNv.Secondary_status &= ~FD_STATUS_MORE_STATUS_AVAIL;
	// Set up to write RAM to FLASH
	updateNvRam = TRUE;					
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
int isAddressValid(void)
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
// Function Name: sendHartFrame()
//
// Description:
//
// Starts the transmit of the completed response 
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
void sendHartFrame (void)
{
	// clear the Lrc byte
	szLrc = 0;
	// Set the transmit state machine to send a preamble
	ePresentXmitState = eXmitPreamble;
	// set up to transmit the right number of preambles
	respXmitIndex = XMIT_PREAMBLE_BYTES;
	//MH Note that Right order is: 1) prepare RTS 2) Write to SBUF 3) Enable TX IE.
	// Enable the transmit interrupt
	enableTxIntr();
	// Turn on RTS
	rtsXmit();
	// send the first preamble byte
	// HART_TXBUF = HART_PREAMBLE; (it was commented out)

}

// LRC Functions

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: processLrcOnBuffer()
//
// Description:
//
// Calculates the LRC on a complete buffer. Returns the LRC value.
//
// Parameters: 
//     int bufSize: the number of bytes in the buffer
//
// Return Type: unsigned char.
//
// Implementation notes:
//
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
unsigned char processLrcOnBuffer(int bufSize)
{
	int i;
	unsigned char Lrc = 0;
	
	for (i = XMIT_PREAMBLE_BYTES; i < bufSize; ++i)
	{
		Lrc ^= szHartResp[i];
	}
	return Lrc;
}

// Utilities to copy ints and floats into the response buffer
///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: copyLongToRespBuf()
//
// Description:
//
// Copies a long int value into the response buffer
//
// Parameters: 
//     long - the  number to copy
//
// Return Type: void
//
// Implementation notes:
//		The function increases respBufferSize appropriately
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void copyLongToRespBuf(long iVal)
{
	U_LONG_INT temp;
	
	temp.i = iVal;
	// Copy in the number in backwards
	szHartResp[respBufferSize] = temp.b[3];   
	++respBufferSize;							
	szHartResp[respBufferSize] = temp.b[2];   
	++respBufferSize;							
	szHartResp[respBufferSize] = temp.b[1];   
	++respBufferSize;							
	szHartResp[respBufferSize] = temp.b[0];   
	++respBufferSize;							
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: copyIntToRespBuf()
//
// Description:
//
// Copies a short int value into the response buffer
//
// Parameters: 
//     int - the int number to copy
//
// Return Type: void
//
// Implementation notes:
//		The function increases respBufferSize appropriately
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void copyIntToRespBuf(int iVal)
{
	U_SHORT_INT temp;
	
	temp.i = iVal;
	// Copy in the number in backwards
	szHartResp[respBufferSize] = temp.b[1];   
	++respBufferSize;							
	szHartResp[respBufferSize] = temp.b[0];   
	++respBufferSize;							
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: copyFloatToRespBuf()
//
// Description:
//
// Copies a floating point value into the response buffer
//
// Parameters: 
//     float - the floating point number to copy
//
// Return Type: void
//
// Implementation notes:
//		The function increases respBufferSize appropriately
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void copyFloatToRespBuf(float fVal)
{
	U_LONG_FLOAT temp;
	
	temp.fVal = fVal;
	// Copy in the number in backwards
	szHartResp[respBufferSize] = temp.b[3];   
	++respBufferSize;							
	szHartResp[respBufferSize] = temp.b[2];   
	++respBufferSize;							
	szHartResp[respBufferSize] = temp.b[1];   
	++respBufferSize;							
	szHartResp[respBufferSize] = temp.b[0];   
	++respBufferSize;							
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: decodeBufferFloat()
//
// Description:
//
// Decodes a floating point value from the command or response buffer
//
// Parameters: 
//     unsigned char * pBuffer - the pointer to the first byte of the number to decode
//
// Return Type: float
//
// Implementation notes:
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
float decodeBufferFloat(unsigned char * pBuffer)
{
	U_LONG_FLOAT temp;
	
	temp.b[3] = *pBuffer;
	temp.b[2] = *(pBuffer+1);
	temp.b[1] = *(pBuffer+2);
	temp.b[0] = *(pBuffer+3);
	
	return temp.fVal;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: decodeBufferInt()
//
// Description:
//
// Decodes an integer value from the command or response buffer
//
// Parameters: 
//     unsigned char * pBuffer - the pointer to the first byte of the number to decode
//
// Return Type: int
//
// Implementation notes:
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
int decodeBufferInt(unsigned char * pBuffer)
{
	U_SHORT_INT temp;
	
	temp.b[1] = *pBuffer;
	temp.b[0] = *(pBuffer+1);
	return temp.i;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: UpdateSensorType()
//
// Description:
//
// 		copies the 9900 downloades sensor type to the local startup data
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void UpdateSensorType(void)
{
	startUpDataLocalV.defaultSensorType = u9900Database.db.MEASUREMENT_TYPE;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: copyNvDeviceIdToRam()
//
// Description:
//
// 		Copies the NV Device ID to the local startup data if it is not 0xFFFFFF (erased).
//      Otherwise, the RAM value is left as-is
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void copyNvDeviceIdToRam(void)
{
	// Define the erased FLASH pattern
	unsigned char erasedValue[DEVICE_ID_SIZE] = {0xFF, 0xFF, 0xFF};
	// Only copy if the FLASH is NOT erased
	if (memcmp(pNvDevId, erasedValue, DEVICE_ID_SIZE))
	{
		memcpy(startUpDataLocalNv.DeviceID, pNvDevId, DEVICE_ID_SIZE);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: copyDeviceIdToFlash()
//
// Description:
//
// 	   Copies the Device ID to the start of FLASH segment 3
//
// Parameters:
//
//     unsigned char * - pointer to the first byte of device ID
//
// Return Type: int - TRUE if the copy was successful, FALSE otherwise
//
// Implementation notes:
//		Erases segment 3 before copying, just in case the value is being changed
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
int copyDeviceIdToFlash(unsigned char * pDeviceId)
{
	int rtnFlag;
	// Erase segment 3
	rtnFlag = eraseMainSegment(NV_DEVICE_ID_LOCATION, MAIN_SEGMENT_SIZE);
	// copy in the new value
	if (rtnFlag)
	{
		rtnFlag = copyMemToMainFlash(NV_DEVICE_ID_LOCATION, pDeviceId, DEVICE_ID_SIZE);
	}
	// check the new value against the supplied value, and return TRUE if it matches
	if (rtnFlag)
	{
		rtnFlag = verifyFlashContents(NV_DEVICE_ID_LOCATION, pDeviceId, DEVICE_ID_SIZE);
	}
	return rtnFlag;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: verifyDeviceId()
//
// Description:
//
// 		Checks to see if the Device ID is correct, and loads it if it isn't
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void verifyDeviceId(void)
{
	int numSegsToErase;
	// Make sure the FLASH device ID is programmed before proceeding
	// Define the erased FLASH pattern
	unsigned char erasedValue[DEVICE_ID_SIZE] = {0xFF, 0xFF, 0xFF};
	// Only continue if the FLASH is NOT erased
	if (memcmp(pNvDevId, erasedValue, DEVICE_ID_SIZE))
	{
		// Now check to see if what is in FLASH and RAM are identical. Do nothing if 
		// they're the same.
		if (memcmp(pNvDevId, startUpDataLocalNv.DeviceID, DEVICE_ID_SIZE))
		{
			// Copy the correct ID into RAM
			copyNvDeviceIdToRam();
			// Now make sure it is sync'd up:
			// erase the segment of FLASH so it can be reprogrammed
			numSegsToErase = calcNumSegments (sizeof(HART_STARTUP_DATA_NONVOLATILE));
			eraseMainSegment(VALID_SEGMENT_1, (numSegsToErase*MAIN_SEGMENT_SIZE));
			// Copy the local data into NV memory
			copyMemToMainFlash (VALID_SEGMENT_1, ((unsigned char *)&startUpDataLocalNv), 
				sizeof(HART_STARTUP_DATA_NONVOLATILE));
		}
	}
}






