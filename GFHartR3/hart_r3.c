/*!
 *  \file   hart_r3.c
 *  \brief  This module provides HART interface.
 *  Created on: Sep 20, 2012
 *  \author:
 *  Revision History:
 *  Date    Rev.   Engineer     Description
 *  -------- ----- ------------ --------------
 *  04/01/11  0    Vijay Soni    Creation
 *  04/02/11  1    Patel     Add funcations
 *
 */

///////////////////////////////////////////////////////////////////////////////////////////
//
// Module Name:
//
// Functional Unit: 
//
// Description:
//
//
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
#include "utilities_r3.h"
#include "main9900_r3.h"
#include "hart_r3.h"

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
	//unsigned char erasedValue[DEVICE_ID_SIZE] = {0xFF, 0xFF, 0xFF}; //MH Initialization is wrong
  unsigned char erasedValue[DEVICE_ID_SIZE];
  memset(erasedValue,0xFF,DEVICE_ID_SIZE);    //MH erasedValue[0..DEVICE_ID_SIZE-1]= 0xFF
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
	//unsigned char erasedValue[DEVICE_ID_SIZE] = {0xFF, 0xFF, 0xFF}; //MH Initialization is wrong
	unsigned char erasedValue[DEVICE_ID_SIZE];
	memset(erasedValue,0xFF,DEVICE_ID_SIZE);      //MH erasedValue[0..DEVICE_ID_SIZE-1]= 0xFF
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






