///////////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) AB Tech Solution LLC 2011 All Rights Reserved.
// This code may not be copied without the express written consent of 
// AB Tech Solution LLC.
//
//
// Client: GF
//
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY 
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
///////////////////////////////////////////////////////////////////////////////////////////
//
// Module Name:  common_h_cmd.c
//
// Functional Unit: 
//
// Description:
//
// This module provides command handling for flow sensor commands. 
//
// Exported Interfaces:
//
// common_h_cmd.h - This interface file includes all exported interfaces
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
// 04/23/11  0    Vijay Soni    Creation
//
//
///////////////////////////////////////////////////////////////////////////////////////////
 
#include <msp430f5528.h>
#include <string.h>
#include "hardware.h"
#include "protocols.h"
#include "hartr3.h"
#include "main9900r3.h"
#include "utilitiesr3.h"
#include "common_h_cmdr3.h"
#include "merge.h"
// This flag is used by commands 11 & 21 to indicate the tag did not
// match, and that processHartCommand() should return false. All other
// commands set the value to false.
unsigned char badTagFlag = FALSE;


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_0()
//
// Description:
//
// Process the HART command 0 : Transmit Unique ID
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//  MH- Added Byte order from HCF_SPEC-127  Rev 7.1
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void common_cmd_0(void)
{
	// Byte Count
	szHartResp[respBufferSize] = 24;  // Byte count
	++respBufferSize;							
	// Device Status High Byte
	szHartResp[respBufferSize] = 0;   // Device Status high byte
	++respBufferSize;					
	// Status Low Byte		
	szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
		startUpDataLocalNv.Primary_status : 
		startUpDataLocalNv.Secondary_status;  // status byte
	++respBufferSize;							
	// 0 - Response Code
	szHartResp[respBufferSize] = 254;  // Response Code
	++respBufferSize;	
	// 1-2 Expanded Device Type
	copyIntToRespBuf(startUpDataLocalV.expandedDevType);
	// 3  Min number of preambles M -> S
	szHartResp[respBufferSize] = startUpDataLocalV.minPreamblesM2S;
	++respBufferSize;	
	// 4  HART revision
	szHartResp[respBufferSize] = startUpDataLocalV.majorHartRevision;
	++respBufferSize;	
	// 5  Device Revision
	szHartResp[respBufferSize] = startUpDataLocalV.deviceRev;
	++respBufferSize;	
	// 6  sw revision
	szHartResp[respBufferSize] = startUpDataLocalV.swRev;
	++respBufferSize;	
	// 7  hw revision (5 msb) & signal code (3-lsb) combined
	unsigned char temp = startUpDataLocalV.hwRev << 3;
	temp |= (startUpDataLocalV.physSignalCode & 0x07);
	szHartResp[respBufferSize] = temp;
	++respBufferSize;	
	// 8  flags
	szHartResp[respBufferSize] = startUpDataLocalV.flags;
	++respBufferSize;	
	// 9-11 Device ID
	memcpy(&(szHartResp[respBufferSize]), &startUpDataLocalNv.DeviceID, 3);
	respBufferSize += 3;
	// 12 Min number of preambles S -> M
	szHartResp[respBufferSize] = startUpDataLocalV.minPreamblesS2M;
	++respBufferSize;	
	// 13 Max device vars
	szHartResp[respBufferSize] = startUpDataLocalV.maxNumDevVars;
	++respBufferSize;	
	// 14-15  config change counter
	copyIntToRespBuf(startUpDataLocalNv.configChangeCount);
	// 16 Extended field device status
	szHartResp[respBufferSize] = startUpDataLocalV.extendFieldDevStatus;  
	++respBufferSize;							
	// 17-18 Manufacturer ID
	copyIntToRespBuf(startUpDataLocalNv.ManufacturerIdCode);
	// 19-20  Private Label distributor ID
	copyIntToRespBuf(startUpDataLocalV.PrivDistCode);
	// 21 Device Profile
	szHartResp[respBufferSize] = startUpDataLocalV.devProfile;  
	++respBufferSize;		
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_1()
//
// Description:
//
// Process the HART command 1 : Read PV 
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
void common_cmd_1(void)
{
	unsigned char respCode = (updateDelay) ? UPDATE_FAILURE : RESP_SUCCESS;
	
	if (respCode)
	{
		szHartResp[respBufferSize] = 3;  // Byte count
	}
	else
	{
		szHartResp[respBufferSize] = 7;  // Byte count
	}
	++respBufferSize;							
	szHartResp[respBufferSize] = 0;   // Device Status high byte
	++respBufferSize;							
	szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
		startUpDataLocalNv.Primary_status : 
		startUpDataLocalNv.Secondary_status;  // Status byte
	++respBufferSize;
	if (respCode)
	{
		szHartResp[respBufferSize] = respCode;   // Command-specific response code
		++respBufferSize;							
	}
	else
	{							
		szHartResp[respBufferSize] = u9900Database.db.UnitsPrimaryVar;   // PV Units
		++respBufferSize;
		copyFloatToRespBuf(PVvalue);
	}
}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_2()
//
// Description:
//
// Process the HART command 2 : Transmit PV current, PV % range
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
void common_cmd_2(void)
{
	// Capture the loop current value first so it doesn't change
	// while building the response
	float loopCurrent = (TRUE == updateInProgress) ? reportingLoopCurrent : ma4_20;
	float pvPercent = CalculatePercentRange(u9900Database.db.LOOP_SET_HIGH_LIMIT.floatVal,
		u9900Database.db.LOOP_SET_LOW_LIMIT.floatVal, PVvalue);
	
	unsigned char respCode = (updateDelay) ? UPDATE_FAILURE : RESP_SUCCESS;
	if (respCode)
	{
		szHartResp[respBufferSize] = 3;  // Byte count
	}
	else
	{
		szHartResp[respBufferSize] = 10;  // Byte count
	}
	++respBufferSize;							
	szHartResp[respBufferSize] = 0;   // Device Status high byte
	++respBufferSize;							
	szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
		startUpDataLocalNv.Primary_status : 
		startUpDataLocalNv.Secondary_status;  // Status byte
	++respBufferSize;		
	if (respCode)
	{
		szHartResp[respBufferSize] = respCode;   // Command-specific response code
		++respBufferSize;							
	}
	else
	{
		// Loop current
		copyFloatToRespBuf(loopCurrent);
		// Now PV as a % of range	
		copyFloatToRespBuf(pvPercent);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_3()
//
// Description:
//
// Process the HART command 3 : Transmit current PV & dynamic variables
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
void common_cmd_3(void)
{
	float loopCurrent = (TRUE == updateInProgress) ? reportingLoopCurrent : ma4_20;
	unsigned char respCode = (updateDelay) ? UPDATE_FAILURE : RESP_SUCCESS;
	// Make sure the SV units returned are capped at 250 
	unsigned char SVunits = (250 <= u9900Database.db.UnitsSecondaryVar) ? NOT_USED : 
		u9900Database.db.UnitsSecondaryVar;
	if (respCode)
	{
		szHartResp[respBufferSize] = 3;  // Byte count
	}
	else
	{
		// returning both PV & SV
		szHartResp[respBufferSize] = 16;  // Byte count
	}
	++respBufferSize;							
	szHartResp[respBufferSize] = 0;   // Device Status high byte
	++respBufferSize;							
	szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
		startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
	++respBufferSize;	
	if (respCode)
	{
		// Cmd-specific response code
		szHartResp[respBufferSize] = respCode;   // Command-specific response code
		++respBufferSize;							
	}
	else
	{
		// Loop current first	
		copyFloatToRespBuf(loopCurrent);					
		// Now the PV
		szHartResp[respBufferSize] = u9900Database.db.UnitsPrimaryVar;
		++respBufferSize;
		copyFloatToRespBuf(PVvalue);					
		if (NOT_USED == SVunits)
		{
#if 1		
			// Return CELSIUS as a unit code with a non-signaling NaN value
			// Unit code cannot be 0 or 250, it must be what it would be in
			// a different configuration, per Sean Vincent @ HCF. Since Celsius
			// is the most likely SV unit, use it.	
			// NOTE: UAL011b may still fail, because it does not handle non-signaling
			// NaN values. In this case, HCF must evaluate the test case manually to 
			// verify compliance.
			szHartResp[respBufferSize] = CELSIUS;
			++respBufferSize;
			// Return Non-signaling NaN per Sean Vincent
			szHartResp[respBufferSize] = 0x7f;
			++respBufferSize;
			szHartResp[respBufferSize] = 0xff;
			++respBufferSize;
			szHartResp[respBufferSize] = 0xff;
			++respBufferSize;
			szHartResp[respBufferSize] = 0xff;
			++respBufferSize;
#else
			szHartResp[respBufferSize] = NOT_USED;
			++respBufferSize;
			// Return Non-signaling NaN per Sean Vincent
			szHartResp[respBufferSize] = 0x7f;
			++respBufferSize;
			szHartResp[respBufferSize] = 0xA0;
			++respBufferSize;
			szHartResp[respBufferSize] = 0;
			++respBufferSize;
			szHartResp[respBufferSize] = 0;
			++respBufferSize;
#endif			
		}
		else
		{
			szHartResp[respBufferSize] = SVunits;
			++respBufferSize;
			copyFloatToRespBuf(SVvalue);
		}
	}					
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: flow_cmd_6()
//
// Description:
//
// Process the HART command 6 : Save Polling address
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
void common_cmd_6(void)
{
	unsigned char pollAddress;
	// First check to see if we have too few bytes
	unsigned char respCode = (0 == hartDataCount) ? TOO_FEW_DATA_BYTES : RESP_SUCCESS;
	// If we have enough bytes, make sure the poll address is valid
	if (!respCode)
	{
		pollAddress = (longAddressFlag) ? szHartCmd[LONG_DATA_OFFSET] : szHartCmd[SHORT_DATA_OFFSET];
		if (63 < pollAddress)
		{
			respCode = INVALID_POLL_ADDR_SEL;
		}
	}
	// If we have a non-zero code, send back the error return
	if (respCode)
	{
		common_tx_error(respCode);
	}
	else // We can execute the command from here
	{
		// Change the poll address in the local startup data
		startUpDataLocalNv.PollingAddress = pollAddress;
		// Now grab the current mode from the command buffer if it is there
		if (2 == hartDataCount)
		{
			startUpDataLocalNv.currentMode = (longAddressFlag) ? szHartCmd[LONG_DATA_OFFSET+1] : szHartCmd[SHORT_DATA_OFFSET+1];
		}
		else if ((1 == hartDataCount) && (0 < pollAddress))
		{
			// Current mode is disabled for a single byte, non-0 poll address
			startUpDataLocalNv.currentMode = CURRENT_MODE_DISABLE;
		}
		else if ((1 == hartDataCount) && (0 == pollAddress))
		{
			// Current mode is disabled for a single byte, non-0 poll address
			startUpDataLocalNv.currentMode = CURRENT_MODE_ENABLE;
		}
		// Now set the current based upon the command
		if (CURRENT_MODE_DISABLE == startUpDataLocalNv.currentMode)
		{
			// Tell the 9900 to go to fixed current mode at 4 mA
			setFixedCurrentMode(4.0);
			setPrimaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
			setSecondaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
		}
		else
		{
			// Tell the 9900 to go to loop reporting current mode
			setFixedCurrentMode(0.0);
			clrPrimaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
			clrSecondaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
		}
		// Set the change flags
		setPrimaryMasterChg();
		setSecondaryMasterChg();
		incrementConfigCount();
		// Set up to write RAM to FLASH
		updateNvRam = TRUE;					
		// Now build the response buffer
		szHartResp[respBufferSize] = 4;
		++respBufferSize;
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;
		// Parrot back the poll address							
		szHartResp[respBufferSize] = startUpDataLocalNv.PollingAddress;   
		++respBufferSize;	
		// Return the current mode
		szHartResp[respBufferSize] = startUpDataLocalNv.currentMode; 
		++respBufferSize;	
	}	
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_7()
//
// Description:
//
// Process the HART command 7 : Read Loop Configuration
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
void common_cmd_7(void)
{
	// This command always succeeds
	szHartResp[respBufferSize] = 4;  // Byte count
	++respBufferSize;							
	szHartResp[respBufferSize] = 0;   // Device Status high byte
	++respBufferSize;							
	szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
		startUpDataLocalNv.Primary_status : 
		startUpDataLocalNv.Secondary_status;  // Status byte
	++respBufferSize;		
	szHartResp[respBufferSize] = startUpDataLocalNv.PollingAddress;   // Polling address
	++respBufferSize;							
	szHartResp[respBufferSize] = startUpDataLocalNv.currentMode;   // Loop current mode
	++respBufferSize;							
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_8()
//
// Description:
//
// Process the HART command 8 : Read dynamic Variable classification
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
void common_cmd_8(void)
{
	szHartResp[respBufferSize] = 6;  // Byte count
	++respBufferSize;							
	szHartResp[respBufferSize] = 0;   // Device Status high byte
	++respBufferSize;							
	szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
		startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
	++respBufferSize;	
	// units are the PV classification
	szHartResp[respBufferSize] = u9900Database.db.Hart_Dev_Var_Class;   
	++respBufferSize;			
	// units are the SV units				
	szHartResp[respBufferSize] = 0;   
	++respBufferSize;	
	// units are the TV units				
	szHartResp[respBufferSize] = NOT_USED;   
	++respBufferSize;	
	// units are the QV units				
	szHartResp[respBufferSize] = NOT_USED;   
	++respBufferSize;	
}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_9()
//
// Description:
//
// Process the HART command 9 : Read Device Variables with Status
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
void common_cmd_9(void)
{
	float loopCurrent = (TRUE == updateInProgress) ? reportingLoopCurrent : ma4_20;
	float pvPercent = CalculatePercentRange(u9900Database.db.LOOP_SET_HIGH_LIMIT.floatVal,
		u9900Database.db.LOOP_SET_LOW_LIMIT.floatVal, PVvalue);
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	// Make sure we don't respond to more than 8 requests
	unsigned char numVars = (hartDataCount > 8) ? 8 : hartDataCount;
	unsigned char index, reqVar, reqVarIdx;
	// Did we request too few variables?
	if (0 == hartDataCount)
	{
		respCode = TOO_FEW_DATA_BYTES;
	}
	// get the index to the first requested variable
	reqVarIdx = respBufferSize+1;
	// Now determine if there is an invalid selection of 0xFF. If any requested variable
	// is illegal, set the response code & exit with error
	for (index = 0; index < numVars; ++index)
	{
		if (DVC_INVALID_SELECTION == szHartCmd[index + reqVarIdx])
		{
			respCode = INVALID_SELECTION;
			break;
		}
	}
	if (respCode)
	{
		common_tx_error(respCode);
	}
	else
	{
		// Calculate response size
		szHartResp[respBufferSize] = (numVars * 8) + 7;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = respCode;   // Device Status high byte (response code)
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;							
		// First byte: extended status
		szHartResp[respBufferSize] = startUpDataLocalV.extendFieldDevStatus;
		++respBufferSize;							
		// now loop through each request. Only respond t 0 & 1
		for (index = 0; index < numVars; ++index)
		{
			// What variable is requested?
			reqVar = szHartCmd[index + reqVarIdx];
			// what we do depends on the variable requested
			switch (reqVar)
			{
			case DVC_PV: // return PV
			case DVC_PRIMARY_VARIABLE:
				// Device Variable code
				szHartResp[respBufferSize] = reqVar;   
				++respBufferSize;		
				// Device variable classification - read from 9900 database					
				szHartResp[respBufferSize] = u9900Database.db.Hart_Dev_Var_Class;   
				++respBufferSize;			
				// Units code				
				szHartResp[respBufferSize] = u9900Database.db.UnitsPrimaryVar;   
				++respBufferSize;
				// Value
				copyFloatToRespBuf(PVvalue);
				// Status							
				szHartResp[respBufferSize] = PVvariableStatus;   
				++respBufferSize;			
				break;
			case DVC_SV: // return SV
			case DVC_SECONDARY_VARIABLE:
				// Device Variable code
				szHartResp[respBufferSize] = reqVar;   
				++respBufferSize;
				if (NOT_USED > u9900Database.db.UnitsSecondaryVar)
				{		
					// Device variable classification					
					szHartResp[respBufferSize] = u9900Database.db.Hart_Dev_Var_Class;   
					++respBufferSize;			
					// Units code				
					szHartResp[respBufferSize] = u9900Database.db.UnitsSecondaryVar;   
					++respBufferSize;
					// Value
					copyFloatToRespBuf(SVvalue);
					// Status - it is just considered good							
					szHartResp[respBufferSize] = PVvariableStatus;   
					++respBufferSize;			
				}
				else
				{
					// Device variable classification					
					szHartResp[respBufferSize] = 0;   
					++respBufferSize;		
#if 1						
					// Return CELSIUS as a unit code with a non-signaling NaN value
					// Unit code cannot be 0 or 250, it must be what it would be in
					// a different configuration, per Sean Vincent @ HCF. Since Celsius
					// is the most likely SV unit, use it.	
					// NOTE: UAL011b may still fail, because it does not handle non-signaling
					// NaN values. In this case, HCF must evaluate the test case manually to 
					// verify compliance.
					// Units code				
					szHartResp[respBufferSize] = CELSIUS;   
					++respBufferSize;
					// Value
					szHartResp[respBufferSize] = 0x7F;   
					++respBufferSize;			
					szHartResp[respBufferSize] = 0xFF;   
					++respBufferSize;			
					szHartResp[respBufferSize] = 0xFF;   
					++respBufferSize;			
					szHartResp[respBufferSize] = 0xFF;   
					++respBufferSize;		
#else
					// Units code				
					szHartResp[respBufferSize] = NOT_USED;   
					++respBufferSize;
					// Value
					szHartResp[respBufferSize] = 0x7F;   
					++respBufferSize;			
					szHartResp[respBufferSize] = 0xA0;   
					++respBufferSize;			
					szHartResp[respBufferSize] = 0;   
					++respBufferSize;			
					szHartResp[respBufferSize] = 0;   
					++respBufferSize;		
#endif						
					// Status							
					szHartResp[respBufferSize] = VAR_STATUS_BAD | LIM_STATUS_CONST;   
					++respBufferSize;			
				}
				break;
			case DVC_PERCENT_RANGE:
				// Device Variable code
				szHartResp[respBufferSize] = reqVar;   
				++respBufferSize;		
				// Device variable classification					
				szHartResp[respBufferSize] = u9900Database.db.Hart_Dev_Var_Class;   
				++respBufferSize;			
				// Units code				
				szHartResp[respBufferSize] = PERCENT;   
				++respBufferSize;
				// calculate the % of range
				copyFloatToRespBuf(pvPercent);
				// Status							
				szHartResp[respBufferSize] = PVvariableStatus;   
				++respBufferSize;			
				break;
			case DVC_LOOP_CURRENT:	
				// Device Variable code
				szHartResp[respBufferSize] = reqVar;   
				++respBufferSize;		
				// Device variable classification					
				szHartResp[respBufferSize] = u9900Database.db.Hart_Dev_Var_Class;   
				++respBufferSize;			
				// Units code				
				szHartResp[respBufferSize] = MILLIAMPS;   
				++respBufferSize;
				// Value
				copyFloatToRespBuf(loopCurrent);
				// Status							
				szHartResp[respBufferSize] = PVvariableStatus;   
				++respBufferSize;			
				break;
			default: // return the not supported response
				// Device Variable code
				szHartResp[respBufferSize] = reqVar;   
				++respBufferSize;		
				// Device variable classification					
				szHartResp[respBufferSize] = 0;   
				++respBufferSize;			
				// Units code				
				szHartResp[respBufferSize] = NOT_USED;   
				++respBufferSize;							
				// Value		
				szHartResp[respBufferSize] = 0x7f;   
				++respBufferSize;							
				szHartResp[respBufferSize] = 0xa0;   
				++respBufferSize;							
				szHartResp[respBufferSize] = 0;   
				++respBufferSize;							
				szHartResp[respBufferSize] = 0;   
				++respBufferSize;	
				// Status						
				szHartResp[respBufferSize] = VAR_STATUS_BAD | LIM_STATUS_CONST;   
				++respBufferSize;					
				break;
			}
		}
		// data time stamp 
		copyLongToRespBuf(dataTimeStamp);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_11()
//
// Description:
//
// Process the HART command 11 : Transmit Unique ID if Tag matches
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
void common_cmd_11(void)
{
	if (!memcmp(&(szHartCmd[respBufferSize+1]), &(startUpDataLocalNv.TagName[0]), SHORT_TAG_SIZE))
	{
		badTagFlag = FALSE;
		// the first part of the response is identical to command 0
		common_cmd_0();
	}
	else
	{
		// No response at all
		badTagFlag = TRUE;
		// rtsRcv();   // MH: Not necessary as in "Run to completion" we don't send partial messages
		// Get ready for a new command
		// MH substituted prepareToRxFrame();
		initHartRxSm();
	}							
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_12()
//
// Description:
//
// Process the HART command 12 : Transmit MSG
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
void common_cmd_12(void)
{
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	if (respCode)
	{
		szHartResp[respBufferSize] = 3;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;							
		// Cmd-specific response code
		szHartResp[respBufferSize] = respCode;   // Command-specific response code
		++respBufferSize;							
	}
	else
	{
		szHartResp[respBufferSize] = 26;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;							
		memcpy(&(szHartResp[respBufferSize]), &startUpDataLocalNv.HARTmsg, 24);
		respBufferSize += 24;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_13()
//
// Description:
//
// Process the HART command 13 : Transmit Tag, Descriptor, and date
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
void common_cmd_13(void)
{
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	if (respCode)
	{
		szHartResp[respBufferSize] = 3;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;							
		// Cmd-specific response code
		szHartResp[respBufferSize] = respCode;   // Command-specific response code
		++respBufferSize;							
	}
	else
	{
		szHartResp[respBufferSize] = 23;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;
		// Since the are store together, just write them out	
		memcpy(&(szHartResp[respBufferSize]), &startUpDataLocalNv.TagName, 21);
		respBufferSize += 21;	
	}					
}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: flow_cmd_14()
//
// Description:
//
// Process the HART command 14 : Transmit PV sensor information
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
void common_cmd_14(void)
{
	float span = 0.0;
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	if (respCode)
	{
		szHartResp[respBufferSize] = 3;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;							
		// Cmd-specific response code
		szHartResp[respBufferSize] = respCode;   // Command-specific response code
		++respBufferSize;							
	}
	else
	{
		szHartResp[respBufferSize] = 18;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;	
		// The transducer S/N is 0
		szHartResp[respBufferSize] = 0;   // Sensor S/N
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Sensor S/N
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Sensor S/N
		++respBufferSize;			
		// Transducer units are the PV units
		szHartResp[respBufferSize] = u9900Database.db.UnitsPrimaryVar;   
		++respBufferSize;			
		// Now the High limit from the database				
		copyFloatToRespBuf(u9900Database.db.LOOP_SET_HIGH_LIMIT.floatVal);					
		// Now the low limit from the database				
		copyFloatToRespBuf(u9900Database.db.LOOP_SET_LOW_LIMIT.floatVal);					
		// the minimum span is 0				
		copyFloatToRespBuf(span);					
	}
}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_15()
//
// Description:
//
// Process the HART command 15 : Read Device information
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
void common_cmd_15(void)
{
	float damping = 0.0;
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	if (respCode)
	{
		szHartResp[respBufferSize] = 3;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;							
		// Cmd-specific response code
		szHartResp[respBufferSize] = respCode;   // Command-specific response code
		++respBufferSize;							
	}
	else
	{
		szHartResp[respBufferSize] = 20;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = DEV_STATUS_HIGH_BYTE;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;	
		// Alarm selection
		szHartResp[respBufferSize] = ALARM_CODE_LOOP_HIGH;   
		++respBufferSize;	
		// Transfer function						
		szHartResp[respBufferSize] = XFR_FUNCTION_NONE;   
		++respBufferSize;	
		// Upper & lower range units from the DB
		szHartResp[respBufferSize] = u9900Database.db.UnitsPrimaryVar;   
		++respBufferSize;	
		// Now the High limit from the database				
		copyFloatToRespBuf(u9900Database.db.LOOP_SET_HIGH_LIMIT.floatVal);					
		// Now the low limit from the database				
		copyFloatToRespBuf(u9900Database.db.LOOP_SET_LOW_LIMIT.floatVal);					
		// PV damping value				
		copyFloatToRespBuf(damping);					
		// Write protect code
		szHartResp[respBufferSize] = NO_WRITE_PROTECT;   
		++respBufferSize;	
		// Reserved for now
		szHartResp[respBufferSize] = NOT_USED;   
		++respBufferSize;	
		// Analog channel bits
		szHartResp[respBufferSize] = ANALOG_CHANNEL_FLAG;   
		++respBufferSize;	
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_16()
//
// Description:
//
// Process the HART command 16 : Transmit Final Assembly Number
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
void common_cmd_16(void)
{
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	if (respCode)
	{
		szHartResp[respBufferSize] = 3;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;							
		// Cmd-specific response code
		szHartResp[respBufferSize] = respCode;   // Command-specific response code
		++respBufferSize;							
	}
	else
	{
		szHartResp[respBufferSize] = FINAL_ASSY_SIZE+2;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;
		// Final Assembly
		memcpy(&(szHartResp[respBufferSize]), &startUpDataLocalNv.FinalAssy, FINAL_ASSY_SIZE);
		respBufferSize += FINAL_ASSY_SIZE;	
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_17()
//
// Description:
//
// Process the HART command 17 : Write MSG
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
void common_cmd_17(void)
{
	// First check to see if we have too few bytes
	unsigned char respCode = (HART_MSG_SIZE > hartDataCount) ? TOO_FEW_DATA_BYTES : RESP_SUCCESS;
	// If we have enough bytes, make sure the poll address is valid
	if (!respCode)
	{
		respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	}
	// If we have a non-zero code, send back the error return
	if (respCode)
	{
		common_tx_error(respCode);
	}
	else // We can execute the command from here
	{
		// Set the change flags
		setPrimaryMasterChg();
		setSecondaryMasterChg();
		incrementConfigCount();
		// Copy the data into the local structure
		memcpy(startUpDataLocalNv.HARTmsg, &(szHartCmd[respBufferSize+1]), HART_MSG_SIZE);
		// Set up to write RAM to FLASH
		updateNvRam = TRUE;					
		// Build the response				
		szHartResp[respBufferSize] = HART_MSG_SIZE+2;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;	
		// Write back the message
		memcpy(&(szHartResp[respBufferSize]), &startUpDataLocalNv.HARTmsg, HART_MSG_SIZE);
		respBufferSize += HART_MSG_SIZE;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_18()
//
// Description:
//
// Process the HART command 18 : Write tag, descriptor & date
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
void common_cmd_18(void)
{
	// First check to see if we have too few bytes
	unsigned char respCode = (TAG_DESCRIPTOR_DATE_SIZE > hartDataCount) ? TOO_FEW_DATA_BYTES : RESP_SUCCESS;
	// If we have enough bytes, make sure the poll address is valid
	if (!respCode)
	{
		respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	}
	// If we have a non-zero code, send back the error return
	if (respCode)
	{
		common_tx_error(respCode);
	}
	else // We can execute the command from here
	{
		// Set the change flags
		setPrimaryMasterChg();
		setSecondaryMasterChg();
		incrementConfigCount();
		// Copy the received string directly into the structure
		memcpy(&startUpDataLocalNv.TagName, &(szHartCmd[respBufferSize+1]), TAG_DESCRIPTOR_DATE_SIZE);	
		// Set up to write RAM to FLASH
		updateNvRam = TRUE;					
		// Build response
		szHartResp[respBufferSize] = TAG_DESCRIPTOR_DATE_SIZE+2;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;	
		// Parrot back what was written
		memcpy(&(szHartResp[respBufferSize]), &startUpDataLocalNv.TagName, TAG_DESCRIPTOR_DATE_SIZE);
		respBufferSize += TAG_DESCRIPTOR_DATE_SIZE;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_19()
//
// Description:
//
// Process the HART command 19 : Write final assembly number
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void common_cmd_19(void)
{
	// First check to see if we have too few bytes
	unsigned char respCode = (FINAL_ASSY_SIZE > hartDataCount) ? TOO_FEW_DATA_BYTES : RESP_SUCCESS;
	// If we have enough bytes, make sure the poll address is valid
	if (!respCode)
	{
		respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	}
	// If we have a non-zero code, send back the error return
	if (respCode)
	{
		common_tx_error(respCode);
	}
	else // We can execute the command from here
	{
		// Set the change flags	
		setPrimaryMasterChg();
		setSecondaryMasterChg();
		incrementConfigCount();
		// Copy the final assembly into the database
		memcpy(&startUpDataLocalNv.FinalAssy, &(szHartCmd[respBufferSize+1]), 3);	
		// Set up to write RAM to FLASH
		updateNvRam = TRUE;					
		// Build the response
		szHartResp[respBufferSize] = FINAL_ASSY_SIZE+2;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;	
		// Parrot back the final assy
		memcpy(&(szHartResp[respBufferSize]), &startUpDataLocalNv.FinalAssy, FINAL_ASSY_SIZE);
		respBufferSize += FINAL_ASSY_SIZE;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_20()
//
// Description:
//
// Process the HART command 20 : Read Long Tag
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void common_cmd_20(void)
{
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;

	if (respCode)
	{
		szHartResp[respBufferSize] = 3;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;							
		// Cmd-specific response code
		szHartResp[respBufferSize] = respCode;   // Command-specific response code
		++respBufferSize;							
	}
	else
	{
		szHartResp[respBufferSize] = LONG_TAG_SIZE + 2;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;
		// Final Assembly
		memcpy(&(szHartResp[respBufferSize]), &startUpDataLocalNv.LongTag, LONG_TAG_SIZE);
		respBufferSize += LONG_TAG_SIZE;	
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_21()
//
// Description:
//
// Process the HART command 21 : Read unique ID associated w/ Long Tag
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void common_cmd_21(void)
{
	if (!memcmp(&szHartCmd[respBufferSize+1], &startUpDataLocalNv.LongTag, LONG_TAG_SIZE))
	{
		badTagFlag = FALSE;
		// the first part of the response is identical to command 0
		common_cmd_0();
	}
	else
	{
		// No response at all
		badTagFlag = TRUE;
		// rtsRcv();   // MH: Not necessary as in "Run to completion" we don't send partial messages
		// Get ready for a new command
		//MH substituted prepareToRxFrame();
		initHartRxSm();
	}							
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_22()
//
// Description:
//
// Process the HART command 22 : Write Long Tag
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void common_cmd_22(void)
{
	// First check to see if we have too few bytes
	unsigned char respCode = (LONG_TAG_SIZE > hartDataCount) ? TOO_FEW_DATA_BYTES : RESP_SUCCESS;
	// If we have enough bytes, make sure the poll address is valid
	if (!respCode)
	{
		respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	}
	// If we have a non-zero code, send back the error return
	if (respCode)
	{
		common_tx_error(respCode);
	}
	else // We can execute the command from here
	{
		// Set the change flags	
		setPrimaryMasterChg();
		setSecondaryMasterChg();
		incrementConfigCount();
		// Copy the final assembly into the database
		memcpy(&startUpDataLocalNv.LongTag, &(szHartCmd[respBufferSize+1]), LONG_TAG_SIZE);	
		// Set up to write RAM to FLASH
		updateNvRam = TRUE;					
		// Build the response
		szHartResp[respBufferSize] = LONG_TAG_SIZE+2;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;	
		// Parrot back the final assy
		memcpy(&(szHartResp[respBufferSize]), &startUpDataLocalNv.LongTag, LONG_TAG_SIZE);
		respBufferSize += LONG_TAG_SIZE;
	}
}

#ifdef IMPLEMENT_RANGE_CMDS_35_36_37
///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_35()
//
// Description:
//
// Process the HART command 35 : Write Primary Variable Range Values
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void common_cmd_35(void)
{
	float upper, lower;
	unsigned char units;
	// Are we busy?
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	if (!respCode)
	{
		// Check for too few bytes sent 
		if (9 > hartDataCount)
		{
			respCode = TOO_FEW_DATA_BYTES;
		} 
	}
	// While there are several other possible response codes, the HART processor
	// does not have enough information to process them, so BUSY or SUCCESS are the 
	// only possible choices
	if (respCode)
	{
			common_tx_error(respCode);
	}
	else
	{
		// extract the data from the commands
		if (longAddressFlag)
		{
			units = szHartCmd[LONG_DATA_OFFSET];
			upper = decodeBufferFloat(&(szHartCmd[LONG_DATA_OFFSET+1]));
			lower = decodeBufferFloat(&(szHartCmd[LONG_DATA_OFFSET+5]));
		}
		else
		{
			units = szHartCmd[SHORT_DATA_OFFSET];
			upper = decodeBufferFloat(&(szHartCmd[SHORT_DATA_OFFSET+1]));
			lower = decodeBufferFloat(&(szHartCmd[SHORT_DATA_OFFSET+5]));
		}
		// set up the request
		setBothRangeVals(upper, lower);
		// Now build the response
		// Now build the response buffer
		szHartResp[respBufferSize] = 11;
		++respBufferSize;
		// Send status as usual
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;
		// parrot back the units
		szHartResp[respBufferSize] = units;   // Device Status high byte
		++respBufferSize;							
		// Now parrot back the range values
		copyFloatToRespBuf(upper);
		copyFloatToRespBuf(lower);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_36()
//
// Description:
//
// Process the HART command 36 : Set PV Upper Range Value
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void common_cmd_36(void)
{
	// Are we busy?
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	// While there are several other possible response codes, the HART processor
	// does not have enough information to process them, so BUSY or SUCCESS are the 
	// only possible choices
	if (!respCode)
	{
		// execute
		setUpperRangeVal();
	}
	if (respCode)
	{
		szHartResp[respBufferSize] = 3;  // Byte count
	}
	else
	{
		szHartResp[respBufferSize] = 2;  // Byte count
	}
	++respBufferSize;							
	szHartResp[respBufferSize] = 0;   // Device Status high byte
	++respBufferSize;							
	szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
		startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
	++respBufferSize;
	if (respCode)
	{							
		// Cmd-specific response code
		szHartResp[respBufferSize] = respCode;   // Command-specific response code
		++respBufferSize;	
	}						
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_37()
//
// Description:
//
// Process the HART command 37 : Set PV Lower Range Value
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void common_cmd_37(void)
{
	// Are we busy?
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	// While there are several other possible response codes, the HART processor
	// does not have enough information to process them, so BUSY or SUCCESS are the 
	// only possible choices
	if (!respCode)
	{
		// execute
		setLowerRangeVal();
		szHartResp[respBufferSize] = 2;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;
	}
	else
	{
		common_tx_error(respCode);
	}
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_38()
//
// Description:
//
// Process the HART command 38 : Reset Configuration Changed flag
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void common_cmd_38(void)
{
	unsigned char respCode;
	U_SHORT_INT configCounter;
	// If there are no data bytes, we assume the master is not rev 7, and simply reset the 
	// status bit
	// First check to see if we have too few bytes
	if (0 == hartDataCount)
	{
		// Clear the status bit							
		(startUpDataLocalV.fromPrimary) ? clrPrimaryMasterChg() : clrSecondaryMasterChg();
		// Build the response								
		szHartResp[respBufferSize] = 4;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;	
		// Now send back the configuration changed counter
		configCounter.i = startUpDataLocalNv.configChangeCount;
		szHartResp[respBufferSize] = configCounter.b[1];   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = configCounter.b[0];   // Device Status high byte
		++respBufferSize;							
	}
	else
	{
		configCounter.b[1] = szHartCmd[respBufferSize+1];
		configCounter.b[0] = szHartCmd[respBufferSize+2];
		// First check to see if we have too few bytes
		respCode = (CONFIG_COUNTER_SIZE > hartDataCount) ? TOO_FEW_DATA_BYTES : RESP_SUCCESS;
		// If we have enough bytes, make sure the poll address is valid
		if (!respCode)
		{
			respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
		}
		// Now check to see if the config counter sent matches the local count
		if (!respCode)
		{
			respCode = (configCounter.i != startUpDataLocalNv.configChangeCount) ? CONFIG_COUNTER_MISMATCH : RESP_SUCCESS;
		}
		// If we do not have a successful response code, send back a short reply, do
		// not execute the command
		if (respCode)
		{
			common_tx_error(respCode);
		}
		else
		{
			// execute the command
			(startUpDataLocalV.fromPrimary) ? clrPrimaryMasterChg() : clrSecondaryMasterChg();
			// Build the response
			szHartResp[respBufferSize] = CONFIG_COUNTER_SIZE+2;  // Byte count
			++respBufferSize;							
			szHartResp[respBufferSize] = 0;   // Device Status high byte
			++respBufferSize;							
			szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
				startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
			++respBufferSize;	
			// Now send back the configuration changed counter
			configCounter.i = startUpDataLocalNv.configChangeCount;
			szHartResp[respBufferSize] = configCounter.b[1];   // Device Status high byte
			++respBufferSize;							
			szHartResp[respBufferSize] = configCounter.b[0];   // Device Status high byte
			++respBufferSize;							
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_39()
//
// Description:
//
// Process the HART command 39 : EPROM Control Burn/Restore (Deprecated)
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void common_cmd_39(void)
{
	unsigned char burnCommand = szHartCmd[respBufferSize+1];
	// First check to see if we have too few bytes
	unsigned char respCode = (1 > hartDataCount) ? TOO_FEW_DATA_BYTES : RESP_SUCCESS;
	// If we have enough bytes, make sure the poll address is valid
	if (!respCode)
	{
		respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	}
	// if there is no response code, set up to execute the command
	if (!respCode)
	{
		// If the command data is 0, sync to RAM
		if (0 == burnCommand)
		{
			// Burn RAM -> FLASH
			cmdSyncToFlash = TRUE;
		}
		// if it is 1, sync to flash
		else if (1 == burnCommand)
		{
			// Refresh FLASH -> RAM
			cmdSyncToRam = TRUE;
		}
		else
		{
			// Invalid selection, do nothing
			respCode = INVALID_SELECTION;
		}	
	}
	// If we have a non-zero code, send back the error return
	if (respCode)
	{
			common_tx_error(respCode);
	}
	else // We can execute the command from here
	{
		// Now build the response buffer
		szHartResp[respBufferSize] = 3;
		++respBufferSize;
		// Send status as usual
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;
		// Parrot back the command byte
		szHartResp[respBufferSize] = burnCommand;
		++respBufferSize;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_40()
//
// Description:
//
// Process the HART command 40 : Enter/Exit Fixed Current Mode
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
void common_cmd_40 (void)
{
	U_LONG_FLOAT cmdCurrent;
	// Are we busy?
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	// Is loop current signaling disabled?
	if (CURRENT_MODE_DISABLE == startUpDataLocalNv.currentMode)
	{
		respCode = LOOP_CURRENT_NOT_ACTIVE;
	}
	// Did we receive enough bytes?
	if (!respCode)
	{
		respCode = (4 > hartDataCount) ? TOO_FEW_DATA_BYTES : RESP_SUCCESS;
	}
	// Determine if the commanded value is too large or too small
	if (!respCode)
	{
		updateInProgress = TRUE;
		// decode the commanded current
		cmdCurrent.fVal = decodeBufferFloat(&(szHartCmd[respBufferSize+1]));
		// Save the current value of the loop for later, if needed
		if ((0.0 < cmdCurrent.fVal) && (LOOP_OPERATIONAL == loopMode))
		{
			savedLoopCurrent = ma4_20;
		}
		// Set the loop current
		respCode = setFixedCurrentMode(cmdCurrent.fVal);
	}
	if (respCode)
	{
		// clear the update in progress flag
		updateInProgress = FALSE; 
		updateRequestSent = FALSE;
		common_tx_error(respCode);
	}
	else
	{
		// Now store the command value so that the 9900 can be periodically
		// reminded if it is in fixed current state
		lastRequestedCurrentValue = cmdCurrent.fVal;
		if (0.0 == cmdCurrent.fVal)
		{
			clrPrimaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
			clrSecondaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
		}
		else
		{
			setPrimaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
			setSecondaryStatusBits(FD_STATUS_PV_ANALOG_FIXED);
		}
		// Now build the response
		szHartResp[respBufferSize] = 6;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;							
		// Copy in the requested loop current value or the saved value to the response and to
		// the reported loop variable
		if (0.0 == cmdCurrent.fVal)
		{
			copyFloatToRespBuf(savedLoopCurrent);
			reportingLoopCurrent = savedLoopCurrent;
		}
		else
		{
			copyFloatToRespBuf(cmdCurrent.fVal);
			reportingLoopCurrent = cmdCurrent.fVal;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_42()
//
// Description:
//
// Process the HART command 42 : Perform Device Reset
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
void common_cmd_42 (void)
{
	// Are we busy?
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	// if we are not busy, set the reset flag
	if (respCode)
	{
		common_tx_error(respCode);
	}
	else
	{
		cmdReset = TRUE;
		szHartResp[respBufferSize] = 2;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;							
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_45()
//
// Description:
//
// Process the HART command 45 : Trim Loop Current Zero
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
void common_cmd_45 (void)
{
	U_LONG_FLOAT cmdCurrent;
	// Are we busy?
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	// Is loop current signaling disabled?
	if (CURRENT_MODE_DISABLE == startUpDataLocalNv.currentMode)
	{
		respCode = LOOP_CURRENT_NOT_ACTIVE;
	}
	// Did we receive enough bytes?
	if (!respCode)
	{
		respCode = (4 > hartDataCount) ? TOO_FEW_DATA_BYTES : RESP_SUCCESS;
	}
	// Make sure the loop is set up correctly
	if ((!respCode) && (FALSE == setToMinValue))
	{
		respCode = INCORRECT_LOOP_MODE;
	}
	// Determine if the commanded value is too large or too small
	if (!respCode)
	{
		updateInProgress = TRUE;
		cmdCurrent.fVal = decodeBufferFloat(&(szHartCmd[respBufferSize+1]));
		respCode = trimLoopCurrentZero(cmdCurrent.fVal);
	}
	if (respCode)
	{
		updateInProgress = FALSE;
		updateRequestSent = FALSE;
		common_tx_error(respCode);
	}
	else
	{
		reportingLoopCurrent = cmdCurrent.fVal;
		// Now build the response
		szHartResp[respBufferSize] = 6;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;							
		// Copy in the requested loop current value for the response. The 9900 will catch up later
		copyFloatToRespBuf(cmdCurrent.fVal);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_46()
//
// Description:
//
// Process the HART command 46 : Trim Loop Current Gain
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
void common_cmd_46 (void)
{
	U_LONG_FLOAT cmdCurrent;
	// Are we busy?
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	// Is loop current signaling disabled?
	if (CURRENT_MODE_DISABLE == startUpDataLocalNv.currentMode)
	{
		respCode = LOOP_CURRENT_NOT_ACTIVE;
	}
	// Did we receive enough bytes?
	if (!respCode)
	{
		respCode = (4 > hartDataCount) ? TOO_FEW_DATA_BYTES : RESP_SUCCESS;
	}
	// Make sure the loop is set up correctly
	if ((!respCode) && (FALSE == setToMaxValue))
	{
		respCode = INCORRECT_LOOP_MODE;
	}
	// Try to execute the command if no errors so far
	if (!respCode)
	{
		updateInProgress = TRUE;
		cmdCurrent.fVal = decodeBufferFloat(&(szHartCmd[respBufferSize+1]));
		respCode = trimLoopCurrentGain(cmdCurrent.fVal);
	}
	if (respCode)
	{
		updateInProgress = FALSE;
		updateRequestSent = FALSE;
		common_tx_error(respCode);
	}
	else
	{
		// Set the reporting current value
		reportingLoopCurrent = cmdCurrent.fVal;
		// Now build the response
		szHartResp[respBufferSize] = 6;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;							
		// Copy in the requested loop current value for the response. The 9900 will catch up later
		copyFloatToRespBuf(cmdCurrent.fVal);
	}
}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_48()
//
// Description:
//
// Process the HART command 48 : Transmit Additional Status
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
void common_cmd_48(void)
{
	unsigned char match = TRUE;
	if ((0 < hartDataCount) && (9 > hartDataCount))
	{
		common_tx_error(TOO_FEW_DATA_BYTES);
		return;
	}
	// First, check to see if we have to clear the more status available bit.
	// This is only true if we have 9 or more request bytes
	if (9 <= hartDataCount)
	{
		// Were are only going to compare the first 9 bytes
		if (szHartCmd[respBufferSize+1] != startUpDataLocalV.DeviceSpecificStatus[0]) 
		{
			match = FALSE;
		}
		if (szHartCmd[respBufferSize+2] != startUpDataLocalV.DeviceSpecificStatus[1])
		{
			match = FALSE;
		}
		if (szHartCmd[respBufferSize+3] != startUpDataLocalV.DeviceSpecificStatus[2])
		{
			match = FALSE;
		}
		if (szHartCmd[respBufferSize+4] != startUpDataLocalV.DeviceSpecificStatus[3])
		{
			match = FALSE;
		}
		if (szHartCmd[respBufferSize+5] != startUpDataLocalV.DeviceSpecificStatus[4])
		{
			match = FALSE;
		}
		if (szHartCmd[respBufferSize+6] != startUpDataLocalV.DeviceSpecificStatus[5])
		{
			match = FALSE;
		}
		if (szHartCmd[respBufferSize+7] != startUpDataLocalV.extendFieldDevStatus)
		{
			match = FALSE;
		}
		if (szHartCmd[respBufferSize+8] != startUpDataLocalV.DeviceOpMode)
		{
			match = FALSE;
		}
		if (szHartCmd[respBufferSize+9] != startUpDataLocalV.StandardStatus0)
		{
			match = FALSE;
		}
		if (TRUE == match)
		{
			(startUpDataLocalV.fromPrimary) ? clrPrimaryMoreAvailable() : clrSecondaryMoreAvailable();
		}
	}
	// Build the response			
	szHartResp[respBufferSize] = 11;  // Byte count
	++respBufferSize;							
	szHartResp[respBufferSize] = 0;   // Device Status high byte
	++respBufferSize;							
	szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
		startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
	++respBufferSize;	
	// Device specific status
	memcpy(&szHartResp[respBufferSize], startUpDataLocalV.DeviceSpecificStatus, DEV_SPECIFIC_STATUS_SIZE);
	respBufferSize += DEV_SPECIFIC_STATUS_SIZE;
	// Extended device status
	szHartResp[respBufferSize] = startUpDataLocalV.extendFieldDevStatus;   
	++respBufferSize;							
	// Device operating mode
	szHartResp[respBufferSize] = startUpDataLocalV.DeviceOpMode;   
	++respBufferSize;							
	// standard status 0
	szHartResp[respBufferSize] = startUpDataLocalV.StandardStatus0;   
	++respBufferSize;							
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_54()
//
// Description:
//
// Process the HART command 54 : Read Device Variable Information
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
void common_cmd_54 (void)
{
	union 
	{
		unsigned long i;
		unsigned char b[4];
	} UpdateTime;
	
	UpdateTime.i = SENSOR_UPDATE_TIME;
	 
	float span = 0.0;
	float damping = 0.0;
	unsigned char respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	unsigned char requestedVariable = szHartCmd[respBufferSize+1];
	// Did we receive enough bytes?
	if (!respCode)
	{
		respCode = (1 > hartDataCount) ? TOO_FEW_DATA_BYTES : RESP_SUCCESS;
	}
	// Now check to make sure the selection is valid
	if (!respCode)
	{
		switch (requestedVariable)
		{
		case DVC_PV:						
		case DVC_SV:						
		case DVC_PERCENT_RANGE:			
		case DVC_LOOP_CURRENT:		
		case DVC_PRIMARY_VARIABLE:		
		case DVC_SECONDARY_VARIABLE:
			// Do not change the response code
			break;	
		default:
			respCode = INVALID_SELECTION;
			break;
		}
	}
	if (respCode)
	{
		common_tx_error(respCode);
	}
	else
	{
		szHartResp[respBufferSize] = 29;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;	
		// Device Variable Code
		szHartResp[respBufferSize] = requestedVariable;   
		++respBufferSize;							
		// The transducer S/N is 0
		szHartResp[respBufferSize] = 0;   // Sensor S/N
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Sensor S/N
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Sensor S/N
		++respBufferSize;	
		switch(requestedVariable)
		{
		case DVC_SV:						
		case DVC_SECONDARY_VARIABLE:
			// Transducer units are the PV units
			szHartResp[respBufferSize] = (NOT_USED <= u9900Database.db.UnitsSecondaryVar) ? NOT_USED :
				u9900Database.db.UnitsSecondaryVar;   
			++respBufferSize;	
			break;
		case DVC_PV:						
		case DVC_PERCENT_RANGE:			
		case DVC_LOOP_CURRENT:		
		case DVC_PRIMARY_VARIABLE:	
		default:
			// Transducer units are the PV units
			szHartResp[respBufferSize] = u9900Database.db.UnitsPrimaryVar;   
			++respBufferSize;	
			break;
		}		
		// Now the High limit from the database				
		copyFloatToRespBuf(u9900Database.db.LOOP_SET_HIGH_LIMIT.floatVal);					
		// Now the low limit from the database				
		copyFloatToRespBuf(u9900Database.db.LOOP_SET_LOW_LIMIT.floatVal);					
		// the  damping is 0				
		copyFloatToRespBuf(damping);					
		// the minimum span is 0				
		copyFloatToRespBuf(span);	
		// device variable classification
		switch(requestedVariable)
		{
		case DVC_SV:						
		case DVC_SECONDARY_VARIABLE:
			// SV units are not classified
			szHartResp[respBufferSize] = 0;   
			++respBufferSize;	
			break;
		case DVC_PV:						
		case DVC_PERCENT_RANGE:			
		case DVC_LOOP_CURRENT:		
		case DVC_PRIMARY_VARIABLE:	
		default:
			// Transducer units are the PV units
			szHartResp[respBufferSize] = u9900Database.db.Hart_Dev_Var_Class;   
			++respBufferSize;	
			break;
		}		
		// device variable family
		szHartResp[respBufferSize] = NOT_USED;   
		++respBufferSize;							
		// Update time period			
		szHartResp[respBufferSize] = UpdateTime.b[3];   
		++respBufferSize;							
		szHartResp[respBufferSize] = UpdateTime.b[2];   
		++respBufferSize;							
		szHartResp[respBufferSize] = UpdateTime.b[1];   
		++respBufferSize;							
		szHartResp[respBufferSize] = UpdateTime.b[0];   
		++respBufferSize;							
	}
}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_57()
//
// Description:
//
// Process the HART command 57 : Read Unit Tag, Descriptor, and Date (Deprecated)
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
void common_cmd_57 (void)
{
	// Function is identical to command 13, so just use it
	common_cmd_13();
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_58()
//
// Description:
//
// Process the HART command 58 : Write Unit Tag, Descriptor, and Date (Deprecated)
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
void common_cmd_58 (void)
{
	// the code is identical to command 18, so just use it
	common_cmd_18();
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: common_cmd_110()
//
// Description:
//
// Process the HART command 110 : Read All Dynamic Variables (Deprecated)
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
void common_cmd_110 (void)
{
	//unsigned char respCode = (updateDelay) ? UPDATE_FAILURE : RESP_SUCCESS;
	
	szHartResp[respBufferSize] = 12;  // Byte count
	++respBufferSize;							
	szHartResp[respBufferSize] = 0;   // Device Status high byte
	++respBufferSize;							
	szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
		startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
	++respBufferSize;	
	// Now the PV
	szHartResp[respBufferSize] = u9900Database.db.UnitsPrimaryVar;
	++respBufferSize;
	copyFloatToRespBuf(PVvalue);					
	// Now the SV
	szHartResp[respBufferSize] = u9900Database.db.UnitsSecondaryVar;
	++respBufferSize;
	copyFloatToRespBuf(SVvalue);					
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: flow_tx_error()
//
// Description:
//
// Respond to the master with a response code
//
// Parameters: unsigned char: response code
//
// Return Type: void
//
// Implementation notes:
//
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void common_tx_error(unsigned char respCode)
{
	szHartResp[respBufferSize] = 2; // Byte count
	++respBufferSize;
	szHartResp[respBufferSize] = respCode; // response code
	++respBufferSize;
	szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
		startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status; // primary or secondary status
	++respBufferSize;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: flow_tx_comm_error()
//
// Description:
//
// Responds to the Master with a communications error code
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
void common_tx_comm_error(void)
{
	
	szHartResp[respBufferSize] = 2;  // Byte count
	++respBufferSize;							
	szHartResp[respBufferSize] = COMM_ERROR;  // Communication Error
	if (HartErrRegister & BUFFER_OVERFLOW)
	{
		szHartResp[respBufferSize] |= BOVF_ERROR;  // Buffer Overflow Error
		++respBufferSize;							
	}
	if (HartErrRegister & RCV_BAD_LRC)
	{
		szHartResp[respBufferSize] |= LPAR_ERROR;  // LRC Error
		++respBufferSize;							
	}
	if (HartErrRegister & RCV_PARITY_ERROR)
	{
		szHartResp[respBufferSize] |= VPAR_ERROR;  // Parity Error
		++respBufferSize;							
	}							
	if (HartErrRegister & RCV_FRAMING_ERROR)
	{
		szHartResp[respBufferSize] |= FRAM_ERROR;  // Framing Error
		++respBufferSize;							
	}							
	++respBufferSize;							
	szHartResp[respBufferSize] = 0;  // status byte
	++respBufferSize;							
}



///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: mfr_cmd_219()
//
// Description:
//
// Process the HART command 219 : Write device ID
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//		This MFR-specific command is used to set the device ID. It is not 
//		to be published in the 
//
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void mfr_cmd_219(void)
{
	// First check to see if we have too few bytes
	unsigned char respCode = (FINAL_ASSY_SIZE > hartDataCount) ? TOO_FEW_DATA_BYTES : RESP_SUCCESS;
	// If we have enough bytes, make sure the poll address is valid
	if (!respCode)
	{
		respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	}
	// If we have a non-zero code, send back the error return
	if (respCode)
	{
		common_tx_error(respCode);
	}
	else // We can execute the command from here
	{
		// Set the change flags	
		setPrimaryMasterChg();
		setSecondaryMasterChg();
		incrementConfigCount();
		// Copy the new Device ID to FLASH
		copyDeviceIdToFlash(&(szHartCmd[respBufferSize+1]));
		// Copy the final assembly into the database
		memcpy(&startUpDataLocalNv.DeviceID, &(szHartCmd[respBufferSize+1]), DEVICE_ID_SIZE);	
		// Set up to write RAM to FLASH
		updateNvRam = TRUE;					
		// Build the response
		szHartResp[respBufferSize] = DEVICE_ID_SIZE+2;  // Byte count
		++respBufferSize;							
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;	
		// Parrot back the final assy
		memcpy(&(szHartResp[respBufferSize]), &startUpDataLocalNv.DeviceID, DEVICE_ID_SIZE);
		respBufferSize += DEVICE_ID_SIZE;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: mfr_cmd_220()
//
// Description:
//
// Process the HART command 220 : Retrieve Error Counters
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//		This MFR-specific command is used to retrieve the error counters. It is not 
//		to be published in the DD
//
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void mfr_cmd_220(void)
{
	// First check to see if we have too few bytes
	unsigned char respCode;
	
	respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	// If we have a non-zero code, send back the error return
	if (respCode)
	{
		common_tx_error(respCode);
	}
	else // We can execute the command from here
	{
		// Build the response
		szHartResp[respBufferSize] = 2 + 24 + 38;  // Byte count
		++respBufferSize;					
		// RC & Status		
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;	
		// Copy the number of messages ready to process
		copyLongToRespBuf(numMsgReadyToProcess);
		// Copy the number of messages ready to process
		copyLongToRespBuf(numMsgProcessed);
		// Copy the number of messages ready to process
		copyLongToRespBuf(numMsgUnableToProcess);
		// Copy the number of messages ready to process
		copyLongToRespBuf(xmtMsgCounter);
		// Copy the number of messages ready to process
		copyLongToRespBuf(errMsgCounter);
		// Copy the number of flash writes
		copyLongToRespBuf(flashWriteCount);
		// Now copy in the error counters from the startup data structure
		copyIntToRespBuf(startUpDataLocalV.errorCounter[0]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[1]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[2]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[3]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[4]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[5]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[6]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[7]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[8]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[9]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[10]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[11]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[12]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[13]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[14]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[15]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[16]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[17]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[18]);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: mfr_cmd_221()
//
// Description:
//
// Process the HART command 221 : Reset Error Counters
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//		This MFR-specific command is used to reset the error counters. It is not 
//		to be published in the DD
//
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void mfr_cmd_221(void)
{
	// First check to see if we have too few bytes
	unsigned char respCode;
	int index;
	
	respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	// If we have a non-zero code, send back the error return
	if (respCode)
	{
		common_tx_error(respCode);
	}
	else // We can execute the command from here
	{
		// Reset All the long counters to 0
		numMsgReadyToProcess = 0;
		numMsgProcessed = 0;
		numMsgUnableToProcess = 0;
		xmtMsgCounter = 0;
		errMsgCounter = 0;
		flashWriteCount = 0;
		// reset the counters in the startup data to 0
		for (index = 0; index < 19; ++index)
		{
			startUpDataLocalV.errorCounter[index] = 0;
		}
		// Now signal the fact the NVRAM haas to change
		//cmdSyncToFlash = TRUE;
		// Build the response
		szHartResp[respBufferSize] = 2 + 24 + 38;  // Byte count
		++respBufferSize;					
		// RC & Status		
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;	
		// Copy the number of messages ready to process
		copyLongToRespBuf(numMsgReadyToProcess);
		// Copy the number of messages ready to process
		copyLongToRespBuf(numMsgProcessed);
		// Copy the number of messages ready to process
		copyLongToRespBuf(numMsgUnableToProcess);
		// Copy the number of messages ready to process
		copyLongToRespBuf(xmtMsgCounter);
		// Copy the number of messages ready to process
		copyLongToRespBuf(errMsgCounter);
		// Copy the number of flash writes
		copyLongToRespBuf(flashWriteCount);
		// Now copy in the error counters from the startup data structure
		copyIntToRespBuf(startUpDataLocalV.errorCounter[0]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[1]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[2]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[3]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[4]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[5]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[6]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[7]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[8]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[9]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[10]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[11]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[12]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[13]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[14]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[15]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[16]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[17]);
		copyIntToRespBuf(startUpDataLocalV.errorCounter[18]);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: mfr_cmd_222()
//
// Description:
//
// Process the HART command 222 : Dump Nonvolatile memory
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//		This MFR-specific command is used to dump the NV memory. It is not 
//		to be published in the DD
//
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void mfr_cmd_222(void)
{
	// First check to see if we have too few bytes
	unsigned char respCode;
	unsigned char index;
	//unsigned char * pNvMem = VALID_SEGMENT_1;
	unsigned char * pNvMem = (unsigned char *)&startUpDataLocalNv;
	
	respCode = (deviceBusyFlag) ? HART_DEVICE_BUSY : RESP_SUCCESS;
	// If we have a non-zero code, send back the error return
	if (respCode)
	{
		common_tx_error(respCode);
	}
	else // We can execute the command from here
	{
		// Build the response
		szHartResp[respBufferSize] = 2 + sizeof(HART_STARTUP_DATA_NONVOLATILE) + 1;  // Byte count
		++respBufferSize;					
		// RC & Status		
		szHartResp[respBufferSize] = 0;   // Device Status high byte
		++respBufferSize;							
		szHartResp[respBufferSize] = (startUpDataLocalV.fromPrimary) ? 
			startUpDataLocalNv.Primary_status : startUpDataLocalNv.Secondary_status;  // Status byte
		++respBufferSize;
		// Now copy out the NV Flash byte by byte
		for (index = 0; index < sizeof(HART_STARTUP_DATA_NONVOLATILE); ++index)
		{
			szHartResp[respBufferSize] = *(pNvMem + index);   // the NV memory
			++respBufferSize;							
		}	
		// Send back the key from the current setting logic
		szHartResp[respBufferSize] = currentMsgSent;   // last message sent
		++respBufferSize;							
		
	}
}



///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: CalculatePercentRange()
//
// Description:
//
// Utility to calculate the passed value as a percent of range
//
// Parameters: float - the current loop value
//
// Return Type: float - percent of range
//
// Implementation notes:
//
//
/////////////////////////////////////////////////////////////////////////////////////////// 
float CalculatePercentRange(float upper, float lower, float value)
{
	float percentRange = ((value - lower)/(upper - lower)) * 100.0;
	return percentRange;
}







