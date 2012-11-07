///////////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) AB Tech Solution LLC 2011 All Rights Reserved. 
//
// This code may not be copied without the express written consent of 
// AB Tech Solution LLC.
//
//
// Client: GF
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY 
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
///////////////////////////////////////////////////////////////////////////////////////////
//
// Module Name:  utilities.c
//
// Functional Unit: 
//
// Description:
//
// This module provides utility functions for the flash memory, and other 
// miscellaneous functions.
//
// Exported Interfaces:
//
// utilities.h - This interface file includes all exported interfaces
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
// 04/04/11  0    Vijay Soni    Creation
//
// 04/07/11  0    Patel 	Add funcations
//
//
///////////////////////////////////////////////////////////////////////////////////////////
 
#include <msp430f5528.h>
#include "define.h"
#include "hardware.h"
#include "utilitiesr3.h"

unsigned int flashWriteCount;

// Flash programming utilities. Shut off all interrupts and the 
// watchdog before calling any flash programming function to prevent
// undesirable results

// For now, only 4 segments (2048 bytes) are allocated for these operations. The 
// segments are 2-5 (addresses  F800 - FBFF)

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: copyMainFlashToMem()
//
// Description:
//
// Copies the contents of a segment to main (RAM) memory. The pointer MUST point to a valid
// main flash start address, and segSize MUST be an integral multiple of 512. The memory
// reserved for the copy must be big enough to take the number of 512 byte segments
// being copied. Returns TRUE if everything was OK, FALSE otherwise
//
// Parameters:
//
//     unsigned char * flashPtr: pointer to the start of flash to be copied.
//     unsigned char * memPtr: pointer to the start of RAM to be copied to.
//     int segSize: the number of bytes (in multiples of 512) to be copied
//
// Return Type: int.
//
// Implementation notes:
//
// Checks to make sure the parameters are valid, then copies the contents of the flash
// to the RAM memory.
//
/////////////////////////////////////////////////////////////////////////////////////////// 
 
int copyMainFlashToMem (unsigned char * flashPtr, unsigned char * memPtr, int segSize)
{
	int okToCopy = FALSE;
	int success = FALSE;
	int iIdx;
	if ((MAIN_SEGMENT_SIZE > segSize) || ((4 * MAIN_SEGMENT_SIZE) < segSize) 
		|| (0 != (segSize % MAIN_SEGMENT_SIZE)))
	{
		return success; // just bail now
	}
	// Verify we have a valid segment pointer & that segSize doesn't overrun
	if ((VALID_SEGMENT_1 == flashPtr) && 
		((4 * MAIN_SEGMENT_SIZE) >= segSize))
	{
		okToCopy = TRUE;
	}
	else if ((VALID_SEGMENT_2 == flashPtr) 
		&& ((3 * MAIN_SEGMENT_SIZE) >= segSize))
	{
		okToCopy = TRUE;
	}
	else if ((VALID_SEGMENT_3 == flashPtr) 
		&& ((2 * MAIN_SEGMENT_SIZE) >= segSize))
	{
		okToCopy = TRUE;
	}
	else if ((VALID_SEGMENT_4 == flashPtr) 
		&& ((1 * MAIN_SEGMENT_SIZE) >= segSize))
	{
		okToCopy = TRUE;
	}
	if (TRUE == okToCopy)
	{
		stopWatchdog();
		// If we're here, it's a simple copy operation
		for (iIdx = 0; iIdx < segSize; ++iIdx)
		{
			*(memPtr + iIdx) = *(flashPtr + iIdx);
		}
		success = TRUE;
		resetWatchdog();;
	}
	return success; 
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: copyMemToMainFlash()
//
// Description:
//
// Copies memory contents to previously erased flash. It takes a 
// pointer to the starting location in flash (does not need to be on a segment
// boundary), a pointer to the memory to copy from, and the size of the memory
// to copy. The flashPtr & memSize parms are checked to verify all the written
// locations are in the 2K flash block. Returns TRUE if everything worked OK,
// FALSE otherwise. Performs a byte write. 
//
// Parameters:
//
//     unsigned char * flashPtr: pointer to the start of flash to be copied to.
//     unsigned char * memPtr: pointer to the start of RAM to be copied.
//     int memSize: the number of bytes to be copied
//
// Return Type: int.
//
// Implementation notes:
//
// memSize does not have to be a multiple of 512 bytes.
// NOTE: This function cannot be used to program VALID_SEGMENT_4, which has a special LOCK
//       bit, LOCKA!!
//
/////////////////////////////////////////////////////////////////////////////////////////// 
 
int  copyMemToMainFlash (unsigned char * flashPtr, unsigned char * memPtr, int memSize)
{
	int success = FALSE;
	int iIdx;
	unsigned char value = 0x55;
	// If we're out of bounds, bail early
	if (((unsigned char *)VALID_SEGMENT_1 > flashPtr) ||
		(flashPtr + memSize) > INFO_MEMORY_END)
	{
		return success;
	}
	stopWatchdog();
 	_disable_interrupts();		
	// If we're here, the request is valid.
  	FCTL3 = FWKEY;                  // Clear Lock bit
	// set up to write
	FCTL1 = FWKEY | WRT;
	for (iIdx = 0; iIdx < memSize; ++iIdx)
	{
		value = *(memPtr + iIdx);
		*(flashPtr + iIdx) = value;
		// Wait for the previous write to complete
		while (!(FCTL3 & WAIT))
		{
			__no_operation();
		}
#if 0
		// Timing - necessary??
		for (i = 0; i < 100; ++i)
		{
			__no_operation();
		}
#endif
	}
	FCTL1 = FWKEY;			// Turn off the WRT bit
 	FCTL3 = FWKEY | LOCK;   // Set Lock bit
 	success = TRUE;
	resetWatchdog();;
	_enable_interrupts();	
	return success;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: eraseMainSegment()
//
// Description:
//
// Erase individual main flash segments. Takes a pointer to the first segment to
// erase and the total size in bytes to erase. The number of bytes to erase
// MUST be and integral of the segment size, and the flash to be
// erased MUST be in the valid area. All flash contents in the erased area will
// be lost.
//
// Parameters:
//
//     unsigned char * flashPtr: pointer to the start of flash to be erased.
//     int segSize: the number of bytes (in multiples of 128) to be erased
//
// Return Type: int.
//
// Implementation notes:
//
// Checks to make sure the parameters are valid, then erases the contents of the flash.
// NOTE: This function cannot be used to erase VALID_SEGMENT_4, which has a special LOCK
//       bit, LOCKA!!
//
/////////////////////////////////////////////////////////////////////////////////////////// 
 
int eraseMainSegment(unsigned char * flashPtr, int segSize)
{
	int okToErase = FALSE;
	int success = FALSE;
	int numSegsErased;
	int numSegmentsToErase = segSize / MAIN_SEGMENT_SIZE;
	if ((MAIN_SEGMENT_SIZE > segSize) || ((3 * MAIN_SEGMENT_SIZE) < segSize) 
		|| (0 != (segSize % MAIN_SEGMENT_SIZE)))
	{
		return success; // just bail now
	}
	// Verify we have a valid segment pointer & that segSize doesn't overrun
	if ((VALID_SEGMENT_1 == flashPtr) && 
		((3 * MAIN_SEGMENT_SIZE) >= segSize))
	{
		okToErase = TRUE;
	}
	else if ((VALID_SEGMENT_2 == flashPtr) 
		&& ((2 * MAIN_SEGMENT_SIZE) >= segSize))
	{
		okToErase = TRUE;
	}
	else if ((VALID_SEGMENT_3 == flashPtr) 
		&& ((1 * MAIN_SEGMENT_SIZE) >= segSize))
	{
		okToErase = TRUE;
	}
	if (TRUE == okToErase)
	{
 		_disable_interrupts();		// erase all segments
 		stopWatchdog();
		for (numSegsErased = 0; numSegmentsToErase > numSegsErased; ++numSegsErased, flashPtr+=MAIN_SEGMENT_SIZE)
		{
  			FCTL3 = FWKEY;                  // Clear Lock bit
  			FCTL1 = FWKEY | ERASE;          // Set Erase bit, don't allow interrupts
  			*flashPtr = 0;                  // Dummy write to erase Flash seg
  			// Wait until the BUSY bit clears
  			while (FCTL3 & BUSY)
			{
				__no_operation();
			}
			success = TRUE;
		}
	}
	FCTL3 = FWKEY | LOCK;   // Set the lock bit
	resetWatchdog();;
 	_enable_interrupts();	
	return success;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: verifyFlashContents()
//
// Description:
//
// Verifies that the Flash and RAM memory contents are identical in the 
// specified range. Returns TRUE if everything is identical, FALSE otherwise
//
// Parameters:
//
//     unsigned char * flashPtr: pointer to the start of flash to be checked.
//     unsigned char * memPtr: pointer to the start of RAM to be checked.
//     int segSize: the number of bytes to be copied
//
// Return Type: int.
//
// Implementation notes:
//
// The check size does not need to be a multiple of the segnment size.
//
/////////////////////////////////////////////////////////////////////////////////////////// 
 
int verifyFlashContents(unsigned char * flashPtr, unsigned char * memPtr, int memSize)
{
	int ok = TRUE;
	int numBytesCompared = 0;
	stopWatchdog();
	while ((TRUE == ok) && (numBytesCompared < memSize))
	{
		if (*(flashPtr + numBytesCompared) == *(memPtr + numBytesCompared))
		{
			++numBytesCompared;
		}
		else
		{
			ok = FALSE;
		}
	}
	resetWatchdog();;
	return ok;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: syncToRam()
//
// Description:
//
// Copies the contents from FLASH to main (RAM) memory. 
//
// Parameters:
//
//     unsigned char * flashPtr: pointer to the start of flash to be copied.
//     unsigned char * memPtr: pointer to the start of RAM to be copied to.
//     int segSize: the number of bytes to be copied
//
// Return Type: void.
//
// Implementation notes:
//
// Checks to make sure the parameters are valid, then copies the contents of the flash
// to the RAM memory.
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void syncToRam(unsigned char * flashPtr, unsigned char * memPtr, int memSize)
{
	int idx;
	
	for (idx = 0; idx < memSize; ++idx)
	{
		*(memPtr+idx) = *(flashPtr+idx);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: syncToRam()
//
// Description:
//
// Copies the contents from RAM to FLASH memory. 
//
// Parameters:
//
//     unsigned char * flashPtr: pointer to the start of flash to be copied to.
//     unsigned char * memPtr: pointer to the start of RAM to be copied from.
//     int segSize: the number of bytes to be copied
//
// Return Type: int - TRUE if the memory is in sync, FALSE otherwise.
//
// Implementation notes:
//
// Checks to make sure the parameters are valid, then copies the contents of the flash
// to the FLASH memory. FLASH segments are erased first
//
/////////////////////////////////////////////////////////////////////////////////////////// 
int syncToFlash(unsigned char * flashPtr, unsigned char * memPtr, int memSize)
{
	// First see if there is a need to change by verifying the memory values
	int success = verifyFlashContents(flashPtr, memPtr, memSize);
	// If the verify worked, just reurn success now
	if (success)
	{
		return success;
	}
	// Calculate the number of segments to erase
	int numSegsToErase = calcNumSegments(memSize);
	// erase the flash
	eraseMainSegment(flashPtr, (numSegsToErase*MAIN_SEGMENT_SIZE));
	// copy the RAM back in to FLASH
	copyMemToMainFlash(flashPtr, memPtr, memSize);
	// Verify that it worked (or not)
	success = verifyFlashContents(flashPtr, memPtr, memSize);
	// Reset the write timer to 0 so it starts counting up
	//flashWriteTimer = 0;
	return success;
}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: IntToFloat()
//
// Description:
//
// Converts an integer value to the IEEE754 32-bit floating-point equivalent
//
// Parameters:
//
//     int iValue: the number to be converted.
//
// Return Type: float.
//
// Implementation notes:
//
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
 
float IntToFloat (int iValue)
{
	union
	{
		float fValue;
		double dValue;
	} rtnVal;
	
	// I need to make sure the precision is not lost here. later
	rtnVal.dValue = (double)iValue;
	rtnVal.dValue /= 100.;
	rtnVal.fValue = (float)rtnVal.dValue;
	return rtnVal.fValue;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: calcNumSegments()
//
// Description:
//
// Calculates the number of contiguous FLASH segments to erase based upon data size
//
// Parameters:
//
//     int memSize: the number of bytes to erase.
//
// Return Type: int - the number of segments to erase.
//
// Implementation notes:
//
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
int calcNumSegments (int memSize)
{
	int numSegs = memSize / MAIN_SEGMENT_SIZE;
	int remainder = memSize % MAIN_SEGMENT_SIZE;
	// if there is a remainder, increment the number of segments
	numSegs += (remainder) ? 1 : 0;
	return numSegs;
}

