/*
 * merge.c
 *  This is a provisional file to merge original project with new one
 *  ALl functions and variables have global scope
 *
 *  Created on: Oct 16, 2012
 *      Author: Marco.HenryGin
 */


//=======================
// INCLUDES
//=======================
#include "define.h"
#include "protocols.h"
#include "merge.h"
#include "utilitiesr3.h"
#include "main9900r3.h"
#include "string.h"
//========================
//  LOCAL DEFINES
//========================
//================================
//  LOCAL PROTOTYPES.
//================================
//========================
//  GLOBAL DATA
//========================
// Do we have a host actively communicating?
int hostActive = FALSE;


int rcvBroadcastAddr = FALSE;
// 9900 Database
U_DATABASE_9900 u9900Database;

//========================
//  LOCAL DATA
//========================
// 9900 Factory database
const DATABASE_9900 factory9900db =
{
  62,   // DB Length
  "4640500111", // serial number
  "3-9900-1X ", // Model string
  "10-04a", // SW REV
  0.0,    // LOOP_SET_LOW_LIMIT
  15.0,   // LOOP_SET_HIGH_LIMIT
  0.0,    // LOOP_SETPOINT_4MA
  14.0,   // LOOP_SETPOINT_20MA
  4.0,    // LOOP_ADJ_4MA
  20.0,   // LOOP_ADJ_20MA
  1,  // LOOP_ERROR_VAL
  0,  // LOOP_MODE;
  2,  // MEASUREMENT_TYPE;
  'A',  // GF9900_MS_PARAMETER_REVISION;
  'Q',  // Hart_Dev_Var_Class;
  0x3b, // UnitsPrimaryVar;
  0x20, // UnitsSecondaryVar;
  0,  // Pad;  // Just to be even
  0x0972  // checksum;
};


//==============================================================================
// FUNCTIONS
//==============================================================================


///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: initializeLocalData()
//
// Description:
//
// Utility to initialize the local data structure for the first time, or if
// the NV memory is ever corrupted.
//
// Parameters: void
//
// Return Type: void.
//
// Implementation notes:
//    If the NV memory is corrupt, copy the factory structure into
//    ram, then copy the RAM to the FLASH
//
//
///////////////////////////////////////////////////////////////////////////////////////////
void initializeLocalData (void)
{
  int numSegsToErase;
  // Clear the local structure
  memset(&startUpDataLocalNv, 0, sizeof(HART_STARTUP_DATA_NONVOLATILE));
  // Now copy the factory image into RAM
  memcpy(&startUpDataLocalNv, &startUpDataFactoryNv, sizeof(HART_STARTUP_DATA_NONVOLATILE));
  // Now copy in the NV unique device ID
  copyNvDeviceIdToRam();
  // erase the segment of FLASH so it can be reprogrammed
  numSegsToErase = calcNumSegments (sizeof(HART_STARTUP_DATA_NONVOLATILE));
  eraseMainSegment(VALID_SEGMENT_1, (numSegsToErase*MAIN_SEGMENT_SIZE));
  // Copy the local data into NV memory
  copyMemToMainFlash (VALID_SEGMENT_1, ((unsigned char *)&startUpDataLocalNv),
    sizeof(HART_STARTUP_DATA_NONVOLATILE));
}


/*!
 *  initStartUpData()
 *  Wrapper to initialize all data structures - interfaces with main
 *
 */
void initStartUpData()
{
  // Load up the volatile startup data
  // Clear the local structure
  memset(&startUpDataLocalV, 0, sizeof(HART_STARTUP_DATA_VOLATILE));
  // Now copy the factory image into RAM
  memcpy(&startUpDataLocalV, &startUpDataFactoryV, sizeof(HART_STARTUP_DATA_VOLATILE));
  // Load up the nonvolatile startup data
  // Load the startup data from NV memory
  syncToRam(VALID_SEGMENT_1, ((unsigned char *)&startUpDataLocalNv), sizeof(HART_STARTUP_DATA_NONVOLATILE));
  // If the local data structure has bad values, initialize them
  if (GF_MFR_ID != startUpDataLocalNv.ManufacturerIdCode)
  {
    initializeLocalData();
  }
  else
  {
    // Make sure we have the correct Device ID in any case
    verifyDeviceId();
  }
  // Set the COLD START bit for primary & secondary
  setPrimaryStatusBits(FD_STATUS_COLD_START);
  setSecondaryStatusBits(FD_STATUS_COLD_START);
  ++startUpDataLocalV.errorCounter[8];
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
