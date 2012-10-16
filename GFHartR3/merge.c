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
#include "main9900.h"

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
// The startup data
HART_STARTUP_DATA_NONVOLATILE startUpDataLocalNv;
HART_STARTUP_DATA_VOLATILE startUpDataLocalV;
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
