/*
 *  \file   merge.h
 *  \brief  SOlve the merge conflict with original sw
 *  Created on: Oct 16, 2012
 *      Author: Marco.HenryGin
 */

#ifndef MERGE_H_
#define MERGE_H_
//==============================================================================
//  $INCLUDES
//==============================================================================


//==============================================================================
//  $DEFINES
//==============================================================================
// Defines to solve Conflicted protocols.x
//  TODO: Merge and resolve
#define HART_STAT UCA1STAT

// Address field masks
#define BURST_MODE_BIT      0x40        // Burst mode bit
#define PRIMARY_MASTER      0x80
#define POLL_ADDR_MASK      0x3F
#define EXT_DEV_TYPE_ADDR_MASK  0x3FFF

typedef struct stHartStartupV   // HART 7 compliant
{
    // The first member must be an unsigned char
    unsigned char myName [NAME_SIZE];
    unsigned char defaultSensorType;
    int fromPrimary;
    //unsigned char Primary_status;
    //unsigned char Secondary_status;
    // The following members are used for command 0
    unsigned int expandedDevType;
    unsigned char minPreamblesM2S;
    unsigned char majorHartRevision;
    unsigned char deviceRev;
    unsigned char swRev;
    unsigned char hwRev;   // 5 bits only
    unsigned char physSignalCode;  // 3 bits only
    unsigned char flags;
    //unsigned char DeviceID [DEVICE_ID_SIZE];
    unsigned char minPreamblesS2M;
    unsigned char maxNumDevVars;
    //unsigned int configChangeCount;  // Must be changed every revision!!
    unsigned char extendFieldDevStatus;
    //unsigned int ManufacturerIdCode;
    unsigned int PrivDistCode;
    unsigned char devProfile;
    ////////////////////////////////////////////
    //unsigned char LongTag [LONG_TAG_SIZE];
    //unsigned char TagName [SHORT_TAG_SIZE];
    //unsigned char Descriptor [DESCRIPTOR_SIZE];
    //unsigned char Date[DATE_SIZE];
    //unsigned char PollingAddress;
    //unsigned char FinalAssy [FINAL_ASSY_SIZE];
    ////////////////////////////////////////////
    //unsigned char HARTmsg [HART_MSG_SIZE];
    //unsigned char AlertStatus;
    //unsigned char ErrStatus;
    unsigned char DeviceSpecificStatus [DEV_SPECIFIC_STATUS_SIZE];
    unsigned char DeviceOpMode;
    unsigned char StandardStatus0;
    //unsigned char currentMode;
    /////////////////////////////////////////////
    unsigned int errorCounter[19];
} HART_STARTUP_DATA_VOLATILE;

// Error Counter Indices:
// Index 0 = the number of FRAMING errors
// Index 1 = the number of PARITY errors
// Index 2 = the number of OVERFLOW errors
// Index 3 = the number of insufficient preambles
// Index 4 = the number of excess preambles
// Index 5 = the number of STX errors
// Index 6 = the number of bad LRC
// Index 7 = the number of bad byte errors
// Index 8 = the number of cold starts
// Index 9 = the number of commanded resets
// Index 10 = the number of UNMI interrupts
// Index 11 = the number of SYSNMI interrupts
// Index 12 = the number of WDT interrupts
// Index 13 = the number of bad addresses
// Index 14 = the number of SOM overflows
// Index 15 = the number of not my addresses
// Index 16 = the number of errored byte counts
// Index 17 = the number of received characters exceed the buffer
// Index 18 = the number of fatal SOM characters

typedef struct stHartStartupNV   // HART 7 compliant
{
    // The first member must be an unsigned char
    //unsigned char myName [NAME_SIZE];
    //unsigned char defaultSensorType;
    //int fromPrimary;
    unsigned char Primary_status;
    unsigned char Secondary_status;
    // The following members are used for command 0
    //unsigned int expandedDevType;
    //unsigned char minPreamblesM2S;
    //unsigned char majorHartRevision;
    //unsigned char deviceRev;
    //unsigned char swRev;
    //unsigned char hwRev;   // 5 bits only
    //unsigned char physSignalCode;  // 3 bits only
    //unsigned char flags;
    unsigned char DeviceID [DEVICE_ID_SIZE];
    //unsigned char minPreamblesS2M;
    //unsigned char maxNumDevVars;
    unsigned int configChangeCount;  // Must be changed every revision!!
    //unsigned char extendFieldDevStatus;
    unsigned int ManufacturerIdCode;
    //unsigned int PrivDistCode;
    //unsigned char devProfile;
    ////////////////////////////////////////////
    unsigned char LongTag [LONG_TAG_SIZE];
    unsigned char TagName [SHORT_TAG_SIZE];
    unsigned char Descriptor [DESCRIPTOR_SIZE];
    unsigned char Date[DATE_SIZE];
    unsigned char PollingAddress;
    unsigned char FinalAssy [FINAL_ASSY_SIZE];
    ////////////////////////////////////////////
    unsigned char HARTmsg [HART_MSG_SIZE];
    //unsigned char AlertStatus;
    //unsigned char ErrStatus;
    //unsigned char DeviceSpecificStatus [DEV_SPECIFIC_STATUS_SIZE];
    //unsigned char DeviceOpMode;
    //unsigned char StandardStatus0;
    unsigned char currentMode;
    /////////////////////////////////////////////
    //unsigned int errorCounter[19];
} HART_STARTUP_DATA_NONVOLATILE;

//==============================================================================
//  $GLOBAL PROTOTYPES
//==============================================================================
void copy9900factoryDb(void);
//==============================================================================
//
//  $GLOBAL VARIABLES
//
//==============================================================================
extern int hostActive;

extern HART_STARTUP_DATA_NONVOLATILE startUpDataLocalNv;
extern const HART_STARTUP_DATA_NONVOLATILE startUpDataFactoryNv;
extern HART_STARTUP_DATA_VOLATILE startUpDataLocalV;
extern const HART_STARTUP_DATA_VOLATILE startUpDataFactoryV;



//==============================================================================
//
//  $INLINE FUNCTIONS
//==============================================================================

#endif /* MERGE_H_ */
