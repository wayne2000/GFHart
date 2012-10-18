#ifndef HART_H_
#define HART_H_
///////////////////////////////////////////////////////////////////////////////////////////
//
// Header File Name:  hartr3.h
//
// Description:
//
// This module provides HART interface. Functions are implemented in 
// the file, hart.c
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

/*************************************************************************
  *   $INCLUDES
*************************************************************************/
/*************************************************************************
  *   $DEFINES
*************************************************************************/
// Defines for certain Common Practice Commands
//#define IMPLEMENT_RANGE_CMDS_35_36_37

// Initial fixed values of local data
#define GF_MFR_ID           0x6055
#define EXPANDED_DEV_TYPE   0xE1B3
#define HART_MAJ_REV        7
#define DEVICE_REVISION     1
#define SOFTWARE_REVISION   1
#define HARDWARE_REVISION   1
#define SIGNAL_CODE         0
#define FLAGS               0x03
#define MAX_DEV_VARS        1 // Changed from 4 -> 1 per Sean Vincent. Only PV & SV
#define CURRENT_CONFIG_CNT  0
#define EXT_FLD_STATUS      0
#define DEVICE_PROFILE      1

// HART Error Register Flags
#define NO_HART_ERRORS          0x0000
#define RCV_PARITY_ERROR        0x0001
#define RCV_FRAMING_ERROR       0x0002
#define RCV_OVERRUN_ERROR       0x0004
#define RCV_BREAK_CHAR          0x0008
#define EXCESS_PREAMBLE         0x0010
#define INSUFFICIENT_PREAMBLE   0x0020
#define WRONG_ADDR              0x0040
#define UNSUPPORTED_CMD         0x0080
#define HART_TIMER_EXPIRED      0x0100
#define STX_ERROR               0x0200
#define RCV_BAD_BYTE_COUNT      0x0400
#define RCV_BAD_LRC             0x0800
#define BUFFER_OVERFLOW         0x1000
#define GAP_TIMER_EXPIRED       0x2000
#define EXTRA_CHAR_RCVD         0x4000

// HART Comm response error codes
#define COMM_ERR        0x80
#define VERT_PAR_ERR    0x40
#define OVRRUN_ERR      0x20
#define FRM_ERR         0x10
#define LONG_PAR_ERR    0x08
#define BUFF_OVR_ERR    0x02

// Default Sensor types - from 9900
#define NOT_CONFIGURED      0x00
#define FLOW_TYPE           0x01
#define PH_TYPE             0x02
#define ORP_TYPE            0x03
#define CONDUCTIVITY_TYPE   0x04
#define PRESSURE_TYPE       0x05
#define LEVEL_TYPE          0x06
#define TEMPERATURE_TYPE    0x07
#define MA4_20_TYPE         0x08
#define SALINITY_TYPE       0x09

// Defines
#define RESP_SOF_OFFSET         0
#define LONG_DATA_OFFSET        8 //RESP_LONG_COUNT_OFFSET+1
#define SHORT_DATA_OFFSET       4 // RESP_SHORT_COUNT_OFFSET+1

// Error Codes
#define CMD_NOT_IMPLEMENTED 64  // Command not implemented
#define UNIT_BUSY           32  // The device is busy
#define SHORT_DATA          5   // not enough data received
#define SELECTION_INVALID   2   // The selection is invalid

// Other response codes
#define NO_WRITE_PROTECT    251 // Write protect mode not available
#define NOT_USED            250

// Alarm response codes for command 15
#define ALARM_CODE_LOOP_HIGH    0
#define ALARM_CODE_LOOP_LOW     1
#define ALARM_CODE_LOOP_HOLD    239
#define ALARM_CODE_LOOP_MFG     240
#define ALARM_CODE_LOOP_NONE    250
#define ALARM_CODE_LOOP_NULL    251
#define ALARM_CODE_LOOP_UNKN    252

//#define ANALOG_CHANNEL_FLAG   1
#define ANALOG_CHANNEL_FLAG 0
#define XFR_FUNCTION_NONE   0

#define DEV_STATUS_HIGH_BYTE    0

// Return ERR code

#define COMM_ERROR  0x80        // Communications error
#define VPAR_ERROR  0x40        // Vertical parity error
#define OVRN_ERROR  0x20        // UART Character Overrun error
#define FRAM_ERROR  0x10        // UART Character Framing error
#define LPAR_ERROR  0x08        // Longitudinal parity error
#define BOVF_ERROR  0x02        // Buffer overflow error

// Unit codes
#define PRESSURE_IN_PSI     6
#define CELSIUS             32
#define FARENHEIT           33
#define RANKIN              34
#define KELVIN              35
#define MILLIVOLTS          36
#define HERTZ               38
#define MILLIAMPS           39
#define MINUTES             50
#define SECONDS             51
#define HOURS               52
#define DAYS                53
#define PERCENT             57
#define VOLTS               58
#define DEGREES             143
#define RADIANS             144

// Command-specific response codes
#define RESP_SUCCESS            0
#define INVALID_POLL_ADDR_SEL   2
#define INVALID_SELECTION       2
#define PASSED_PARM_TOO_LARGE   3
#define PASSED_PARM_TOO_SMALL   4
#define TOO_FEW_DATA_BYTES      5
#define DEV_SPECIFIC_CMD_ERR    6
#define IN_WRITE_PROTECT_MODE   7
#define UPDATE_FAILURE          8
#define SET_NEAREST_POSSIBLE    8
#define LOWER_RANGE_TOO_HIGH    9
#define INVALID_DATE_CODE       9
#define CONFIG_COUNTER_MISMATCH 9
#define INCORRECT_LOOP_MODE     9
#define APPL_PROCESS_TOO_HIGH   9
#define APPL_PROCESS_TOO_LOW    10
#define LOWER_RANGE_TOO_LOW     10
#define UPPER_RANGE_TOO_HIGH    11
#define LOOP_CURRENT_NOT_ACTIVE 11
#define IN_MULTIDROP_MODE       11
#define INVALID_MODE_SEL        12
#define UPPER_RANGE_TOO_LOW     12
#define DYN_VARS_RETURNED       14
#define NEW_LOWER_RANGE_PUSHED  14
#define SPAN_TOO_SMALL          14
#define ACCESS_RESTRICTED       16
#define INVALID_SPAN            29
#define CMD_RESP_TRUNCATED      30
#define HART_DEVICE_BUSY        32

// Current Modes (command 6
#define CURRENT_MODE_DISABLE    0
#define CURRENT_MODE_ENABLE     1

// Address field masks 
#define BURST_MODE_BIT      0x40        // Burst mode bit
#define PRIMARY_MASTER      0x80
#define POLL_ADDR_MASK      0x3F
#define EXT_DEV_TYPE_ADDR_MASK  0x3FFF

// Lower status byte bit masks
#define FD_STATUS_DEVICE_MALFUNCTION    0x80
#define FD_STATUS_CONFIG_CHANGED        0x40
#define FD_STATUS_COLD_START            0x20
#define FD_STATUS_MORE_STATUS_AVAIL     0x10
#define FD_STATUS_PV_ANALOG_FIXED       0x08
#define FD_STATUS_PV_ANALOG_SATURATED   0x04
#define FD_STATUS_SV_OUT_OF_LIMITS      0x02
#define FD_STATUS_PV_OUT_OF_LIMITS      0x01

// Device Variable Codes
#define DVC_PV                      0
#define DVC_SV                      1
#define DVC_BATTERY_LIFE            243
#define DVC_PERCENT_RANGE           244
#define DVC_LOOP_CURRENT            245
#define DVC_PRIMARY_VARIABLE        246
#define DVC_SECONDARY_VARIABLE      247
#define DVC_TERTIARY_VARIABLE       248
#define DVC_QUARTERNARY_VARIABLE    249
#define DVC_INVALID_SELECTION       255

// exported variables
// The following structure is the HART startup information, which needs 
// to be stored in non-volatile memory
#define NAME_SIZE 32
#define DEVICE_ID_SIZE 3
#define SHORT_TAG_SIZE 6
#define LONG_TAG_SIZE 32
#define DESCRIPTOR_SIZE 12
#define DATE_SIZE 3
#define FINAL_ASSY_SIZE 3
#define HART_MSG_SIZE 24
#define TAG_DESCRIPTOR_DATE_SIZE 21
#define CONFIG_COUNTER_SIZE 2
#define DEV_SPECIFIC_STATUS_SIZE 6

#define NV_DEVICE_ID_LOCATION   VALID_SEGMENT_3


/*
Extended device status

01 - maint req
02 - dev variable alert
04 - critical power fail


Device op mode

0 - Reserved for now

Std status 0

01 - simulation active
02 - NV mem defect
04 - vol mem defect
08 - watchdog reset executed
10 - voltage condition OO range
20 - envirnmental condition OOR
40 - hardware problem,


*/

// Values for Standard Status 0
#define SS0_SIMULATION_ACTIVE   0x01
#define SS0_NV_MEM_DEFECT       0x02
#define SS0_VOL_MEM_DEFECT      0x04
#define SS0_WATCHDOG_CALLED     0x08
#define SS0_VOLTAGE_OOR         0x10
#define SS0_ENVIRONMENT_OOR     0x20
#define SS0_HARDWARE_PROBLEM    0x40

// Variable Status Flags
#define VAR_STATUS_GOOD         0xC0
#define VAR_STATUS_MAN_FIXED    0x80
#define VAR_STATUS_INACCURATE   0x40
#define VAR_STATUS_BAD          0x00
#define LIM_STATUS_CONST        0x30
#define LIM_STATUS_HIGH         0x20
#define LIM_STATUS_LOW          0x10
#define LIM_STATUS_NOT_LIMITED  0x00

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

// Utility unions
typedef union uLongFloat
{
    float fVal;
    unsigned char b[4];
} U_LONG_FLOAT;

typedef union uIntByte
{
    int i;
    unsigned char b[2];
} U_SHORT_INT;

typedef union uLongByte
{
    long i;
    unsigned char b[4];
} U_LONG_INT;


/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/
void incrementConfigCount(void);
void setPrimaryMasterChg(void);
void setSecondaryMasterChg(void);
void clrPrimaryMasterChg(void);
void clrSecondaryMasterChg(void);
void setPrimaryMoreAvailable(void);
void setSecondaryMoreAvailable(void);
void clrPrimaryMoreAvailable(void);
void clrSecondaryMoreAvailable(void);
void setPrimaryStatusBits(unsigned char);
void setSecondaryStatusBits(unsigned char);
void clrPrimaryStatusBits(unsigned char);
void clrSecondaryStatusBits(unsigned char);
void syncNvRam(void);
int isAddressValid(void);

// The programmed Device ID must be set in a different area of FLASH
// and copied into the local data on initialization
void copyNvDeviceIdToRam(void);
int copyDeviceIdToFlash(unsigned char *);
void verifyDeviceId(void);

// Utilities to copy ints and floats into the response buffer
void copyIntToRespBuf(int);
void copyLongToRespBuf(long);
void copyFloatToRespBuf(float);
float decodeBufferFloat(unsigned char *);
int decodeBufferInt(unsigned char *);
//
void UpdateSensorType(void);

/*************************************************************************
  *   $GLOBAL VARIABLES
*************************************************************************/
extern unsigned char updateNvRam;           // A flag so NV ram is only updated in the main loop
// Process variables
extern float PVvalue;
extern float SVvalue;
extern float ma4_20;
extern float savedLoopCurrent;
extern float reportingLoopCurrent;

extern HART_STARTUP_DATA_NONVOLATILE startUpDataLocalNv;
extern const HART_STARTUP_DATA_NONVOLATILE startUpDataFactoryNv;
extern HART_STARTUP_DATA_VOLATILE startUpDataLocalV;
extern const HART_STARTUP_DATA_VOLATILE startUpDataFactoryV;

extern const unsigned char * pNvDevId;
// If the device is busy, we need to respond appropriately
extern unsigned char deviceBusyFlag;

// Flags to indicate a HART command has been received
extern unsigned char cmdSyncToRam;
extern unsigned char cmdSyncToFlash;
extern unsigned char cmdReset;
extern unsigned char doNotRespond;

/*************************************************************************
  *   $INLINE FUNCTIONS
*************************************************************************/


	
#endif /*HART_H_*/
