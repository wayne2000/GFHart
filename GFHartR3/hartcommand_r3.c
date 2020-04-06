/*!
 *  \file     hartcommand_r3.c
 *  \brief    This module provides the HART command interface
 *
 *  Revision History:
 *  Date    Rev.   Engineer     Description
 *  -------- ----- ------------ --------------
 *  04/02/11  0    Vijay Soni    Creation
 *
 *  04/07/11  0    patel        Add funcations
 *
 */

#include <msp430f5528.h>
#include "define.h"
#include "hardware.h"
#include "protocols.h"
#include "hart_r3.h"
#include "common_h_cmd_r3.h"


#include "hartcommand_r3.h"

// Did we get a broadcast address?
int rcvBroadcastAddr = FALSE;
extern HART_STARTUP_DATA_VOLATILE startUpDataLocalV;


/*!
 * \function    processHartCommand()
 * \brief       Process the HART command, generate the response.
 *
 *    Isr and Drivers store Hart command message in a global buffer szHartCommadn[]. This function
 *    process the command (by creating a response) or prepares a error response
 *
 */
unsigned char processHartCommand (void)
{
	unsigned char rtnVal = FALSE;
	// Make sure the CD interrupt is off so no new messages can be received
	numMsgProcessed++;
	if (hartFrameRcvd)
	{
		// Verify that the number of data bytes received is <= expected byte count
		if (expectedByteCnt > hartDataCount)
		{
			return rtnVal;
		} 
		// Only proceed if the address is valid
		if (addressValid)
		{
			// If there is a short frame and the command is NOT 0, return
			if (!longAddressFlag &&(HART_CMD_0 != hartCommand))
			{
				return rtnVal;
			}
			if (rcvLrcError || parityErr || overrunErr)
			{
				executeCommErr ();
				rtnVal = TRUE;
			}
			else
			{
				// Make sure I have a short address before processing cmd 0
				if (!longAddressFlag)
				{
					if (HART_CMD_0 == hartCommand)
					{
						executeCmd0();
						rtnVal = TRUE;
					}
				}
				else
				{
					if (rcvBroadcastAddr)
					{
						if (HART_CMD_11 == hartCommand)
						{
							executeCmd11();
							rtnVal = (badTagFlag) ? FALSE : TRUE;
							badTagFlag = FALSE;
						}
						else if (HART_CMD_21 == hartCommand)
						{
							executeCmd21();
							rtnVal = (badTagFlag) ? FALSE : TRUE;
							badTagFlag = FALSE;
						}
						else
						{
							executeTxErr(CMD_NOT_IMPLEMENTED);
							rtnVal = TRUE;
						}
					}
					else
					{
						executeCommand();
						rtnVal = (badTagFlag) ? FALSE : TRUE;
						badTagFlag = FALSE;
					}
				}
			}
		}
		else
		{
			numMsgUnableToProcess++;
		}
	} 
	else
	{
		numMsgUnableToProcess++;
	}
	return rtnVal;
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: executeCommand()
//
// Description:
//
// Execute the HART command, generate the response. The function first checks to
// see if the command goes to a common handler, or a sensor-specific one. 
//
// Parameters: void
//
// Return Type: void
//
// Implementation notes:
//		This function steers the command to the correct handler based upon the 
//      sensor type
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void executeCommand(void)
{
	// First, check to see if the command is a common command. If not,
	// then select the specific sensor handler
	switch(hartCommand)
	{
	case HART_CMD_0:
		common_cmd_0();
		break;		
	case HART_CMD_1:
		common_cmd_1();
		break;		
	case HART_CMD_2:
		common_cmd_2();
		break;		
	case HART_CMD_3:
		common_cmd_3();
		break;	
	case HART_CMD_6:
		common_cmd_6();
		break;		
	case HART_CMD_7:
		common_cmd_7();
		break;		
	case HART_CMD_8:
		common_cmd_8();
		break;		
	case HART_CMD_9:
		common_cmd_9();
		break;		
	case HART_CMD_11:
		common_cmd_11();
		break;		
	case HART_CMD_12:
		common_cmd_12();
		break;		
	case HART_CMD_13:
		common_cmd_13();
		break;		
	case HART_CMD_14:
		common_cmd_14();
		break;		
	case HART_CMD_15:
		common_cmd_15();
		break;		
	case HART_CMD_16:
		common_cmd_16();
		break;		
	case HART_CMD_17:
		common_cmd_17();
		break;		
	case HART_CMD_18:
		common_cmd_18();
		break;		
	case HART_CMD_19:
		common_cmd_19();
		break;		
	case HART_CMD_20:
		common_cmd_20();
		break;		
	case HART_CMD_21:
		common_cmd_21();
		break;		
	case HART_CMD_22:
		common_cmd_22();
		break;		
#ifdef IMPLEMENT_RANGE_CMDS_35_36_37		
	case HART_CMD_35:
		common_cmd_35();
		break;		
	case HART_CMD_36:
		common_cmd_36();
		break;		
	case HART_CMD_37:
		common_cmd_37();
		break;		
#endif		
	case HART_CMD_38:
		common_cmd_38();
		break;		
	case HART_CMD_39:
		common_cmd_39();
		break;	
	case HART_CMD_40:
		common_cmd_40();
		break;	
#if 0
	case HART_CMD_42:   // We don't support CMD_42  MH 12/6/12
		common_cmd_42();
		break;
#endif

#if 0		
	case HART_CMD_43:
		common_cmd_43();
		break;
#endif			
	case HART_CMD_45:
		common_cmd_45();
		break;	
	case HART_CMD_46:
		common_cmd_46();
		break;	
	case HART_CMD_48:
		common_cmd_48();
		break;	
	case HART_CMD_54:
		common_cmd_54();
		break;	
	case HART_CMD_57:
		common_cmd_57();
		break;	
	case HART_CMD_58:
		common_cmd_58();
		break;	
	case HART_CMD_110:
		common_cmd_110();
		break;	
	case HART_CMD_219:
		mfr_cmd_219();
		break;	
	case HART_CMD_220:
		mfr_cmd_220();
		break;	
	case HART_CMD_221:
		mfr_cmd_221();
		break;	
	case HART_CMD_222:
		mfr_cmd_222();
		break;	

	// If it is not a common command, select the
	// handler based upon the sensor type	
	default:	
#ifdef USE_MULTIPLE_SENSOR_COMMANDS	
		switch(startUpDataLocalV.defaultSensorType)
		{
		case CONDUCTIVITY_TYPE:
			executeConductivityCommand();
			break;	
		case LEVEL_TYPE:
			executeLevelCommand();
			break;	
		case ORP_TYPE:
			executeOrpCommand();
			break;	
		case PH_TYPE:
			executePhCommand();
			break;			
		case PRESSURE_TYPE:
			executePressureCommand();
			break;	
		case MA4_20_TYPE:
			executeMa4_20Command();
			break;			
		case FLOW_TYPE:
		default:
			executeFlowCommand();
			break;
		}
#else
		common_tx_error(CMD_NOT_IMPLEMENTED);
		break;
#endif		
	}
}		

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: executeFlowCommand()
//
// Description:
//
// Execute the HART command, generate the response.  
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
void executeFlowCommand(void)
{
	common_tx_error(CMD_NOT_IMPLEMENTED);
}

#ifdef USE_MULTIPLE_SENSOR_COMMANDS	

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: executeConductivityCommand()
//
// Description:
//
// Execute the HART command, generate the response.  
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
void executeConductivityCommand(void)
{
	common_tx_error(CMD_NOT_IMPLEMENTED);
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: executeLevelCommand()
//
// Description:
//
// Execute the HART command, generate the response.  
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
void executeLevelCommand(void)
{
	common_tx_error(CMD_NOT_IMPLEMENTED);
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: executeMa4_20Command()
//
// Description:
//
// Execute the HART command, generate the response.  
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
void executeMa4_20Command(void)
{
	common_tx_error(CMD_NOT_IMPLEMENTED);
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: executeOrpCommand()
//
// Description:
//
// Execute the HART command, generate the response. 
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
void executeOrpCommand(void)
{
	common_tx_error(CMD_NOT_IMPLEMENTED);
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: executePhCommand()
//
// Description:
//
// Execute the HART command, generate the response. 
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
void executePhCommand(void)
{
	common_tx_error(CMD_NOT_IMPLEMENTED);
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: executePressureCommand()
//
// Description:
//
// Execute the HART command, generate the response.  
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
void executePressureCommand(void)
{
	common_tx_error(CMD_NOT_IMPLEMENTED);
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: executeCmd0()
//
// Description:
//
// Execute the HART command 0, generate the response. 
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
void executeCmd0(void)
{
	common_cmd_0();
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: executeCmd11()
//
// Description:
//
// Execute the appropriate HART command 11, generate the response.  
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
void executeCmd11(void)
{
	common_cmd_11();
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: executeCmd21()
//
// Description:
//
// Execute the appropriate HART command 21, generate the response.  
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
void executeCmd21(void)
{
	common_cmd_21();
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: executeTxErr()
//
// Description:
//
// Execute the  HART tx err command, generate the response. 
//
// Parameters: unsigned char error code
//
// Return Type: void
//
// Implementation notes:
//
// 
//
/////////////////////////////////////////////////////////////////////////////////////////// 
void executeTxErr(unsigned char errCode)
{
	common_tx_error(errCode);
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Function Name: executeCommErr()
//
// Description:
//
// Execute the  HART comm err command, generate the response. 
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
void executeCommErr(void)
{
	common_tx_comm_error();
}


