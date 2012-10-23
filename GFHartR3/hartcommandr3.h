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
// Header File Name:  HartCommand.h
//
// Description:
//
// This module provides HART command interface. Functions are implemented in 
// the file, HartCommand.c
//
// Revision History:
// Date    Rev.   Engineer     Description
// -------- ----- ------------ --------------
// 04/02/11  0    Vijay Soni    Creation
//
// 04/07/11  0    patel    	Add funcations
//
///////////////////////////////////////////////////////////////////////////////////////////
#ifndef HARTCOMMAND_H_
#define HARTCOMMAND_H_

// Offsets
#define CMD_SOF_OFFSET			0
#define CMD_ADDR_FIELD_OFFSET	CMD_SOF_OFFSET
#define CMD_LONG_ADDR_OFFSET	CMD_ADDR_FIELD_OFFSET+5
#define CMD_SHORT_ADDR_OFFSET	CMD_ADDR_FIELD_OFFSET+1
#define CMD_SHORT_CMD_OFFSET	CMD_SHORT_ADDR_OFFSET
#define CMD_LONG_CMD_OFFSET		CMD_LONG_ADDR_OFFSET

// Command values
#define HART_CMD_0		0
#define HART_CMD_1		1
#define HART_CMD_2		2
#define HART_CMD_3		3
#define HART_CMD_6		6
#define HART_CMD_7		7
#define HART_CMD_8      8
#define HART_CMD_9		9
#define HART_CMD_11		11
#define HART_CMD_12		12
#define HART_CMD_13		13
#define HART_CMD_14		14
#define HART_CMD_15		15
#define HART_CMD_16		16
#define HART_CMD_17		17
#define HART_CMD_18		18
#define HART_CMD_19		19
#define HART_CMD_20		20
#define HART_CMD_21		21
#define HART_CMD_22		22
#define HART_CMD_35		35
#define HART_CMD_36		36
#define HART_CMD_37		37
#define HART_CMD_38		38
#define HART_CMD_39		39
#define HART_CMD_40		40
#define HART_CMD_42		42
#define HART_CMD_43		43
#define HART_CMD_45		45
#define HART_CMD_46		46
#define HART_CMD_48		48
#define HART_CMD_54		54
#define HART_CMD_57		57
#define HART_CMD_58		58
#define HART_CMD_110	110
#define HART_CMD_128	128
#define HART_CMD_129	129
#define HART_CMD_131	131
#define HART_CMD_135	135
#define HART_CMD_139	139
#define HART_CMD_140	140
#define HART_CMD_142	142
#define HART_CMD_143	143
#define HART_CMD_146	146
#define HART_CMD_147	147
#define HART_CMD_150	150
#define HART_CMD_160	160
#define HART_CMD_161	161
#define HART_CMD_162	162
#define HART_CMD_163	163
#define HART_CMD_164	164
#define HART_CMD_165	165

#define HART_CMD_219	219
#define HART_CMD_220	220
#define HART_CMD_221	221
#define HART_CMD_222	222

unsigned char processHartCommand (void);
void executeCommand(void);

#ifdef USE_MULTIPLE_SENSOR_COMMANDS	
// Prototypes of individual sensor command handlers
void executeFlowCommand(void);
void executeConductivityCommand(void);
void executeLevelCommand(void);
void executeMa4_20Command(void);
void executeOrpCommand(void);
void executePhCommand(void);
void executePressureCommand(void);
#endif

void executeCmd0(void);
void executeCmd11(void);
void executeCmd21(void);
void executeTxErr(unsigned char);
void executeCommErr(void);

#endif /*HARTCOMMAND_H_*/
