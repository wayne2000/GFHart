//////////////////////////////////////////////////////////////////////////////////////////
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
// Header File Name:  common_h_cmd.h
//
// Description:
//
// This module provides common HART interface command. Functions are implemented in 
// the file, common_h_cmd.c
//
// Revision History:
// Date    Rev.   Engineer     Description
// -------- ----- ------------ --------------
// 04/23/11  0    Vijay Soni    Creation
//
//
///////////////////////////////////////////////////////////////////////////////////////////

#ifndef COMMON_H_CMD_H_
#define COMMON_H_CMD_H_

void common_cmd_0(void);
void common_cmd_1(void);
void common_cmd_2(void);
void common_cmd_3(void);
void common_cmd_6(void);
void common_cmd_7(void);
void common_cmd_8(void);
void common_cmd_9(void);
void common_cmd_11(void);
void common_cmd_12(void);
void common_cmd_13(void);
void common_cmd_14(void);
void common_cmd_15(void);
void common_cmd_16(void);
void common_cmd_17(void);
void common_cmd_18(void);
void common_cmd_19(void);
void common_cmd_20(void);
void common_cmd_21(void);
void common_cmd_22(void);
#ifdef IMPLEMENT_RANGE_CMDS_35_36_37
void common_cmd_35(void);
void common_cmd_36(void);
void common_cmd_37(void);
#endif
void common_cmd_38(void);
void common_cmd_39(void);
void common_cmd_40 (void);
void common_cmd_42 (void);
void common_cmd_43 (void);
void common_cmd_45 (void);
void common_cmd_46 (void);
void common_cmd_48(void);
void common_cmd_54 (void);
void common_cmd_57 (void);
void common_cmd_58 (void);
void common_cmd_110 (void);


void mfr_cmd_219(void);
void mfr_cmd_220(void);
void mfr_cmd_221(void);
void mfr_cmd_222(void);


void common_tx_error(unsigned char);
void common_tx_comm_error(void);

float CalculatePercentRange(float, float, float);

extern unsigned char badTagFlag;

#endif /*COMMON_H_CMD_H_*/
