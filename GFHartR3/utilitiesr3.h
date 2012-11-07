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
// Header File Name:  utilitiesr3.h
//
// Description:
//
// This module provides utility functions for the flash memory, and other 
// miscellaneous functions. Functions are implemented in the file, utilities.c
//
// Revision History:
// Date    Rev.   Engineer     Description
// -------- ----- ------------ --------------
// 04/04/11  0    Vijay Soni    Creation
//
// 04/07/11  0    Patel  	Add funcations
//
///////////////////////////////////////////////////////////////////////////////////////////
 
 
#ifndef UTILITIES_H_
#define UTILITIES_H_

// Misc. defines

// Flash function prototypes
int copyMainFlashToMem (unsigned char *, unsigned char *, int);
int copyMemToMainFlash (unsigned char *, unsigned char *, int);
int eraseMainSegment(unsigned char *, int);
int verifyFlashContents(unsigned char *, unsigned char *, int);
void syncToRam(unsigned char *, unsigned char *, int);
int syncToFlash(unsigned char *, unsigned char *, int);
int calcNumSegments (int);

// Flash definitions
#define MAIN_SEGMENT_SIZE	128
// Four segments (2048 bytes) are valid for writing, starting at 0x1000
// The Information memory segment starts at 0x18OO and goes to address 0x1c00,
// So these areas must be avoided. Program memory starts at 0x4400, at 
// the other end of the flash memory 
#define VALID_SEGMENT_1 (unsigned char *)0x1800
#define VALID_SEGMENT_2 (unsigned char *)(VALID_SEGMENT_1+MAIN_SEGMENT_SIZE)  // 1200
#define VALID_SEGMENT_3 (unsigned char *)(VALID_SEGMENT_2+MAIN_SEGMENT_SIZE)  // 1400
#define VALID_SEGMENT_4 (unsigned char *)(VALID_SEGMENT_3+MAIN_SEGMENT_SIZE)  // 1600
#define INFO_MEMORY_END (unsigned char *)0x1A00

// Misc. utility prototypes
float IntToFloat (int);
extern unsigned int flashWriteCount;

#endif /*UTILITIES_H_*/
