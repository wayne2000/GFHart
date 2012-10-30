/*
 *  \file   merge.h
 *  \brief  Solve compile errors during merge conflicts with original sw.
 *  Keep it light, at the end it will be deleted
 *
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

#define NO_CURRENT_MESSAGE_SENT     0xF0
#define FIXED_CURRENT_MESSAGE_SENT  0xF1
#define LOOP_CURRENT_MESSAGE_SENT   0xF2


//==============================================================================
//  $GLOBAL PROTOTYPES
//==============================================================================
void copy9900factoryDb(void);
void initStartUpData();
void initalizeLocalData(void);

unsigned char setFixedCurrentMode(float cmdValue);  // Main9900.c

//==============================================================================
//
//  $GLOBAL VARIABLES
//
//==============================================================================
extern unsigned char currentMsgSent;
extern unsigned int flashWriteCount;
extern long dataTimeStamp;
extern int8u updateDelay;

//==============================================================================
//
//  $INLINE FUNCTIONS
//==============================================================================

#endif /* MERGE_H_ */
