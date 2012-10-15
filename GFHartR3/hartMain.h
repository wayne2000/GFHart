/*
 *  \file   hartMain.h
 *  \brief  Hart main application exported information
 *  \author MH
 *  Created on: Sep 28, 2012
 */

#ifndef HARTMAIN_H_
#define HARTMAIN_H_
/*************************************************************************
  *   $INCLUDES
*************************************************************************/

/*************************************************************************
  *   $DEFINES
*************************************************************************/
typedef enum
{
	evNull=0,									//!< Strictly is an event occupying bit location in Event memory, but ordinal gives zero
  //	Hart Receiver
	evHartRxChar,							//!< Hart receiver has a new element data + status in input stream
	evHartRcvGapTimeout,			//!< Inter-character time exceded
  evHartRcvReplyTimer,			//<! Just a silent time between master command and a slave reply

  // System
  evTimerTick,							//!< general purpose System Time tick

  evLastEvent								//!< For implementation use: define last event

} tEvent;


/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/
void main (void);
void _c_int00(void);    //!< Entry point if we are commanded to reset
/*************************************************************************
  *   $GLOBAL VARIABLES
  *
  */
extern unsigned int sEvents[];	// Word array where events are stored
/*************************************************************************
  *   $INLINE FUNCTIONS
*************************************************************************/
//	Note on implementation of Events:
//	A bit within a word is assigned for every event. Basic macro operations are provided: set, clear and test
//	When implementation finds an event as a #define, it should promote the use of preprocessor arithmetic
//	when it finds as a variable, a call to function is generated
//
#define SET_SYSTEM_EVENT(e) 		(	sEvents[e >>4] |= 	0x0001 << e % 0x10	)			/* set the indicated event */
#define CLEAR_SYSTEM_EVENT(e) 	( sEvents[e >>4] &= ~(0x0001 << e % 0x10)	)			/*	clear the indicated event */
#define IS_SYSTEM_EVENT(e)			( (sEvents[e >> 4] >> e % 0x10) & 0x0001)				/* Is the event set? */


#endif /* HARTMAIN_H_ */
