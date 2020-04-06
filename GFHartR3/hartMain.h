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
/*!
 * Enumerated events
 *
 * These are the registered events that Hart module responds to
 *
 */
typedef enum
{
  evNull=0,                 //!< Strictly is an event occupying bit location in Event memory, but ordinal gives zero
  evHsbRecComplete,         //!< Hsb Command message has been received, 12/26/12 Moved to 1st priority

  //  Hart Receiver
  evHartRxChar,             //!< Hart receiver has a new element data + status in input stream, moved to 2nd priority, still 9.1mS for next


  evHartRcvGapTimeout,      //!< Inter-character time exceded
  evHartRcvReplyTimer,      //<! Just a silent time between master command and a slave reply
  evHartTransactionDone,    //<! The Hart modem is set to LISTEN mode after the reply is complete, we still have
                            //   around ~78mS silent line before the bits from following Hart frame arrive

  /////////////////////////////

  //////////////////////////

  // System
  evTimerTick,              //!< general purpose System Time tick

  evLastEvent               //!< For implementation use: define last event

} tEvent;

/*!
 * Hsb states
 *
 * Create a basic state machine to recover from errors and trigger a signal
 *
 */
typedef enum
{
  smHsbInit=0,              //!< Initial State for the HSB
  smHsbDbXchg,              //!< Data Base Exchange
  smHsbSync                //!< After Data Base has been loaded, Hsb is sync. (is when Low power is possible)
} tHsbStates;


/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/
void main (void);
void _c_int00(void);    //!< Entry point if we are commanded to reset
/*************************************************************************
  *   $GLOBAL VARIABLES
  *
  */
extern volatile unsigned int sEvents[];	// Word array where events are stored

/*************************************************************************
  *   $INLINE FUNCTIONS
*************************************************************************/
//	Note on implementation of Events:
//	A bit within a word is assigned for every event. Basic macro operations are provided: set, clear and test
//	When implementation finds an event as a #define, it should promote the use of preprocessor arithmetic
//	when it finds as a variable, a call to function is generated
//  12/28/12 = We need a Atomic operation that summarizes a no-event condition. This restriction reduces to only 16 events.
//
#define SET_SYSTEM_EVENT(e) 		(	sEvents[0] |= 	0x0001 << e	)			    /* set the indicated event */
#define CLEAR_SYSTEM_EVENT(e) 	( sEvents[0] &= ~(0x0001 << e))			    /*	clear the indicated event */
#define IS_SYSTEM_EVENT(e)			(sEvents[0] & (0x0001<< e) )            /* Is the event set? */
#define ANY_EVENT()             (sEvents[0] ? : 1 :0)                    /* gives a TRUE if at least one event, false other wise */
#define NO_EVENT()              (sEvents[0]== 0 ? 1 : 0)                 /* Returns TRUE if no event , false otherwise ==>safe condition to sleep*/


#endif /* HARTMAIN_H_ */
