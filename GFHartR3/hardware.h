#ifndef HARDWARE_H_
#define HARDWARE_H_
/*!
 *  \file   hardware.h
 *  \brief  Provides the hardware interface for GF Hart implementation
 *
 *  Software switches "define" are declared here.
 *
 *  Created on: Sep 28, 2012
 *  \author: MH
 */

/*************************************************************************
  *   $INCLUDES
*************************************************************************/
#include "msp_port.h"
/*************************************************************************
  *   $DEFINES
*************************************************************************/
//
// Software Configuration Switch defines
//  Analying side effects 11/13/12
//
//  Always included code for (original project define) QUICK_START

/*!
 * Compare current consumption on low power modes vs. active mode
 *
 * Used to compare current consumption in low power vs. active mode.\n
 * While developing SW sometimes is necessary to work always in Active mode as some silicon revisions
 * has emulator Errata. Production code should have this define
 */
#define LOW_POWERMODE_ENABLED
/*!
 * Define how deep the microcontroller will sleep in low power
 *
 * Define the sleep depth: LPM0_bits to LPM3_bits.\n
 * Production code is LMP0  - MH 12/11/12
 */
#define LPM_BITS  LPM0_bits

/*!
 * Silicon Rev. E has UCS10 errata
 *
 * We remove the impact of UCS10 by using ACLK to generate Hart Baud rate\r
 * Select ONE of the available clock sources for Hart UART:
 * - HART_UART_USES_ACLK
 * - HART_UART_USES_SMCLK
 * Production code uses ACLK
 *
 */
#define HART_UART_USES_ACLK
//#define HART_UART_USES_SMCLK
/*
 * WatchDog Time Interval
 *
 * WD uses ACLK (32K) and Time interval possible values are
 * WDTIS_3   16 Sec
 * WDTIS_4   1 Sec
 * WDTIS_5   0.25 Sec
 * WDTIS_6   0.0156 Sec
 * Production Code Time interval is WDTIS_4 = 1 Sec
 */
#define WD_TIME_INTERVAL WDTIS_4

/*!
 *  Enable ACLK and SMCLK monitoring in pins
 *
 *  During testing Clock jumps (silicon rev. E) we need to observe ACLK and SMCL;
 *  may be removed from production code to save some power
 */
#define MONITOR_ACLK
// #define MONITOR_SMCLK

//
//  DISABLE_FLASH_WRTING
//  This define won't allow writing to Flash as writing to flash requires 40mS.
//  During some testing CPU availability sometimes I need to evaulate best time
//  to write. This define is only for debug
//#define DISABLE_INTERNAL_FLASH_WRITE

//
//  Trap all unused interrupts
//  undefine for production code
#define TRAP_INTERRUPTS

// 	Timers clock source are ACLK/8 = 32768/8 = 4096hz
//	Reload value is (mS -1)*4.096
//		8, 508, 8188 40956u      	Tick aprox 2mS, 125mS, 2 Sec, 10Sec
#define SYSTEM_TICK_TPRESET	508
#define SYSTEM_TICK_MS  125
#define NUMBER_OF_MS_IN_24_HOURS  86400000
//  Minimum time to wait between Flash writes (other conditions apply)
//  Time is in mS 0 to 65000
#define FLASH_WRITE_MS  2000  /*  Change from 4000 to 2000 since less chance to catch a write */

/*!
 *  Hart slave timers
 *
 *  They behave like watchdog and are kicked everytime a char is received
 *  - Inter-character GAP time for 2 chars @1200bps =  11* 2 * 4096 /1200 =  75
 *  - Slave reply time for 1.5 chars @1200bps =  11* 1.5 * 4096 /1200 =  57
 *  - Time out is measured from RX STOP bit + 1 Baud time + Timer preset
 *
 *  Gap timer
 *  - Start:  when command message start delimiter is received
 *  - Stop:   at LRC command reception reaches (we don't care when extra char ends)
 *            after receiving a Gap Time out notification, just after calling initHartRx extrnal prep
 *  - Event   Taken when hartFrameRcvd is false, i.e. avoid if there is a valid message to reply to
 *            the event should notify the error and initHartRx state machine
 *
 *  Reply Timer
 *  - Start   when receiver reaches LRC (we are going to reply to all complete messages, some may not be to us)
 *  - Stop    When the Reply event is taken
 *  - Event   hartFrameRcvd should be true
 */
#define GAP_TIMER_PRESET  82      /* 2 chars + 10% = 2.2* 9.1666 ~ 20mS  or 82 counts THIS IS PRODUCTION SETTING */
//  Follows test and comments
//#define GAP_TIMER_PRESET  75    /* 18mS ~ 75 tested w 50mS~201 and hyperterminal    */  (== ORIGINAL VALUE)
//#define GAP_TIMER_PRESET  56    /* 13.5mS (1.5 chars) accelerate fault probability  */  (PROBLM found, kickHartRecTimer to ISR)
//#define GAP_TIMER_PRESET  38      /* 38~10 mS (1.1 chars) test functionality, (FUNCTIONALITY TESTED)  with 39 (9.5) takes a long time to fail*/
#define REPLY_TIMER_PRESET 53     /* 14mS ~ 53, tested w 100mS~406 and hyperterminal */
/*!
 *  High Speed Bus timer
 *  This timer measures the start of a Hsb message $H and times out to receive next command
 *  -Start    when hsb receives $H
 *  -At Time Out it stops the counter (MC=0) and Enables HSB Rx interrupt
 */
#define HSB_ATTENTION_CCR_PRESET   573   /* Set to 140mS Theoretically should be 150 - 2*0.512= 149mS*/
/*!
 *  Flash Write slot-
 *  Strategy is to not drop any Hart message. Hart will trigger a Flash write at the end of TX bit,
 *  when RTS goes high.
 *  Second condition is if HSB is in the IDLE slot. We are going to reduce the IDLE_SLOT, ideally by 40mS
 *  which is the time to write flash. 150-40 = 110mS. With a 20% safety margin, IDLE is 88 mS
 *
 *
 *///  12/7/12 Flash WRITE
//  To obtain the
//#define HSB_IDLE_SLOT     545   /* Theoretically should be 150mS less a safety margin give 133mSto be @middle */
//#define HSB_IDLE_SLOT  361      /* 88 mS, test if we drop a HSB message, WE are listening to last "part" of $R section */
//#define HSB_IDLE_SLOT  409      /* 100 mS, test if we drop a HSB message - SAVE is taken, listening 20 mS of $R section */
//#define HSB_IDLE_SLOT  450      /* 110 mS, test if we drop a HSB message - SAVE is taken, listening 7.6 mS of $R section */
#define HSB_IDLE_SLOT  492        /* 120mS - SAVE has higher probability to be taken sooner, May drop HSB message if syncFlash requires flashing */


/*
 * HSB_SEQUENCER
 * This define generates a delay to start Hart first and then HSB
 *
 */
//#define HSB_SEQUENCER

/*!
 *  Hart is started first and Hsb starts after following delay
 *  ticks are x125mS
 */
#ifdef HSB_SEQUENCER
#define HART_HSB_SEQUENCE_DELAY 12    /* 12 x 0.125 =  1.5 Sec */
#endif

/*!
 *  Hart is stopped if we are not able to communicate with 9900
 *  This is the way original code handled:
 *  Timeout preset to Drop Hart comm when 9900 is not communicating (in 125ms ticks)
 */
#define MAX_9900_TIMEOUT  40  /* 40 x 125mS ticks = 5 secs  */

/*!
 * Supervisory HSB watchdog
 *
 * If HSB doesn't receive messages, due a lost of sync, excessive errors, etc
 * It may result in a disabled receiver. This timer intends to re-init the serial
 * port RX interrupt after no sensing HSB acitvity
 * Ticks are in x125 mS ticks
 */
#define HSB_NO_ACTIVITY_TIMEOUT  9  /* Reinit HSB serial port after 9*0.125 = 1.125 SECS */

/*!
 *  FORCE_FLASH_WRITE
 *  This define simulates a change in NV memory such that flashing happens often
 *  Used to debug flash writes as it introduces a 40mS delay
 */

//  #define FORCE_FLASH_WRITE     /* will let it run for weekend */

/*!
 * DEBUG_SIGN_MAINLOOP_TOGGLE
 * The main loop is toggled every time is take to measure its scan time.
 * This define signs the scan duration by toggling "event number" times TPx at the end of loop scan
 * Note: if event (shouldn't happen) is "evNull", evLastEvent toggles are used
 */
#define DEBUG_SIGN_MAINLOOP_TOGGLE

//HICCUP
#define NO_CURRENT_MESSAGE_SENT   0xF0
#define FIXED_CURRENT_MESSAGE_SENT  0xF1
#define LOOP_CURRENT_MESSAGE_SENT 0xF2

//  Promote to Global as it is seen by driverUart 12/26/12
#define HART_PREAMBLE       0xFF


/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/
void INIT_set_Vcore(unsigned char);
void INIT_SVS_supervisor(void);

void initHardware(void);            //!< Initialize Peripherals for the Hart Application
//void stop_oscillator();             //!< Stops the CPU clock and go to sleep mode

/*************************************************************************
  *   $GLOBAL VARIABLES
*************************************************************************/
extern volatile WORD SistemTick125mS;
//HICCUP
extern unsigned char currentMsgSent;
/*!
 *  bHartRecvFrameCompleted
 *  This global indicates that a complete (until LRC) Hart frame has been received in the hartReceiver (main loop)
 *  which runs outside the interrupt receiver. The flag is used to block a evHartRcvGapTimeout events
 */
extern volatile BOOLEAN bHartRecvFrameCompleted;

/*************************************************************************
  *   $INLINE FUNCTIONS
*************************************************************************/


#if 0
//MH   These function have given me so many troubles - just discard and rewrite proper ones
startWatchdog(), stopWatchdog(),  resetWatchdog()
#endif

/*!
 *  kickWatchdog()
 *  Reset WD internal counter
 */
inline void kickWatchdog(void)
{
  WDTCTL = WDTPW | WDTCNTCL;
}
/*!
 *  startWatchdog()
 *  Assigns a predefined value to the WDTCTL: set to watchdog POR, clock source is ACLK
 *  Time interval WD_TIME_INTERVAL is defined for porduction to WDTIS_4 (1 Sec @32KHz)
 */
inline void startWatchdog(void)
{
  WDTCTL = WDTPW | WDTSSEL_1 | WDTCNTCL | WD_TIME_INTERVAL; // WD clk is ACLK and trips at 32K (1 secs)
}

/*!
 *  stopWatchdog()
 *  Stops the WD counter, this should be matched by a startWatchdog() on its context
 *  We use the same clk source in order not to request a CLOCK change (and alter LPMx)
 */
inline void stopWatchdog(void)
{
  WDTCTL = WDTPW | WDTHOLD | WDTSSEL_1 | WD_TIME_INTERVAL;   // Sets the HOLD bit, WD is not operative
}


/*!
 *	kickHartGapTimer()
 *	Reset timer count register to zero
 *
 */
inline void kickHartGapTimer(void)					//!< Kick the Hart DLL dog
{
	HART_RCV_GAP_TIMER_CTL 		|= TACLR;     // Count=0
}

/*!
 *	startGapTimerEvent()
 *	Start Counting Up and generate an interrupt if reaches preset
 *
 */
inline void startGapTimerEvent(void)					//!< Kick the Hart DLL dog
{
	HART_RCV_GAP_TIMER_CTL |= MC_1;						// Up mode
}


/*!
 *	startReplyTimerEvent()
 *	Start Counting Up and generate an interrupt if reaches preset
 *
 */
inline void startReplyTimerEvent(void)					//!< Kick the Hart DLL dog
{
	HART_RCV_REPLY_TIMER_CTL |= MC_1 ;						// Up mode
}

/*!
 *  startHsbAttentionTimer()
 *  Start Counting Up to the initialized preset and generate an interrupt when done
 *  \return As a side effect, it will generate and interrupt TIMER0_A0_VECTOR
 *
 */
inline void startHsbAttentionTimer()
{
  HSB_ATTENTION_TIMER_CTL |= MC_1;           //  Count in Up mode
}

/*!
 *  stopHsbAttentionTimer()
 *  Stop the Timer from counting
 *
 */
inline void stopHsbAttentionTimer()           //(typo error corrected)
{
  HSB_ATTENTION_TIMER_CTL  &= ~MC_3;         //  This makes MC_0 = stop counting
}




#endif /* HARDWARE_H_ */
