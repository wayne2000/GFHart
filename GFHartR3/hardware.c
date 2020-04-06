/*!
 *  \file   hardware.c
 *  \brief  Provides the hardware interface for GF Hart implementation
 *
 *
 *  Created on: Sep 28, 2012
 *  \author: MH
 */

//==============================================================================
//  INCLUDES
//==============================================================================
#include "define.h"
#include "msp_port.h"
#include "hardware.h"
#include "driverUart.h"
#include "hartMain.h"

//==============================================================================
//  LOCAL DEFINES
//==============================================================================

//==============================================================================
//  LOCAL PROTOTYPES.
//==============================================================================
static void toggleRtsLine();
//==============================================================================
//  GLOBAL DATA
//==============================================================================
volatile WORD SistemTick125mS=0;
unsigned char currentMsgSent = NO_CURRENT_MESSAGE_SENT;
volatile BOOLEAN bHartRecvFrameCompleted = FALSE;

//  volatile WORD flashWriteTimer =0;   // Original timer was 4000 mS, local var
//==============================================================================
//  LOCAL DATA
//==============================================================================

//==============================================================================
// FUNCTIONS
//==============================================================================


/*!
 * \fn  INIT_SVS_supervisor()
 *
 *  \brief    Enables supply voltage supervision and monitoring
 *
 */
void INIT_SVS_supervisor(void)
{
  // remove the SVS from the circuit to prevent uncommanded resets
   PMMCTL0_H = 0xA5;          // Un-Lock PMM module registers for write access
   SVSMLCTL &= ~( SVMLE + SVSLE);    // Disable Low side SVM
   SVSMHCTL &= ~( SVMHE + SVSHE);    // Disable High side SVM

   PMMCTL0_H = 0x00;           // Lock PMM module registers for write access
}


/*!
 * \fn  INIT_set_Vcore
 *
 * \brief     Set The VCore Voltage
 * \param     Vcore Level
 * \return    none
 *
 */
void INIT_set_Vcore (unsigned char level)
{
  PMMCTL0_H = 0xA5;   // Unlock PMM module registers to allow write access

  // Set SVS/M high side to new level
  SVSMHCTL = (SVSMHCTL & ~(SVSHRVL0*3 + SVSMHRRL0)) | \
             (SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level);

  // Set SVM new Level
  SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
  // Set SVS/M low side to new level
  SVSMLCTL = (SVSMLCTL & ~(SVSMLRRL_3)) | (SVMLE + SVSMLRRL0 * level);

  while ((PMMIFG & SVSMLDLYIFG) == 0);      // Wait till SVM is settled (Delay)
  PMMCTL0_L = PMMCOREV0 * level;            // Set VCore to x
  PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);        // Clear already set flags

  if ((PMMIFG & SVMLIFG))
    while ((PMMIFG & SVMLVLRIFG) == 0);     // Wait till level is reached

  // Set SVS/M Low side to new level
  SVSMLCTL = (SVSMLCTL & ~(SVSLRVL0*3 + SVSMLRRL_3)) | \
             (SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level);

  PMMCTL0_H = 0x00;   // Lock PMM module registers to prevent write access
}

/*!
 * \fn checkClock
 * Checks and clear any oscillator fault
 * Periodically (100 to 500 mS) call this routine to assure XT1 oscillator is ON
 * JY
 */
void checkClock(void)
{
  while ( (SFRIFG1 & OFIFG))              // Check OFIFG fault flag
  {   // Clear OSC fault flags
    UCSCTL7 &= ~(DCOFFG + XT1LFOFFG + /* XT1HFOFFG */ + XT2OFFG);
    SFRIFG1 &= ~OFIFG;                  // Clear OFIFG fault flag
  }
}

/*!
 * \fn  initClock(void)
 * \brief     Set clocks MCLK & SMCLK - DCO 1.048576MHz enable FLL:XT1,
 *            ACLK -XT1 xtal 32.768KHz
 *
 * Main clock MCLK source is DCO, maximum permissible freq is 1.048MHz for Low power
 * DCO stabilized by FLL (as default) but using XT1 as the reference clock.
 * SMCLK will be used for HSB Uart, ACLK for Hart Uart and system timers
 *
 * 12/11/12
 * - FLL is enabled at Power up. After XT1 starts up, we disable FLL and leave DCO without compensation
 *    (this is a temp solution since our Hart HW has silicon rev. E with the UCS10 errata)
 * - Future designs expect silicon rev F/G/H+ without FLL -clock jump problem
 *
 * \author  MH 9/29/12
 */
void initClock(void)
{

    // X2 should be off after POR - Turn it off if we came from a Soft-reset
  UCSCTL6 |= XT2OFF;          // Set XT2 Off
  //  Enable XT1 Clock pins
  XT1_PORTREN &= ~(XIN_MASK | XOUT_MASK);   // remove resistors (if any) from Xtal pins
  XT1_PORTDIR |= XOUT_MASK;                 // XOUT as output (JY)
  XT1_PORTOUT |= XOUT_MASK;                 // XOUT high      (JY lowest drive?)
  XT1_PORTSEL |= XIN_MASK | XOUT_MASK;      // Xtal function, this enables XT1 oscillator but uses REFO until XT1 stabilizes

  //  Values from TI examples are the defaults after a POR- MH 11/19/12
  //  XT2DRIVE=3, X2TOFF=1, XT1DRIVE=3, XTS=0, XT1BYPASS=0, SMCLKOFF=0, XCAP=3 (12pF), XT1OFF=1
  //  Use no internal caps as this board has external 22pF - Recommend CL external value should be 12pF each
  UCSCTL6 &=  ~(XCAP0_L | XCAP1_L | XT1OFF);  // Clear defaults XCAP_3 and XT1OFF
  //  ==> set the internal Caps as necessary UCSCTL6 |= XCAP_x
  //
  //  Setup DCO frequency by the FLL frequency multiplier FLLN and loop divider FLLD
  //  After stabilization: DCOCLK = (32768/1 ) * ( 1 * (31+1)) = 1,048,576 Hz
  //  UCSCTL2 = FLLD__1 |       //  FLLD=0  divide-feedback-by-1 ?? default is 1 and still div-by-1
  //            0x001F;         //  FLLN=31 divide feedback by 31+1
  //  ===> UCSCTL2 = 0x101F is the default as TI example: 0x101F -MH 11/19/12

  //  Set FLL clock reference and its divider FLLREFDIV
  UCSCTL3 = SELREF_0  |     //  SELREF = 0  FLL reference is XT1 = 32768 Hz, XT1 should start from here
            FLLREFDIV__1;   //  FLLREFDIV=0 FLL reference div-by-1

  // Select the system clocks (same as TI example, default 0x0044) as Some favor DCOCLKDIV over DCOCLK (less jitter)
  //  ACLK = XT1CLK, SMCLK = DCOCLKDIV, MCLK  = DCOCLKDIV
  UCSCTL4 =   SELA_0 |      //  ACLK  = XT1CLK  =   32.768Khz external crystal
              SELS_4 |      //  SMCLK = DCOCLKDIV=  1,048,576   as TI Example (default) MH -11/19/12
              SELM_4;       //  MCLK  = DCOCLKDIV=  1,048,576   as TI Example (default) MH -11/19/12

  // Set clocks Dividers DIVPA = DIVA = DIVS = DIVM = 1 (these are Defaults and same as TI example)
  UCSCTL5 = DIVPA_0 |       //  ACLKpin = ACLK /1
            DIVA_0  |       //  ACLK /1
            DIVS_0  |       //  SMCLK /1
            DIVM_0;         //  MCLK /1

  //  Loop until XT1 & DCO stabilize.
  //  Clearing fault generated by XT1 allows the FLL to switch from REFO to XT1 again, until XT1 exceeds fault level
  //  We stay in this loop for ~800mS, (maximum startup time is 500mS: 3V, 32Khz,XTS=0,XT1DRIVE=3,CLeff=12pF).
  //  After clearing OFIFG fault, we wait 10mS before testing the bit
  //
  WORD u=0;
  do
  {
    UCSCTL7 &= ~(XT1LFOFFG | DCOFFG);       //  Clear XT1 and DCO fault flags
    SFRIFG1 &= ~OFIFG;                      //  Clear fault flags
    __delay_cycles(10000);                  //  10mS @1Mhz - this is a delay before asking for Flasg status
  } while (u++ < 80 || (SFRIFG1&OFIFG) );   //  Force to test oscillator fault flag for at least 800mS
  //
#ifdef MONITOR_ACLK
  // Monitor Clocks: ACLK --> P1.0
  P1DIR |= BIT0;
  P1SEL |= BIT0;  //P1.0 is ACLK    32.768KHz
  _no_operation();                          //  Debug point: DCO calibrated close to 32*32768 ~ 1.048MHz and FLL OFF
#endif
#ifdef MONITOR_SMCLK
  // Monitor Clocks: SMCLK --> P2.2
  P2DIR |= BIT2;
  P2SEL |= BIT2;  //P2.2 is SMCLK   1.048MHz
  _no_operation();                          //  Debug point: DCO calibrated close to 32*32768 ~ 1.048MHz and FLL OFF
#endif
  //
  //    Done with system clock settings
  //    TEMPORARY code: Turn OFF FLL.
  // FINALLY Enable FLL  __bis_SR_register(SCG0);                  // Disable the FLL control loop === Will be applied during the Idle Slot ===
  //  12/06/12  FLL will be always ON - unless evaluation of Rev. F shows a lot improvement
  //
  _no_operation();
  _no_operation();
  _no_operation();                          //  Debug point: DCO calibrated close to 32*32768 ~ 1.048MHz and FLL OFF


}

/*!
 * 	\fn  initTimers()
 * 	Configure the msp430 timers as follows:
 * 	- System timer: TB, TBCLK = ACLK/8 = 4096Hz, 8 ticks/sec,
 * 	- HSB slot timeout, up, ACLK/8 = 4096Hz
 * 	- Hart receiver gap between characters TA1, up, ACLK/8 = 4096Hz
 * 	-	Hart slave message reply TA2, up, ACLK/8 = 4096Hz,
 */
initTimers()
{

	// Timer B0 = System Timer
  TBCCTL0 = CCIE;     // TRCCR0 interrupt enabled
  TBCCR0 =  SYSTEM_TICK_TPRESET;

  TBCTL = TBSSEL_1 |  // TBCLK -> ACLK source
          MC_1 |      // Up Mode
          ID_3 |      // TBCLK = ACLK/8 = 4096 Hz
          TBCLR;      // Clear

  // Timer A1 = Hart Rec. GAP timing
  HART_RCV_GAP_TIMER_CTL =		TASSEL_1 |				// Clock Source = ACLK
    													ID_3 |						// /8
    													TACLR;						// Clear
  HART_RCV_GAP_TIMER_CCR	= 	GAP_TIMER_PRESET;	// CCR0 preset
  HART_RCV_GAP_TIMER_CCTL =		CCIE;							// Enable CCR0 interrupt

  // Timer A2 = Hart Rec. Slave reply time
  HART_RCV_REPLY_TIMER_CTL =	TASSEL_1	|				// Clock Source = ACLK
  														ID_3 |						// /8
  														TACLR;						//	Clear
  HART_RCV_REPLY_TIMER_CCR = 	REPLY_TIMER_PRESET;// CCR0
  HART_RCV_REPLY_TIMER_CCTL =	CCIE;							// Enable CCR0 interrupt


  // Timer A0 = HSB Attention timer: Enables the RXIE to get the starting command
  HSB_ATTENTION_TIMER_CTL  =        TASSEL_1  |       // Clock Source = ACLK
                                    ID_3 |            // /8
                                    TACLR;            // Clear
  HSB_ATTENTION_TIMER_CCR  =       HSB_ATTENTION_CCR_PRESET;
  HSB_ATTENTION_TIMER_CCTL =       CCIE;             // Enable interrupt

}



/*
 * \fn    initHardware
 * Initializes clock system, set GPIOs, init USB power, set Hart in Rx mode, init uC Uarts and timers
 *
 */
void initHardware(void)
{

  stopWatchdog();
  /////////////////////////////////////////////////////////////////////////////
  //  2/5/13  === RTS on Modem A5191 side should be High during Reset
  // Hart Transmit control
  HART_UART_TXCTRL_PORTSEL &= ~HART_UART_TXCTRL_MASK;    // Make sure pin is GPIO out
  HART_UART_TXCTRL_PORTDIR |= HART_UART_TXCTRL_MASK;
  HART_UART_TXCTRL_PORTOUT |= HART_UART_TXCTRL_MASK;    // app disableHartTxDriver();   // Put Modem Line in listen mode
  //
  /////////////////////////////////////////////////////////////////////////////
  //  Increase RTS Drive strength  2/4/13 !MH
  HART_UART_TXCTRL_PORTDS |= HART_UART_TXCTRL_MASK;
  //
  toggleRtsLine();
  //
  /////////////////////////////////////////////////////////////////////////////

  // Change clock initialization earlier
  initClock();

  //
  // Set up GPIO test pins
  P1OUT &= ~0xF0;  // We are using TPS to trigger data logging
  TP_PORTDIR = (TP1_MASK | TP2_MASK | TP3_MASK | TP4_MASK); // Test Points as Outputs
  //
  //!MH These original configurations looks like we can have a soft-reset
  P1SEL &= ~(BIT2|BIT3);  // i/o pins
  P1DIR &= ~(BIT2|BIT3);  // input
  P1DS &= ~(BIT2|BIT3);   // Drive strength low
  P1IES &= ~(BIT2|BIT3);  // lo->hi transition (for now)
  P1IFG &= ~(BIT2|BIT3);  // clear IFG in case it's set


  // Hart Uart Pins
  HART_UART_PORTSEL |= HART_UART_RX_MASK | HART_UART_TX_MASK;
  HART_UART_PORTDIR |= HART_UART_TX_MASK;
  HART_UART_PORTDS  |= HART_UART_TX_MASK;       // Full output drive strength in TX

  // initialize the clock system
  initClock();

  _disable_interrupts(); //vj  make sure all irq are disable
  //  Pulse TxRxHart line to Tx mode two times and leave it in Rx Mode
  toggleRtsLine();
  // Added the following function:
  stopWatchdog();

  // Disable VUSB LDO and SLDO
  USBKEYPID   =     0x9628;           // set USB KEYandPID to 0x9628: access to USB config registers enabled
  USBPWRCTL &= ~(SLDOEN+VUSBEN);      // Disable the VUSB LDO and the SLDO
  USBKEYPID   =    0x9600;            // access to USB config registers disabled

  unsigned int pmmReg;
  // Make Sure the power control is low as possible
  pmmReg = PMMCTL0;
  pmmReg &= 0x00FC;
  PMMCTL0 = PMMPW | pmmReg;
  //
  //
  // Set VCore
  INIT_set_Vcore(PMMCOREV_0);
  //
  initTimers();

  //USE_PMM_CODE
  INIT_SVS_supervisor();

  /// Here we go, Enable watchdog()
  startWatchdog();

}

/*!
 * \fn    toggleRtsLine
 * Toggle TxRx Hart line to have extern FF in a defined state and levae it at Receiver mode
 *
 *  This function assures that the TxRx will always be on RxMode, by toggling the TxRx line
 *  into TxMode two times 140uS ea. It uses software delay with clock at 1.048Mhz
 *
 *  !MH == 2/4/13  The 140uS is marginal for the hardware AC-coupling:  R8=499 Ohms, C7=0.033uS
 *  4*T*=  67us, Rise + Fall + 2 Level *1.1 margin =
 *    Pulse width at least 300uS
 *  Optocoupler also contributes with 7.5*2uS = 15uS
 *    Patch will provide double= 500uS pulse delay
 *  2/5/13== only a single positive transition is necessary
 *
 * \param   none
 * \return  none
 *
 *
 */
void toggleRtsLine()
{

  volatile int i;
  _no_operation();
  for(i=0; i< 1; ++i)   // 2/5/13 Only One positve transition is necessary
  {
    disableHartTxDriver();
    _delay_cycles(500);
    enableHartTxDriver();
    _delay_cycles(500);
    kickWatchdog();
  }
  disableHartTxDriver(); // make sure rts is in receive mode
  //
  //
  P1IFG &= ~(BIT2|BIT3); // clear the DCD flags just in case they've been set
}

/*!
 * Timer B0 interrupt service routine for CCR0
 * TB0 TB0CCR0 59
 *
 */
#pragma vector=TIMER0_B0_VECTOR
__interrupt void _hart_TIMER0_B0_VECTOR(void)
{
  _no_operation();
  ++SistemTick125mS;
  SET_SYSTEM_EVENT(evTimerTick);
#ifdef LOW_POWERMODE_ENABLED
  _bic_SR_register_on_exit(LPM_BITS);
  _no_operation();    //
  _no_operation();
  _no_operation();
#endif
}


/*!
 * Timer A0 interrupt service routine for CC0 TIMER0_A0_VECTOR (53)
 * HSB Slot Timer - Active (50mS) and Idle (100mS)
 *
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void hsbAttentionTimerISR(void)
{
  //TOGGLEB(TP_PORTOUT, TP2_MASK);     // Mark the Enable POINT
  hsbUart.hRxInter.enable();
  stopHsbAttentionTimer(); // Stop This event   (corrected typo error)
  hsbActivitySlot = TRUE;  // We are preparing for RX, DO NOT WRITE NOW ON, DO NOT FALL TO SLEEP

  // This is the best time to clear the HSB === 12/28/12
  BYTE rxbyte = UCA0RXBUF;                     // read & clears the RX interrupt flag and UCRXERR status flag

  //while(1); // Trap
#ifdef LOW_POWERMODE_ENABLED
  _bic_SR_register_on_exit(LPM_BITS);
  _no_operation();    //
  _no_operation();
  _no_operation();
#endif
}

/*!
 * Timer A1	interrupt service routine for CC0, TA1CCR0 (49)
 * Hart Inter-character GAP expired
 * 12/26/12 the gap timer is one shot
 *
 */
#pragma vector=TIMER1_A0_VECTOR
__interrupt void gapTimerISR(void)
{
  //SETB(TP_PORTOUT, TP3_MASK);             // Indicate an Gap Error and leave pin there
  if(!bHartRecvFrameCompleted)              // We time-out but hartReceiver() has ACK and waiting to Reply
    SET_SYSTEM_EVENT(evHartRcvGapTimeout);

	// Stop the timer (Only one shot)
	HART_RCV_GAP_TIMER_CTL  &= ~MC_3;     // This makes MC_0 = stop counting
	HART_RCV_GAP_TIMER_CTL  |= TACLR;     // Clear TAR

	// while(1);	// TRAP HART PROBLEM == Didn't get Gap Error => Looking for STACK
#ifdef LOW_POWERMODE_ENABLED
	_bic_SR_register_on_exit(LPM_BITS);
	_no_operation();    //
	_no_operation();
	_no_operation();
#endif
}

/*!
 * Timer A2	interrupt service routine for CC0, TA2CCR0 (44)
 * Hart slave response time, we also stop the timer here to
 * have a ONE shot timer. Main loop doesn't need to control timer
 *
 */
#pragma vector=TIMER2_A0_VECTOR
__interrupt void slaveReplyTimerISR(void)
{
	SET_SYSTEM_EVENT(evHartRcvReplyTimer);
	//  12/26/12 Stop timer from an ISR
	HART_RCV_REPLY_TIMER_CTL  &= ~MC_3;     // This makes MC_0 = stop counting
	HART_RCV_REPLY_TIMER_CTL  |= TACLR;     // Clear the counting register TAR


#ifdef LOW_POWERMODE_ENABLED
	_bic_SR_register_on_exit(LPM_BITS);
	_no_operation();    //
	_no_operation();
	_no_operation();
#endif
	//while(1);	// Trap
}
