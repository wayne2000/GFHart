/*!
 *  \file   hardware.c
 *  \brief  Provides the hardware interface for GF Hart implementation
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
//  volatile WORD flashWriteTimer =0;   // Original timer was 4000 mS, local var
//==============================================================================
//  LOCAL DATA
//==============================================================================

//==============================================================================
// FUNCTIONS
//==============================================================================


/*!
 * \function  INIT_SVS_supervisor()
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
 * \function  INIT_set_Vcore
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
 * \function checkClock
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
 * \function  initClock(void)
 * \brief     Set clocks MCLK & SMCLK - DCO 1.048576MHz FLL:XT1, ACLK -XT1 xtal 32.768KHz
 *
 * Main clock MCLK source is DCO, maximum permissible 1.048MHz (Low power)
 * DCO stabilized by FLL (as default) using XT1 as the reference clock.
 * ACLK source is XT1 and connected to external 32.768KHz crystal (TBD Evaluate LPM0, and if necessary ACLK->LPM3)
 * SMCLK will be used for Uarts and system timers
 * Note:    It takes 775mS to Stabilize
 * \author  MH 9/29/12
 */
void initClock(void)
{

  // X2 should be off after POR - Turn it off if we came from a Soft-reset
  UCSCTL6 |= XT2OFF;          // Set XT2 Off
  //  XT1 Clock pins
  XT1_PORTREN &= ~(XIN_MASK | XOUT_MASK);   // remove resistors (if any) from Xtal pins
  XT1_PORTDIR |= XOUT_MASK;                 // XOUT as output (JY)
  XT1_PORTOUT |= XOUT_MASK;                 // XOUT high      (JY lowest drive?)
  XT1_PORTSEL |= XIN_MASK | XOUT_MASK;      // Xtal function, this enables XT1 oscillator but uses REFO here
  UCSCTL6 =  ~(XCAP0_L | XCAP1_L) & UCSCTL6 | XCAP_2;     // Clear default XCAP_3 and set Cap = (10.5 + 2)pF
  //
  //
  //  Setup DCO and FLL
  //  DCOCLK = (32768/1 ) * ( 1 * (31+1)) = 1,048,576 Hz
  UCSCTL2 = FLLD__1 |       //  FLLD=0  divide-feedback-by-1
            0x001F;         //  FLLN=31 divide feedback by 31+1
  UCSCTL3 = SELREF_0  |     //  SELREF = 0  FLL reference is XT1 = 32768 Hz, XT1 should start from here
            FLLREFDIV__1;   //  FLLREFDIV=0 FLL reference div-by-1

  // Select the clocks
  UCSCTL4 |=  SELA_0 |      //  ACLK  = XT1CLK  =   32.768Khz external crystal
              SELS_4 |      //  SMCLK = DCOCLKDIV=  1,048,576
              SELM_4;       //  MCLK  = DCOCLKDIV=  1,048,576

  // Set clocks Dividers DIVPA = DIVA = DIVS = DIVM = 1
  UCSCTL5 = DIVPA_0 |       //  ACLKpin = ACLK /1
            DIVA_0  |       //  ACLK /1
            DIVS_0  |       //  SMCLK /1
            DIVM_0;         //  MCLK /1

  // Loop until XT1 & DCO stabilizes. Clear faults generated by FLL clk switching from REFO to XT1
  //!DEBUG: Time to stabilize ACLK and DCO = Typ 550mS, 750mS Max
  SETB(TP_PORTOUT, TP1_MASK);  P1DIR |=TP1_MASK; // TP1 indicates time to stabilize XT1 & DCO
  do
  {
    UCSCTL7 &= ~(XT2OFFG | XT1LFOFFG | DCOFFG); // Clear XT2,XT1,DCO fault flags
    SFRIFG1 &= ~OFIFG;                          // Clear fault flags
  } while (SFRIFG1&OFIFG);                      // Test oscillator fault flag
  //!DEBUG:
  CLEARB(TP_PORTOUT, TP1_MASK);  // TP1 On duration indicates XT1&DCO stabilization
  _no_operation();          //  Debug point: DO any change here
  //
  //    Done with system clock settings
}

/*!
 * 	\function  initTimers()
 * 	Configure the msp430 timers as follows:
 * 	- System timer: TB, TBCLK = ACLK/8 = 4096Hz, 8 ticks/sec,
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




}
/*
 * \function    initHardware
 * Initializes clock system, set GPIOs, init USB power, set Hart in Rx mode, init uC Uarts and timers
 *
 */
void initHardware(void)
{

  stopWatchdog();
  //
  // Set up GPIO test pins
  TP_PORTDIR = (TP1_MASK | TP2_MASK | TP3_MASK | TP4_MASK); // Test Points as Outputs
  //
  //!MH These original configurations looks like we can have a soft-reset
  P1SEL &= ~(BIT2|BIT3);  // i/o pins
  P1DIR &= ~(BIT2|BIT3);  // input
  P1DS &= ~(BIT2|BIT3);   // Drive strength low
  P1IES &= ~(BIT2|BIT3);  // lo->hi transition (for now)
  P1IFG &= ~(BIT2|BIT3);  // clear IFG in case it's set

  // Hart Transmit control
  HART_UART_TXCTRL_PORTSEL &= ~HART_UART_TXCTRL_MASK;    // Make sure pin is GPIO out
  HART_UART_TXCTRL_PORTDIR |= HART_UART_TXCTRL_MASK;
  HART_UART_TXCTRL_PORTOUT |= HART_UART_TXCTRL_MASK;    // app disableHartTxDriver();   // Put Modem Line in listen mode

  // Hart Uart Pins
  HART_UART_PORTSEL |= HART_UART_RX_MASK | HART_UART_TX_MASK;
  HART_UART_PORTDIR |= HART_UART_TX_MASK;
  HART_UART_PORTDS  |= HART_UART_TX_MASK;       // Full output drive strength in TX

  // initialize the clock system
  initClock();
  // Monitor Clocks: ACLK --> P1.0, SMCLK --> P2.2
  P1DIR |= BIT0;
  P1SEL |= BIT0;  //P1.0 is ACLK    32.768KHz
  P2DIR |= BIT2;
  P2SEL |= BIT2;  //P2.2 is SMCLK   1.048MHz

  _disable_interrupts(); //vj  make sure all irq are disable
  //  Pulse TxRxHart line to Tx mode two times and leave it in Rx Mode
  toggleRtsLine();

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

}

/*!
 * \function    toggleRtsLine
 * Toggle TxRx Hart line to have extern FF in a defined state and levae it at Receiver mode
 *
 *  This function assures that the TxRx will always be on RxMode, by toggling the TxRx line
 *  into TxMode two times 140uS ea. It uses software delay with clock at 1.048Mhz
 * \param   none
 * \return  none
 *
 *
 */
static void toggleRtsLine()
{
  // variables needed for the RTS initialization sequence
  unsigned int idleCount = 0;
  unsigned int idleCount_1 = 0;
  unsigned int rts_init = 0;
  unsigned int second_round = 0;

  while(idleCount < 0x2ff) //0x9ff) //0x27ff) //0x9fff)
  {
      ++idleCount;
      resetWatchdog();
  }
  // Toggle RTS a few times
  idleCount=0;
  while(!rts_init)
  {
    if (idleCount > 0xf)
    {
        disableHartTxDriver();
        ++idleCount_1;
    }
    else
    {
        ++idleCount;
        enableHartTxDriver();
    }
    if (idleCount_1 > 0xf)
    {
        idleCount_1 = 0;
        idleCount = 0;
        if (second_round == 1)
        {
            rts_init = 1;
        }
        second_round = 1;
    }
  }
  disableHartTxDriver(); // make sure rts is in receive mode
  idleCount =0;
  P1IFG &= ~(BIT2|BIT3); // clear the DCD flags just in case they've been set
  while(idleCount < 0x2ff) //0x9ff) //0xfff)
  {
      ++idleCount;
      resetWatchdog();
  }
}

/*!
 * Timer B0 interrupt service routine
 *
 */
#pragma vector=TIMERB0_VECTOR
__interrupt void TIMERB1_ISR(void)
{
  _no_operation();
  ++SistemTick125mS;
  SET_SYSTEM_EVENT(evTimerTick);
  //TOGGLEB(TP_PORTOUT, TP1_MASK);            // Toggle TP1
}

/*!
 * Timer A1	interrupt service routine for CC0
 * 0= Inter character GAP expired
 *
 */
#pragma vector=TIMER1_A0_VECTOR
__interrupt void gapTimerISR(void)
{
	SET_SYSTEM_EVENT(evHartRcvGapTimeout);
	//while(1);	// Trap
}
/*!
 * Timer A1	interrupt service routine for CC1-CC2 and TAIFG
 * 1=Slave response time
 *
 */
#pragma vector=TIMER2_A0_VECTOR
__interrupt void slaveReplyTimerISR(void)
{
	SET_SYSTEM_EVENT(evHartRcvReplyTimer);
	//while(1);	// Trap
}

