/*!
 *  \file   errorHandler.c
 *  \brief  Trap unused interrupts
 *  Created on: Nov 29, 2012
 *  \author: MH
 */
#include "hardware.h"
#include <msp430f5528.h>

//  List of unused vectors

//   63 is System Reset = Not trapped =


/*!
 * System NMI  62
 */
#pragma vector=SYSNMI_VECTOR
__interrupt void _hart_SYSNMI_VECTOR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}

/*!
 *  User NMI  61
 */
#pragma vector=UNMI_VECTOR
__interrupt void _hart_UNMI_VECTOR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}

/*!
 * Comp_B  60
 */
#pragma vector=COMP_B_VECTOR
__interrupt void _hart_COMP_B_VECTOR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}


/*!
 * TB0 TB0CCR1 to TB0CCR6,TB0IFG 58
 */
#pragma vector=TIMER0_B1_VECTOR
__interrupt void _hart_TIMER0_B1_VECTOR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}


/*!
 * Watchdog Timer_A Interval mode  57
 */
#pragma vector=WDT_VECTOR
__interrupt void _hart_WDT_VECTOR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}



/*!
 * USCI_B0 Rec/Tra 55
 */
#pragma vector=USCI_B0_VECTOR
__interrupt void _hart_USCI_B0_VECTOR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}

/*!
 * ADC12_A 54
 */
#pragma vector=ADC12_VECTOR
__interrupt void _hart_ADC12_VECTOR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}


/*!
 * TA0 TA0CCR1 to TA0CCR4, TA0IFG  52
 */
#pragma vector=TIMER0_A1_VECTOR
__interrupt void _hart_TIMER0_A1_VECTOR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}


/*!
 * USB_UBM USB interrupts  51
 */
#pragma vector=USB_UBM_VECTOR
__interrupt void _hart_USB_UBM_VECTOR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}


/*!
 * DMA  50
 */
#pragma vector=DMA_VECTOR
__interrupt void _hart_DMA_VECTOR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}



/*!
 * TA1 TA1CCR1 to TA1CCR2,   TA1IFG (TA1IV)(1) (3)
 */
#pragma vector=TIMER1_A1_VECTOR
__interrupt void _hart_TIMER1_A1_VECTOR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}


/*!
 * I/O Port P1 47
 */
#pragma vector=PORT1_VECTOR
__interrupt void _hart_PORT1_VECTOR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}



/*!
 * USCI_B1 45
 */
#pragma vector=USCI_B1_VECTOR
__interrupt void _hart_USCI_B1_VECTOR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}

#if 0

//  Timer A2 interrupt service routine for CC0, TA2CCR0 (44)
//  Hart slave response time
#pragma vector=TIMER2_A0_VECTOR
__interrupt void slaveReplyTimerISR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}
#endif



/*!
 * TA2 TA2CCR1 to TA2CCR2,TA2IFG 43
 */
#pragma vector=TIMER2_A1_VECTOR
__interrupt void _hart_TIMER2_A1_VECTOR(void)
{
#ifdef TRAP_INTERRUPTS
  while(1);
#endif
  //  A reti; is generated by compiler
}