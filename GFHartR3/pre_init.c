/*!
 *  pre_init.c
 *  This routine provides a pre initialization of CCS
 *
 *  Disables Watch dog as standard 32mS time slot is not enough to init all global and static
 *
 *  Created on: Nov 29, 2012
 *      Author: MH
 *  Disable Watchdog before running zero initialization segment
 *
 */
#include <msp430f5528.h>
//int _system_pre_init (void)
int _system_pre_init (void)
{
  // disable watchdog timer
  //------------------------
  WDTCTL = WDTPW + WDTHOLD;               // Stop WDT

  // place your code for hardware initialization here

  /*==================================*/
  /* Choose if segment initialization */
  /* should be done or not.           */
  /* Return: 0 to omit seg_init       */
  /*         1 to run seg_init        */
  /*==================================*/
  return (1);
}



