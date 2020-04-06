/*!
 *  \file  msp_port.h
 *  \brief Collect all MSP430F5528 defines related to Hart Board
 *  \author MH
 *  Created on: Sep 19, 2012
 *
 */

#ifndef MSP_PORT_H_
#define MSP_PORT_H_
/*************************************************************************
*   $INCLUDES
*************************************************************************/
#include <msp430f5528.h>
#include "define.h"

/*************************************************************************
*   $DEFINES
*************************************************************************/


//#define HART_ICTL   UCA1ICTL
// A0 is the HSB UART
//#define MAIN_CTL1   UCA0CTL1
//#define MAIN_BR0    UCA0BR0
//#define MAIN_BR1    UCA0BR1
//#define MAIN_MCTL   UCA0MCTL
//#define MAIN_IV     UCA0IV
//#define MAIN_IE     UCA0IE
//#define MAIN_IFG    UCA0IFG
//#define MAIN_STAT   UCA0STAT
//#define MAIN_CTLW0  UCA0CTLW0

//#define MAIN_BRW    UCA0BRW
//#define MAIN_ABCTL  UCA0ABCTL
//#define MAIN_IRCTL  UCA0IRCTL
//#define MAIN_IRTCTL UCA0IRTCTL
//#define MAIN_IRRCTL UCA0IRRCTL
//#define MAIN_ICTL   UCA0ICTL

/////////////  Hardware Pins    ////////////////
//  Test Points
#define TP_PORTOUT P1OUT
#define TP_PORTDIR P1DIR
#define TP1_MASK 0x10
#define TP2_MASK 0x20
#define TP3_MASK 0x40
#define TP4_MASK 0x80

//  X1 Xtal
#define XT1_PORTOUT  P5OUT
#define XT1_PORTDIR  P5DIR
#define XT1_PORTSEL  P5SEL
#define XT1_PORTREN  P5REN
#define XIN_MASK     0x10
#define XOUT_MASK    0x20

//  Hart Uart
#define HART_UART_PORTSEL  P4SEL        /*!<  msp430 Hart port SEL */
#define HART_UART_PORTDIR  P4DIR        /*!<  msp430 Hart port DIR */
#define HART_UART_PORTDS   P4DS         /*!<  msp430 Hart port Drive strength DS */
#define HART_UART_TX_MASK   0x10        /*!<  msp430 Hart port pin mask for TX */
#define HART_UART_RX_MASK   0x20        /*!<  msp430 Hart port pin mask for RX */

// HArt Uart Tx control (RTS) line HI= Driver disable
#define HART_UART_TXCTRL_PORTOUT  P4OUT /*!<  Hart msp430 TX driver port OUT */
#define HART_UART_TXCTRL_PORTSEL  P4SEL /*!<  Hart msp430 TX driver port SEL */
#define HART_UART_TXCTRL_PORTDIR  P4DIR /*!<  Hart msp430 TX driver port DIR */
#define HART_UART_TXCTRL_MASK   0x01    /*!<  Hart msp430 TX driver pin port MASK */
#define HART_UART_TXCTRL_PORTDS  P4DS   /*!<  Hart msp430 RTS Hi Drive Strength */


//  High Speed Bus uses TA0
#define HSB_ATTENTION_TIMER_TR       TA0R        /*!< Hsb slot counter Value */
#define HSB_ATTENTION_TIMER_CTL      TA0CTL      /*!< Hsb slot Setup Register */
#define HSB_ATTENTION_TIMER_CCR      TA0CCR0     /*!< Compare register CCR0 for Hsb slot time */
#define HSB_ATTENTION_TIMER_CCTL     TA0CCTL0    /*!< CCTL0 control register for Hsb slot module */

//  Use the CCR1 module of TA0 to reduce the WRITE TO FLASH opportunity
#define HSB_ATTENTION_TIMER_FLASH_WR_DISABLE_CCR  TA0CCR1     /*!< Time to end the Hsb flash write slot */
#define HSB_ATTENTION_TIMER_FLASH_WR_DISABLE_CCTL TA0CCTL1    /*!< Control for the capture module of Hsb Flash wr slot */

//	Hart Receiver Dog timers
//	Reception GAP between chars uses TA1
#define HART_RCV_GAP_TIMER_TR					TA1R				/*!< Hart Gap timer count Value */
#define HART_RCV_GAP_TIMER_CTL				TA1CTL			/*!< Hart gap timer Setup Register */
#define HART_RCV_GAP_TIMER_CCR				TA1CCR0			/*!< Hart compare register CCR0 for GAP	time */
#define HART_RCV_GAP_TIMER_CCTL				TA1CCTL0		/*!< CCTL0 control register for GAP	module */

//	Slave Reply uses TA2
#define HART_RCV_REPLY_TIMER_TR				TA2R				/*!< Hart Reply timer Count Value */
#define HART_RCV_REPLY_TIMER_CTL			TA2CTL			/*!< Hart Reply timer Setup Register */
#define HART_RCV_REPLY_TIMER_CCR			TA2CCR0			/*!< Hart reply timer Compare register CCR0  */
#define HART_RCV_REPLY_TIMER_CCTL			TA2CCTL0		/*!< Hart reply timer CCTL0 Control register	*/

// Definitions from msp430x.h not found in CCS
#define P1OUT_              (0x0202u)  /* Port 1 Output */
#define P2OUT_              (0x0203u)  /* Port 2 Output */
#define P3OUT_              (0x0222u)  /* Port 3 Output */
#define P4OUT_              (0x0223u)  /* Port 4 Output */

#define STOP_WD() ( WDTCTL = WDTPW | WDTHOLD)



/*!
 *  Used to set/reset/test pin in a port using inlines
 *  Dropped caus generated non-atomic functions
 *
 *
 */
typedef struct
{
  int8u *port;
  int8u mask;
} tIoPinPort;


#endif /* MSP_PORT_H_ */
