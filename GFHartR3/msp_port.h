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
#define MAIN_CTL0   UCA0CTL0
#define MAIN_CTL1   UCA0CTL1
#define MAIN_BR0    UCA0BR0
#define MAIN_BR1    UCA0BR1
#define MAIN_MCTL   UCA0MCTL
#define MAIN_IV     UCA0IV
#define MAIN_IE     UCA0IE
#define MAIN_IFG    UCA0IFG
#define MAIN_RXBUF  UCA0RXBUF
#define MAIN_TXBUF  UCA0TXBUF
#define MAIN_STAT   UCA0STAT
#define MAIN_CTLW0  UCA0CTLW0
#define MAIN_BRW    UCA0BRW
#define MAIN_ABCTL  UCA0ABCTL
#define MAIN_IRCTL  UCA0IRCTL
#define MAIN_IRTCTL UCA0IRTCTL
#define MAIN_IRRCTL UCA0IRRCTL
#define MAIN_ICTL   UCA0ICTL

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
#define HART_UART_PORTSEL  P4SEL
#define HART_UART_PORTDS   P4DS
#define HART_UART_TX_MASK   0x10
#define HART_UART_RX_MASK   0x20


// Definitions from msp430x.h not found in CCS
#define P1OUT_              (0x0202u)  /* Port 1 Output */
#define P2OUT_              (0x0203u)  /* Port 2 Output */
#define P3OUT_              (0x0222u)  /* Port 3 Output */
#define P4OUT_              (0x0223u)  /* Port 4 Output */

#define STOP_WD() ( WDTCTL = WDTPW | WDTHOLD)



// Hardware abstraction for the Hart MOdule
typedef struct
{
  int8u *port;
  int8u mask;
} tIoPinPort;


#endif /* MSP_PORT_H_ */
