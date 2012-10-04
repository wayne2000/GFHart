/*
 * define.h
 *
 *  Created on: Sep 19, 2012
 *      Author: Marco.HenryGin
 *      General type definitions

 */

#ifndef DEFINE_H_
#define DEFINE_H_


/* Type definitions */
typedef unsigned      char      BOOLEAN;
typedef unsigned      char      BYTE;
typedef signed        char      SBYTE;
typedef unsigned      int       WORD;
typedef signed        int       SWORD;
typedef unsigned long int       LWORD;
typedef signed   long int       SLWORD;

// These typedefs are used for 9900 communications
typedef unsigned char int8u;
typedef unsigned int int16u;

typedef union uByteFloat
{
    float floatVal;
    int8u byteVal[4]; // Allows easy conversion of FP -> hex
} fp32;



/* Boolean definitions */
#define TRUE    1
#define FALSE   0
#define YES     1
#define NO      0
#define ON      1
#define OFF     0

/* Logical definitions */
#define AND     &&
#define OR      ||
#define NOT     !

/* Bit-masking operations */
#define BIT_AND         &
#define BIT_OR          |
#define BIT_NOT         ~

/* Bit operation on registers */
#define SETB(p,m)   p |= m
#define CLEARB(p,m) p &= ~m
#define TOGGLEB(p,m) p ^= m

/* Error values */
#define NO_ERROR        0
#define YES_ERROR       1

/* character defines */
#define BLANK  0x20     /* blank ASCII character */
#ifndef EOF
#define EOF    -1
#endif
#define NEWLINE '\n'    /* End of line terminator */
#ifndef NULL
#define NULL (0L)
#endif


/* macro expressions */
#define DIM(x)  ((int)(sizeof(x) / sizeof(x[0])))
#define TBD     ;               /* flag to fix something */


#endif /* DEFINE_H_ */
