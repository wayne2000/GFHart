/*!
 *  \file   define.h
 *  \brief  Global types and defines for the project
 *
 *  Created on: Sep 19, 2012
 *  Author:     MH
 *

 */

#ifndef DEFINE_H_
#define DEFINE_H_


/* Type definitions */
typedef unsigned      char      BOOLEAN;  //!<  Provide a bool for C
typedef unsigned      char      BYTE;     //!<  Provide a 8-bit unsigned
typedef signed        char      SBYTE;    //!<  Provide a 8-bit signed
typedef unsigned      int       WORD;     //!<  Provide a 16-bit unsigned
typedef signed        int       SWORD;    //!<  Provide a 16-bit signed
typedef unsigned long int       LWORD;    //!<  Provide a 32-bit unsigned
typedef signed   long int       SLWORD;   //!<  Provide a 32-bit signed

// These typedefs are used for 9900 communications
typedef unsigned char int8u;              //!<  Provide a 8-bit unsigned
typedef unsigned int int16u;              //!<  Provide a 16-bit unsigned

/*!
 * \union  fp32
 * A type that easy conversion of FP -> hex
 *
 */
typedef union
{
    float floatVal;
    int8u byteVal[4]; // Allows easy conversion of FP -> hex
} fp32;

/*!
 * \struct  stFuncCtrl
 * Structure enable, disable and get status of a given feature
 *
 */
typedef struct
{
  void (*enable)(void);         //!< Enable feature
  void (*disable)(void);        //!< Disable feature
  BOOLEAN (*isEnabled)(void);   //!< Request feature status
} stFuncCtrl;

/// @cond SKIP_COMMENTS
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

/// @endcond

/* Bit operation on registers */
/*!
 *  \fn SETB(p,m)
 *  \brief  Macro to set the indicated bit of word, byte
 *
 *  \param  p This is the variable to set bit to
 *  \param  m this is the mask indicating the bit to set
 *
 *  The idea of the macro is that if a constant is used as p bit number, the implementation will generate an atomic OR
 *
 */
#define SETB(p,m)   p |= m
/*!
 *  \fn CLEARB(p,m)
 *  \brief  Macro to clear the indicated bit of word, byte
 *
 *  \param  p This is the variable to clear the bit to
 *  \param  m this is the mask indicating the bit to be clear
 *
 *  The idea of the macro is that if a constant is used as p bit number, the implementation will generate an atomic AND
 *
 */

#define CLEARB(p,m) p &= ~m
/*!
 *  \fn TOGGLEB(p,m)
 *  \brief  Macro to toggle the indicated bit of word, byte
 *
 *  \param  p This is the variable to toggle bit to
 *  \param  m this is the mask indicating the bit to toggle
 *
 *  The idea of the macro is that if a constant is used as p bit number, the implementation will generate an atomic XOR
 *
 */

#define TOGGLEB(p,m) p ^= m
/// @cond SKIP_COMMENTS
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
/// @endcond

/* macro expressions */
/*!
 *  \fn DIM(x)
 *  \brief  Macro to provide the DIMENSION (number of elements) of the array
 *
 *  \param  x Ia the array to be dimensioned
 *  \return The number of elements in the array
 *
 *
 */
#define DIM(x)  ((int)(sizeof(x) / sizeof(x[0])))
#define TBD     ;               /* flag to fix something */


#endif /* DEFINE_H_ */
