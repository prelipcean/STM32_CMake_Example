#ifndef PLATFORM_TYPES_H_
#define PLATFORM_TYPES_H_

/**
 * @name    CPU_Types
 * @details Symbols that specify the CPU type according to its own architecture (8, 16, 32 or 64 bits)
 */
/**@{*/
#define CPU_TYPE_8      8  /*!< Define a 8 bit CPU, e.g. PIC18F, ATmega, STM8 etc.. */
#define CPU_TYPE_16     16 /*!< Define a 16 bit CPU, e.g. PIC24F, S12, xMega etc.. */
#define CPU_TYPE_32     32 /*!< Define a 32 bit CPU, e.g. PIC132MX, STM32, TC2xx etc.. */
#define CPU_TYPE_64     64 /*!< Define a 64 bit CPU, e.g. ARMv8, x86 etc.. */
/**@}*/

/**
 * @name    Bit_order
 * @details Symbols to specify the bit order according to the CPU endianess, LSB for little endian and
 *          MSB for big endian
 */
/**@{*/
#define MSB_FIRST       0 /*!< CPU with most significant bit arranged first */
#define LSB_FIRST       1 /*!< CPU with least significant bit arranged first */
/**@}*/

/**
 * @name    Byte_order
 * @details Symbols to specify the endianess types of the CPU
 */
/**@{*/
#define HIGH_BYTE_FIRST 0 /*!< Define a CPU with a big endian architecture */
#define LOW_BYTE_FIRST  1 /*!< Define a CPU with a little endian architecture */
/**@}*/

/**
 * @name    CPU_Definition
 * @details Specifies the CPU data and endianess architecture.
 */
/**@{*/
#define CPU_TYPE        CPU_TYPE_32    /*!< Indicate the CPU type (8,16,32 or 64 bits) */
#define CPU_BIT_ORDER   LSB_FIRST      /*!< CPU bit order (lsb or msb) */
#define CPU_BYTE_ORDER  LOW_BYTE_FIRST /*!< Endianess type little or big */
/**@}*/

/**
 * @name    Boolean_Values
 * @details Symbols to specify true and false values
 */
/**@{*/
#ifndef FALSE
  #define FALSE       (0U == 1U) /*!< Boolean representation of false */
#endif
#ifndef TRUE
  #define TRUE        (1U == 1U) /*!< Boolean representation of true */
#endif
/**@}*/

/**
 * @name   Data_Types
 */
/**@{*/
typedef unsigned char         boolean;        /*!<                0 = FALSE .. 1 = TRUE                   */
typedef unsigned char         uint8;          /*!<                        0 .. 255                        */
typedef signed char           sint8;          /*!<                     -128 .. +127                       */
typedef unsigned short        uint16;         /*!<                        0 .. 65535                      */
typedef signed short          sint16;         /*!<                   -32768 .. +32767                     */
typedef unsigned long         uint32;         /*!<                        0 .. 4294967295                 */
typedef signed long           sint32;         /*!<              -2147483648 .. +2147483647                */
typedef unsigned long long    uint64;         /*!<                        0 .. 18446744073709551615       */
typedef signed long long      sint64;         /*!<     -9223372036854775808 .. 9223372036854775807        */
typedef float                 float32;        /*!<           -3.4028235e+38 .. +3.4028235e+38             */
typedef double                float64;        /*!< -2.2250738585072014e-308 .. +1.7976931348623157e+308   */
/**@}*/

/**
 * @name    Void_Pointers
 * @details Cross platform type definitions to declare void pointers
 */
/**@{*/
typedef void *VoidPtr;                     /*!< Pointer to void */
typedef const void *ConstVoidPtr;          /*!< Pointer to constant void */
/**@}*/

#endif /* PLATFORM_TYPES_H_ */
