/**
 * @file reg_util.h
 * @brief A utility header file for common register and bit manipulation macros
 * in C for embedded systems.
 */

#ifndef REG_UTIL_H_
#define REG_UTIL_H_

#include "std_types.h"

/*===========================================================================*/
/* Utility Macros                                                            */
/*===========================================================================*/

/**
 * @brief Creates a bitmask for the n-th bit.
 * @param n The bit position (0-31).
 */
#define BIT(n) (1U << (n))

/**
 * @brief Creates a bitmask for 'n' bits (e.g., MASK(3) = 0b111 = 0x7).
 * @note This only works for n < 32.
 * @param n The number of bits to mask.
 */
#define MASK(n) (BIT(n) - 1)

/*===========================================================================*/
/* Basic Register Access                                                     */
/*===========================================================================*/

/**
 * @brief Writes a value to a register.
 * @details Assumes 'reg' is a volatile l-value (e.g., *MY_REG).
 * @param reg The register to write to.
 * @param val The value to write.
 */
#define REG_WRITE(reg, val) ((reg) = (val))

/**
 * @brief Reads a value from a register.
 * @details Assumes 'reg' is a volatile l-value.
 * @param reg The register to read from.
 * @return The 32-bit value read from the register.
 */
#define REG_READ(reg) ((reg))

/*===========================================================================*/
/* Single-Bit Manipulation (Read-Modify-Write)                               */
/*===========================================================================*/

/******************************************************************************
 * @brief WARNING: Read-Modify-Write (RMW) Operations
 *
 * The macros in this section (SET_BIT, CLR_BIT, TOGGLE_BIT, WRITE_BIT)
 * perform read-modify-write operations, which are NOT atomic.
 *
 * If the same register is modified by both main-line code and an
 * interrupt service routine (ISR), a race condition can occur, leading to
 * unpredictable behavior.
 *
 * For critical registers, use processor-specific atomic instructions
 * (e.g., LDREX/STREX on ARM) or disable interrupts during the operation.
 *****************************************************************************/

/**
 * @brief Sets a specific bit in a register to 1. (RMW)
 * @param reg The register to modify.
 * @param pos The bit position (0-31) to set.
 */
#define REG_SET_BIT(reg, pos) ((reg) |= BIT(pos))

/**
 * @brief Clears a specific bit in a register to 0. (RMW)
 * @param reg The register to modify.
 * @param pos The bit position (0-31) to clear.
 */
#define REG_CLR_BIT(reg, pos) ((reg) &= ~BIT(pos))

/**
 * @brief Toggles a specific bit in a register. (RMW)
 * @param reg The register to modify.
 * @param pos The bit position (0-31) to toggle.
 */
#define REG_TOGGLE_BIT(reg, pos) ((reg) ^= BIT(pos))

/**
 * @brief Reads a specific bit from a register.
 * @param reg The register to read from.
 * @param pos The bit position (0-31) to read.
 * @return The bit value (1 or 0).
 */
#define REG_READ_BIT(reg, pos) (((reg) >> (pos)) & 1U)

/**
 * @brief Writes a specific value (1 or 0) to a bit. (RMW)
 * @param reg The register to modify.
 * @param pos The bit position (0-31) to write.
 * @param val The value to write (1 or 0).
 */
#define REG_WRITE_BIT(reg, pos, val) \
    ((reg) = ((reg) & ~BIT(pos)) | (((val)&1U) << (pos)))

/*===========================================================================*/
/* Bit-Field Manipulation (Read-Modify-Write)                                */
/*===========================================================================*/

/**
 * @brief Clears a field of bits in a register. (RMW)
 * @param reg The register to modify.
 * @param clrmask The mask of bits to clear (relative to bit 0, e.g., MASK(4)).
 * @param pos The starting bit position of the field.
 */
#define REG_CLR_VAL(reg, clrmask, pos) \
    ((reg) &= ~((clrmask) << (pos)))

/**
 * @brief Sets a value within a bit-field of a register. (RMW)
 * @details This macro first clears the target bit-field and then sets
 * the new value. It is wrapped in a do-while(0) block to
 * act as a single, safe C statement.
 * @param reg The register to modify.
 * @param val The value to write (will be masked).
 * @param setmask The mask for the field (relative to bit 0, e.g., MASK(4)).
 * @param pos The starting bit position of the field.
 */
#define REG_SET_VAL(reg, val, setmask, pos)                           \
    do                                                                \
    {                                                                 \
        (reg) = ((reg) & ~((setmask) << (pos))) | /* Clear field */    \
                (((val) & (setmask)) << (pos));   /* Set new value */ \
    } while (0)

/**
 * @brief Reads a value from a bit-field in a register.
 * @param reg The register to read from.
 * @param rdmask The mask for the field (relative to bit 0, e.g., MASK(4)).
 * @param pos The starting bit position of the field.
 * @return The value of the bit-field (shifted down to bit 0).
 */
#define REG_READ_VAL(reg, rdmask, pos) \
    ((REG_READ(reg) >> (pos)) & (rdmask))

/**
 * @brief Modifies a register using clear and set masks. (RMW)
 * @details This is a generic RMW operation where masks are pre-shifted.
 * @param reg The register to modify.
 * @param clrmask A mask of bits to clear (e.g., GPIO_MODER_MODER5).
 * @param setmask A mask of bits to set (e.g., GPIO_MODER_MODER5_0).
 */
#define REG_MODIFY(reg, clrmask, setmask) \
    ((reg) = ((reg) & ~(clrmask)) | (setmask))

/*===========================================================================*/
/* Polling Macros                                                            */
/*===========================================================================*/

/******************************************************************************
 * @brief WARNING: Polling Loops
 *
 * The macros in this section are blocking and will loop forever
 * if the condition is never met. They are intended for simple
 * driver initialization (e.g., "wait for clock ready").
 *
 * Avoid using these in main application logic. Use a timeout
 * mechanism instead.
 *****************************************************************************/

/**
 * @brief Waits (blocks) until a specific bit is set to 1.
 * @param reg The register to poll.
 * @param pos The bit position to check.
 */
#define WAIT_UNTIL_BIT_SET(reg, pos) \
    while (REG_READ_BIT(reg, pos) == 0) {}

/**
 * @brief Waits (blocks) until a specific bit is cleared to 0.
 * @param reg The register to poll.
 * @param pos The bit position to check.
 */
#define WAIT_UNTIL_BIT_CLR(reg, pos) \
    while (REG_READ_BIT(reg, pos) != 0) {}

/*===========================================================================*/
/* Example GPIO Pin Definitions                                              */
/*===========================================================================*/
#define GPIO_Pin_0 ((uint16)BIT(0))  /* Pin 0 selected */
#define GPIO_Pin_1 ((uint16)BIT(1))  /* Pin 1 selected */
#define GPIO_Pin_2 ((uint16)BIT(2))  /* Pin 2 selected */
#define GPIO_Pin_3 ((uint16)BIT(3))  /* Pin 3 selected */
#define GPIO_Pin_4 ((uint16)BIT(4))  /* Pin 4 selected */
#define GPIO_Pin_5 ((uint16)BIT(5))  /* Pin 5 selected */
#define GPIO_Pin_6 ((uint16)BIT(6))  /* Pin 6 selected */
#define GPIO_Pin_7 ((uint16)BIT(7))  /* Pin 7 selected */
#define GPIO_Pin_8 ((uint16)BIT(8))  /* Pin 8 selected */
#define GPIO_Pin_9 ((uint16)BIT(9))  /* Pin 9 selected */
#define GPIO_Pin_10 ((uint16)BIT(10)) /* Pin 10 selected */
#define GPIO_Pin_11 ((uint16)BIT(11)) /* Pin 11 selected */
#define GPIO_Pin_12 ((uint16)BIT(12)) /* Pin 12 selected */
#define GPIO_Pin_13 ((uint16)BIT(13)) /* Pin 13 selected */
#define GPIO_Pin_14 ((uint16)BIT(14)) /* Pin 14 selected */
#define GPIO_Pin_15 ((uint16)BIT(15)) /* Pin 15 selected */
#define GPIO_Pin_All ((uint16)0xFFFF) /* All pins selected */

#endif /* REG_UTIL_H_ */
