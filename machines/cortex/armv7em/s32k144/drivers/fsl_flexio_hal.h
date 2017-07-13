/*
 * Copyright (c) 2014 - 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FSL_FLEXIO_HAL_H
#define FSL_FLEXIO_HAL_H

#include <stdbool.h>
#include "fsl_device_registers.h"

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.1, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.2, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.4, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 5.5, identifier clash
 * The supported compilers use more than 31 significant characters for identifiers.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower 
 * or different essential type [MISRA 2012 Rule 10.3, required]
 * This is required by the conversion of an unsigned value of a bitfield/bit into an enum value.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 10.5, 
 * Impermissible cast; cannot cast from 'essentially unsigned' to 'essentially Boolean'
 * This is required by the efficient conversion of a bit value into a bool for Get functions.
 * Impermissible cast; cannot cast from 'essentially Boolean' to 'essentially unsigned'
 * This is required by the efficient conversion of a bool into a bit value for Set functions.
 * Impermissible cast; cannot cast from 'essentially unsigned' to 'essentially enum<i>
 * This is required by the efficient conversion of a bit-field value into an enum for Get functions.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite expression 
 *(different essential type categories).
 * This is required by the conversion of a bit/bitfield of a register into boolean or an enum type.
 */

/*!
 * @addtogroup flexio_hal
 * @{
 */

 /*******************************************************************************
 * Definitions
 ******************************************************************************/

 /*! @brief Shift clock polarity options 
  * Implements : flexio_timer_polarity_t_Class 
  */
typedef enum
{
    FLEXIO_TIMER_POLARITY_POSEDGE = 0x00U,     /*!< Shift on positive edge of Shift clock */
    FLEXIO_TIMER_POLARITY_NEGEDGE = 0x01U,     /*!< Shift on negative edge of Shift clock */
} flexio_timer_polarity_t;

 /*! @brief Pin polarity options 
  * Implements : flexio_pin_polarity_t_Class 
  */
typedef enum
{
    FLEXIO_PIN_POLARITY_HIGH = 0x00U,          /*!< Pin is active high */
    FLEXIO_PIN_POLARITY_LOW  = 0x01U,          /*!< Pin is active low  */
} flexio_pin_polarity_t;

 /*! @brief Pin configuration options 
  * Implements : flexio_pin_config_t_Class
  */
typedef enum
{
    FLEXIO_PIN_CONFIG_DISABLED     = 0x00U,    /*!< Shifter pin output disabled */
    FLEXIO_PIN_CONFIG_OPEN_DRAIN   = 0x01U,    /*!< Shifter pin open drain or bidirectional output enable */
    FLEXIO_PIN_CONFIG_BIDIR_OUTPUT = 0x02U,    /*!< Shifter pin bidirectional output data */
    FLEXIO_PIN_CONFIG_OUTPUT       = 0x03U,    /*!< Shifter pin output */
} flexio_pin_config_t;

 /*! @brief Shifter mode options 
  * Implements : flexio_shifter_mode_t_Class
  */
typedef enum
{
    FLEXIO_SHIFTER_MODE_DISABLED          = 0x00U,  /*!< Shifter disabled */
    FLEXIO_SHIFTER_MODE_RECEIVE           = 0x01U,  /*!< Receive mode */
    FLEXIO_SHIFTER_MODE_TRANSMIT          = 0x02U,  /*!< Transmit mode */
    FLEXIO_SHIFTER_MODE_MATCH_STORE       = 0x04U,  /*!< Match Store mode */
    FLEXIO_SHIFTER_MODE_MATCH_CONTINUOUS  = 0x05U,  /*!< Match Continuous mode */
} flexio_shifter_mode_t;

 /*! @brief Shifter input source options 
  * Implements : flexio_shifter_source_t_Class
  */
typedef enum
{
    FLEXIO_SHIFTER_SOURCE_PIN        = 0x00U,  /*!< Input source is selected pin */
    FLEXIO_SHIFTER_SOURCE_SHIFTER    = 0x01U,  /*!< Input source is shifter N+1 output */
} flexio_shifter_source_t;

 /*! @brief Read/Write mode for shifter buffer 
  * Implements : flexio_shifter_buffer_mode_t_Class
  */
typedef enum
{
    FLEXIO_SHIFTER_RW_MODE_NORMAL    = 0x00U,  /*!< Normal shifter buffer read/write */
    FLEXIO_SHIFTER_RW_MODE_BIT_SWAP  = 0x01U,  /*!< Data is bit-swapped */
    FLEXIO_SHIFTER_RW_MODE_BYTE_SWAP = 0x02U,  /*!< Data is byte-swapped */
    FLEXIO_SHIFTER_RW_MODE_BB_SWAP   = 0x03U,  /*!< Data in each byte is bit-swapped */
} flexio_shifter_buffer_mode_t;

 /*! @brief Trigger polarity 
  * Implements : flexio_trigger_polarity_t_Class
  */
typedef enum
{
    FLEXIO_TRIGGER_POLARITY_HIGH = 0x00U,          /*!< Trigger is active high */
    FLEXIO_TRIGGER_POLARITY_LOW  = 0x01U,          /*!< Trigger is active low  */
} flexio_trigger_polarity_t;

 /*! @brief Trigger sources 
  * Implements : flexio_trigger_source_t_Class
  */
typedef enum
{
    FLEXIO_TRIGGER_SOURCE_EXTERNAL  = 0x00U,  /*!< External trigger selected */
    FLEXIO_TRIGGER_SOURCE_INTERNAL  = 0x01U,  /*!< Internal trigger selected */
} flexio_trigger_source_t;

 /*! @brief Timer mode options 
  * Implements : flexio_timer_mode_t_Class
  */
typedef enum
{
    FLEXIO_TIMER_MODE_DISABLED    = 0x00U,  /*!< Timer Disabled. */
    FLEXIO_TIMER_MODE_8BIT_BAUD   = 0x01U,  /*!< Dual 8-bit counters baud/bit mode. */
    FLEXIO_TIMER_MODE_8BIT_PWM    = 0x02U,  /*!< Dual 8-bit counters PWM mode. */
    FLEXIO_TIMER_MODE_16BIT       = 0x03U,  /*!< Single 16-bit counter mode. */
} flexio_timer_mode_t;

 /*! @brief Timer initial output options 
  * Implements : flexio_timer_output_t_Class
  */
typedef enum
{
    FLEXIO_TIMER_INITOUT_ONE          = 0x00U,  /*!< Timer output is logic one when enabled, unaffected by timer reset. */
    FLEXIO_TIMER_INITOUT_ZERO         = 0x01U,  /*!< Timer output is logic zero when enabled, unaffected by timer reset. */
    FLEXIO_TIMER_INITOUT_ONE_RESET    = 0x02U,  /*!< Timer output is logic one when enabled and on timer reset. */
    FLEXIO_TIMER_INITOUT_ZERO_RESET   = 0x03U,  /*!< Timer output is logic zero when enabled and on timer reset. */
} flexio_timer_output_t;

 /*! @brief Timer decrement options 
  * Implements : flexio_timer_decrement_t_Class
  */
typedef enum
{
    FLEXIO_TIMER_DECREMENT_CLK_SHIFT_TMR      = 0x00U,  /*!< Decrement counter on FlexIO clock, Shift clock equals Timer output. */
    FLEXIO_TIMER_DECREMENT_TRG_SHIFT_TMR      = 0x01U,  /*!< Decrement counter on Trigger input (both edges), Shift clock equals Timer output. */
    FLEXIO_TIMER_DECREMENT_PIN_SHIFT_PIN      = 0x02U,  /*!< Decrement counter on Pin input (both edges), Shift clock equals Pin input. */
    FLEXIO_TIMER_DECREMENT_TRG_SHIFT_TRG      = 0x03U,  /*!< Decrement counter on Trigger input (both edges), Shift clock equals Trigger input. */
} flexio_timer_decrement_t;

 /*! @brief Timer reset options 
  * Implements : flexio_timer_reset_t_Class
  */
typedef enum
{
    FLEXIO_TIMER_RESET_NEVER                  = 0x00U,  /*!< Timer never reset. */
    FLEXIO_TIMER_RESET_PIN_OUT                = 0x02U,  /*!< Timer reset on Timer Pin equal to Timer Output. */
    FLEXIO_TIMER_RESET_TRG_OUT                = 0x03U,  /*!< Timer reset on Timer Trigger equal to Timer Output. */
    FLEXIO_TIMER_RESET_PIN_RISING             = 0x04U,  /*!< Timer reset on Timer Pin rising edge. */
    FLEXIO_TIMER_RESET_TRG_RISING             = 0x06U,  /*!< Timer reset on Trigger rising edge. */
    FLEXIO_TIMER_RESET_TRG_BOTH               = 0x07U,  /*!< Timer reset on Trigger rising or falling edge. */
} flexio_timer_reset_t;

 /*! @brief Timer disable options 
  * Implements :  flexio_timer_disable_t_Class
  */
typedef enum
{
    FLEXIO_TIMER_DISABLE_NEVER                = 0x00U,  /*!< Timer never disabled. */
    FLEXIO_TIMER_DISABLE_TIM_DISABLE          = 0x01U,  /*!< Timer disabled on Timer N-1 disable. */
    FLEXIO_TIMER_DISABLE_TIM_CMP              = 0x02U,  /*!< Timer disabled on Timer compare. */
    FLEXIO_TIMER_DISABLE_TIM_CMP_TRG_LOW      = 0x03U,  /*!< Timer disabled on Timer compare and Trigger Low. */
    FLEXIO_TIMER_DISABLE_PIN                  = 0x04U,  /*!< Timer disabled on Pin rising or falling edge. */
    FLEXIO_TIMER_DISABLE_PIN_TRG_HIGH         = 0x05U,  /*!< Timer disabled on Pin rising or falling edge provided Trigger is high. */
    FLEXIO_TIMER_DISABLE_TRG                  = 0x06U,  /*!< Timer disabled on Trigger falling edge. */
} flexio_timer_disable_t;

 /*! @brief Timer disable options 
  * Implements :  flexio_timer_enable_t_Class
  */
typedef enum
{
    FLEXIO_TIMER_ENABLE_ALWAYS                  = 0x00U,  /*!< Timer always enabled. */
    FLEXIO_TIMER_ENABLE_TIM_ENABLE              = 0x01U,  /*!< Timer enabled on Timer N-1 enable. */
    FLEXIO_TIMER_ENABLE_TRG_HIGH                = 0x02U,  /*!< Timer enabled on Trigger high. */
    FLEXIO_TIMER_ENABLE_TRG_PIN_HIGH            = 0x03U,  /*!< Timer enabled on Trigger high and Pin high. */
    FLEXIO_TIMER_ENABLE_PIN_POSEDGE             = 0x04U,  /*!< Timer enabled on Pin rising edge. */
    FLEXIO_TIMER_ENABLE_PIN_POSEDGE_TRG_HIGH    = 0x05U,  /*!< Timer enabled on Pin rising edge and Trigger high. */
    FLEXIO_TIMER_ENABLE_TRG_POSEDGE             = 0x06U,  /*!< Timer enabled on Trigger rising edge. */
    FLEXIO_TIMER_ENABLE_TRG_EDGE                = 0x07U,  /*!< Timer enabled on Trigger rising or falling edge. */
} flexio_timer_enable_t;

 /*! @brief Timer stop bit options 
  * Implements : flexio_timer_stop_t_Class
  */
typedef enum
{
    FLEXIO_TIMER_STOP_BIT_DISABLED              = 0x00U,  /*!< Stop bit disabled. */
    FLEXIO_TIMER_STOP_BIT_TIM_CMP               = 0x01U,  /*!< Stop bit is enabled on timer compare. */
    FLEXIO_TIMER_STOP_BIT_TIM_DIS               = 0x02U,  /*!< Stop bit is enabled on timer disable. */
    FLEXIO_TIMER_STOP_BIT_TIM_CMP_DIS           = 0x03U,  /*!< Stop bit is enabled on timer compare and disable. */
} flexio_timer_stop_t;

 /*! @brief Timer stop bit options - for Transmit, Receive or Match Store modes only 
  * Implements :  flexio_shifter_stop_t_Class
  */
typedef enum
{
    FLEXIO_SHIFTER_STOP_BIT_DISABLED            = 0x00U,  /*!< Stop bit disabled. */
    FLEXIO_SHIFTER_STOP_BIT_0                   = 0x02U,  /*!< Transmit/expect stop bit value 0 */
    FLEXIO_SHIFTER_STOP_BIT_1                   = 0x03U,  /*!< Transmit/expect stop bit value 1 */
} flexio_shifter_stop_t;

 /*! @brief Timer start bit options - for Transmit, Receive or Match Store modes only 
  * Implements :  flexio_shifter_start_t_Class
  */
typedef enum
{
    FLEXIO_SHIFTER_START_BIT_DISABLED       = 0x00U,  /*!< Start bit disabled, transmitter loads data on enable */
    FLEXIO_SHIFTER_START_BIT_DISABLED_SH    = 0x01U,  /*!< Start bit disabled, transmitter loads data on first shift */
    FLEXIO_SHIFTER_START_BIT_0              = 0x02U,  /*!< Transmit/expect start bit value 0 */
    FLEXIO_SHIFTER_START_BIT_1              = 0x03U,  /*!< Transmit/expect start bit value 1 */
} flexio_shifter_start_t;

 /*! @brief Timer start bit options 
  * Implements : flexio_timer_start_t_Class 
  */
typedef enum
{
    FLEXIO_TIMER_START_BIT_DISABLED              = 0x00U,  /*!< Start bit disabled. */
    FLEXIO_TIMER_START_BIT_ENABLED               = 0x01U,  /*!< Start bit enabled. */
} flexio_timer_start_t;

 /*! @brief FlexIO module version number 
  * Implements : flexio_version_info_t_Class
 */
typedef struct
{
    uint8_t  majorNumber;       /**< Major Version Number */
    uint8_t  minorNumber;       /**< Minor Version Number */
    uint16_t featureNumber;     /**< Feature Specification Number */
} flexio_version_info_t;


/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name Template Group
 * @{
 */

#if defined(__cplusplus)
extern "C" {
#endif


/*!
 * @brief Reads the version of the FlexIO module
 *
 * This function reads the version number of the FLEXIO hardware module
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @param[out] versionInfo Device Version Number
  * Implements : FLEXIO_HAL_GetVersion_Activity
 */
static inline void FLEXIO_HAL_GetVersion(const FLEXIO_Type *baseAddr, flexio_version_info_t *versionInfo)
{
    uint32_t tmp = baseAddr->VERID;
    versionInfo->majorNumber = (uint8_t)((tmp & FLEXIO_VERID_MAJOR_MASK) >> FLEXIO_VERID_MAJOR_SHIFT);
    versionInfo->minorNumber = (uint8_t)((tmp & FLEXIO_VERID_MINOR_MASK) >> FLEXIO_VERID_MINOR_SHIFT);
    versionInfo->featureNumber = (uint16_t)((tmp & FLEXIO_VERID_FEATURE_MASK) >> FLEXIO_VERID_FEATURE_SHIFT);
    return;
}


/*!
 * @brief Returns the number of external triggers of the FlexIO module
 *
 * This function returns the number of external triggers of the FlexIO module
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @return  the number of external triggers of the FlexIO module.
 * Implements : FLEXIO_HAL_GetTriggerNum_Activity
 */
static inline uint8_t FLEXIO_HAL_GetTriggerNum(const FLEXIO_Type *baseAddr)
{
    uint32_t tmp = baseAddr->PARAM;
    tmp = (tmp & FLEXIO_PARAM_TRIGGER_MASK) >> FLEXIO_PARAM_TRIGGER_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Returns the number of pins of the FlexIO module
 *
 * This function returns the number of pins of the FlexIO module
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @return  the number of pins of the FlexIO module.
 * Implements : FLEXIO_HAL_GetPinNum_Activity
 */
static inline uint8_t FLEXIO_HAL_GetPinNum(const FLEXIO_Type *baseAddr)
{
    uint32_t tmp = baseAddr->PARAM;
    tmp = (tmp & FLEXIO_PARAM_PIN_MASK) >> FLEXIO_PARAM_PIN_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Returns the number of timers of the FlexIO module
 *
 * This function returns the number of timers of the FlexIO module
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @return  the number of timers of the FlexIO module.
 * Implements : FLEXIO_HAL_GetTimerNum_Activity
 */
static inline uint8_t FLEXIO_HAL_GetTimerNum(const FLEXIO_Type *baseAddr)
{
    uint32_t tmp = baseAddr->PARAM;
    tmp = (tmp & FLEXIO_PARAM_TIMER_MASK) >> FLEXIO_PARAM_TIMER_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Returns the number of shifters of the FlexIO module
 *
 * This function returns the number of shifters of the FlexIO module
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @return  the number of shifters of the FlexIO module.
 * Implements : FLEXIO_HAL_GetShifterNum_Activity
 */
static inline uint8_t FLEXIO_HAL_GetShifterNum(const FLEXIO_Type *baseAddr)
{
    uint32_t tmp = baseAddr->PARAM;
    tmp = (tmp & FLEXIO_PARAM_SHIFTER_MASK) >> FLEXIO_PARAM_SHIFTER_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Returns the current doze mode setting
 *
 * This function returns the current doze mode setting for the FlexIO module
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @return  The current doze mode setting.
 * Implements : FLEXIO_HAL_GetDozeMode_Activity
 */
static inline bool FLEXIO_HAL_GetDozeMode(const FLEXIO_Type *baseAddr)
{
    uint32_t regValue = (uint32_t)baseAddr->CTRL;
    regValue = (regValue & FLEXIO_CTRL_DOZEN_MASK) >> FLEXIO_CTRL_DOZEN_SHIFT;
    return (bool)regValue;
}


/*!
 * @brief Set the FlexIO module behavior in doze mode
 *
 * This function enables of disables FlexIO functionality in doze mode.
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @param[in] enable  Specifies whether to enable or disable FlexIO in doze mode
 * Implements : FLEXIO_HAL_SetDozeMode_Activity
 */
static inline void FLEXIO_HAL_SetDozeMode(FLEXIO_Type *baseAddr, bool enable)
{
    uint32_t regValue = (uint32_t)baseAddr->CTRL;
    regValue &= (uint32_t)(~(FLEXIO_CTRL_DOZEN_MASK));
    regValue |= FLEXIO_CTRL_DOZEN(enable);
    baseAddr->CTRL = (uint32_t)regValue;
}


/*!
 * @brief Returns the current debug mode setting
 *
 * This function returns the current debug mode setting for the FlexIO module
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @return  The current debug mode setting.
 * Implements : FLEXIO_HAL_GetDebugMode_Activity
 */
static inline bool FLEXIO_HAL_GetDebugMode(const FLEXIO_Type *baseAddr)
{
    uint32_t regValue = (uint32_t)baseAddr->CTRL;
    regValue = (regValue & FLEXIO_CTRL_DBGE_MASK) >> FLEXIO_CTRL_DBGE_SHIFT;
    return (bool)regValue;
}


/*!
 * @brief Set the FlexIO module behavior in debug mode
 *
 * This function enables of disables FlexIO functionality in debug mode.
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @param[in] enable  Specifies whether to enable or disable FlexIO in debug mode
 * Implements : FLEXIO_HAL_SetDebugMode_Activity
 */
static inline void FLEXIO_HAL_SetDebugMode(FLEXIO_Type *baseAddr, bool enable)
{
    uint32_t regValue = (uint32_t)baseAddr->CTRL;
    regValue &= (uint32_t)(~(FLEXIO_CTRL_DBGE_MASK));
    regValue |= FLEXIO_CTRL_DBGE(enable);
    baseAddr->CTRL = (uint32_t)regValue;
}


/*!
 * @brief Returns the current fast access setting
 *
 * This function returns the current fast access setting for the FlexIO module.
 * Fast access allows faster accesses to FlexIO registers, but requires the FlexIO
 * clock to be at least twice the frequency of the bus clock.
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @return  The current fast access setting.
 * Implements : FLEXIO_HAL_GetFastAccess_Activity
 */
static inline bool FLEXIO_HAL_GetFastAccess(const FLEXIO_Type *baseAddr)
{
    uint32_t regValue = (uint32_t)baseAddr->CTRL;
    regValue = (regValue & FLEXIO_CTRL_FASTACC_MASK) >> FLEXIO_CTRL_FASTACC_SHIFT;
    return (bool)regValue;
}


/*!
 * @brief Configure the FlexIO fast access feature.
 *
 * This function enables of disables the fast access feature for the FlexIO module.
 * Fast access allows faster accesses to FlexIO registers, but requires the FlexIO
 * clock to be at least twice the frequency of the bus clock.
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @param[in] enable Enables fast register accesses to FlexIO registers
 * Implements : FLEXIO_HAL_SetFastAccess_Activity
 */
static inline void FLEXIO_HAL_SetFastAccess(FLEXIO_Type *baseAddr, bool enable)
{
    uint32_t regValue = (uint32_t)baseAddr->CTRL;
    regValue &= (uint32_t)(~(FLEXIO_CTRL_FASTACC_MASK));
    regValue |= FLEXIO_CTRL_FASTACC(enable);
    baseAddr->CTRL = (uint32_t)regValue;
}


/*!
 * @brief Returns the current software reset state
 *
 * This function returns the state of the FlexIO software reset bit.
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @return  The current software reset setting.
 * Implements : FLEXIO_HAL_GetSoftwareReset_Activity
 */
static inline bool FLEXIO_HAL_GetSoftwareReset(const FLEXIO_Type *baseAddr)
{
    uint32_t regValue = (uint32_t)baseAddr->CTRL;
    regValue = (regValue & FLEXIO_CTRL_SWRST_MASK) >> FLEXIO_CTRL_SWRST_SHIFT;
    return (bool)regValue;
}


/*!
 * @brief Set/clear the FlexIO reset command
 *
 * Calling this function with enable parameter set to true resets all internal
 * master logic and registers, except the FlexIO Control Register. The reset state
 * persists until this function is called with enable parameter set to false.
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @param[in] enable Specifies the reset state of the FlexIO logic
 * Implements : FLEXIO_HAL_SetSoftwareReset_Activity
 */
static inline void FLEXIO_HAL_SetSoftwareReset(FLEXIO_Type *baseAddr, bool enable)
{
    uint32_t regValue = (uint32_t)baseAddr->CTRL;
    regValue &= (uint32_t)(~(FLEXIO_CTRL_SWRST_MASK));
    regValue |= FLEXIO_CTRL_SWRST(enable);
    baseAddr->CTRL = (uint32_t)regValue;
}


/*!
 * @brief Returns the current enable/disable setting of the FlexIO
 *
 * This function checks whether or not the FlexIO module is enabled.
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @return  The current enable/disable setting.
 * Implements : FLEXIO_HAL_GetEnable_Activity
 */
static inline bool FLEXIO_HAL_GetEnable(const FLEXIO_Type *baseAddr)
{
    uint32_t regValue = (uint32_t)baseAddr->CTRL;
    regValue = (regValue & FLEXIO_CTRL_FLEXEN_MASK) >> FLEXIO_CTRL_FLEXEN_SHIFT;
    return (bool)regValue;
}


/*!
 * @brief Enables of disables the FlexIO module
 *
 * This function enables or disables the FlexIO module.
 *
 * @param[in] baseAddr Base address of the FlexIO module
 * @param[in] enable Specifies the enable/disable state of the FlexIO module
 * Implements : FLEXIO_HAL_SetEnable_Activity
 */
static inline void FLEXIO_HAL_SetEnable(FLEXIO_Type *baseAddr, bool enable)
{
    uint32_t regValue = (uint32_t)baseAddr->CTRL;
    regValue &= (uint32_t)(~(FLEXIO_CTRL_FLEXEN_MASK));
    regValue |= FLEXIO_CTRL_FLEXEN(enable);
    baseAddr->CTRL = (uint32_t)regValue;
}


/*!
 * @brief Returns the current input data read from the FlexIO pins
 *
 * This function returns the data read from all the FLEXIO pins. Only the lower
 * n bits are valid, where n is the number of pins returned by FLEXIO_HAL_GetPinNum().
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @return  Data read from the FlexIO pins.
 * Implements : FLEXIO_HAL_GetPinData_Activity
 */
static inline uint8_t FLEXIO_HAL_GetPinData(const FLEXIO_Type *baseAddr)
{
    uint32_t tmp = baseAddr->PIN;
    tmp = (tmp & FLEXIO_PIN_PDI_MASK) >> FLEXIO_PIN_PDI_SHIFT;
    return (uint8_t)tmp;
}


/*!
 * @brief Returns the current status of the specified shifter.
 *
 * This function returns the value of the status flag for the specified shifter.
 * The meaning of the status flag depends on the current mode.
 *  - Transmit mode: shifter buffer is empty and ready to accept more data
 *  - Receive mode: shifter buffer is full and received data can be read from it
 *  - Match Store mode: match occurred between shifter buffer and shifter
 *  - Match Continuous mode: current match result between shifter buffer and shifter
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @return  Status of the specified shifter.
 * Implements : FLEXIO_HAL_GetShifterStatus_Activity
 */
static inline bool FLEXIO_HAL_GetShifterStatus(const FLEXIO_Type *baseAddr, uint8_t shifter)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    uint32_t regValue = (uint32_t)baseAddr->SHIFTSTAT;
    regValue = (regValue >> shifter) & 1U;
    return (bool)regValue;
}


/*!
 * @brief Returns the current status flags for all shifters.
 *
 * This function returns the value of the status flags for all shifters. Each bit in 
 * the returned value specifies the status for one shifter, starting with 
 * shifter 0 from least significant bit.
 * The meaning of the status flag depends on the current mode.
 *  - Transmit mode: shifter buffer is empty and ready to accept more data
 *  - Receive mode: shifter buffer is full and received data can be read from it
 *  - Match Store mode: match occurred between shifter buffer and shifter
 *  - Match Continuous mode: current match result between shifter buffer and shifter
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @return  Status of the shifters.
 * Implements : FLEXIO_HAL_GetAllShifterStatus_Activity
 */
static inline uint32_t FLEXIO_HAL_GetAllShifterStatus(const FLEXIO_Type *baseAddr)
{
    return baseAddr->SHIFTSTAT;
}


/*!
 * @brief Clears the status of the specified shifter.
 *
 * This function clears the status flag for the specified shifter. This is possible in
 * all modes except Match Continuous mode.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * Implements : FLEXIO_HAL_ClearShifterStatus_Activity
 */
static inline void FLEXIO_HAL_ClearShifterStatus(FLEXIO_Type *baseAddr, uint8_t shifter)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    baseAddr->SHIFTSTAT = 1UL << shifter;
}


/*!
 * @brief Returns the current error status of the specified shifter.
 *
 * This function returns the value of the error status flag for the specified shifter.
 * The meaning of the error status flag depends on the current mode.
 *  - Transmit mode: shifter buffer was not written before it was transferred in the shifter (buffer overrun)
 *  - Receive mode: shifter buffer was not read before new data was transferred from the shifter (buffer underrun)
 *  - Match Store mode: match event occurred before the previous match data was read from shifter buffer (buffer overrun)
 *  - Match Continuous mode: match occurred between shifter buffer and shifter
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @return  Error status of the specified shifter.
 * Implements : FLEXIO_HAL_GetShifterErrorStatus_Activity
 */
static inline bool FLEXIO_HAL_GetShifterErrorStatus(const FLEXIO_Type *baseAddr, uint8_t shifter)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    uint32_t regValue = (uint32_t)baseAddr->SHIFTERR;
    regValue = (regValue >> shifter) & 1U;
    return (bool)regValue;
}


/*!
 * @brief Returns the current error status for all shifters.
 *
 * This function returns the value of the error status flags for all shifters. Each bit in 
 * the returned value specifies the error status for one shifter, starting with 
 * shifter 0 from least significant bit.
 * The meaning of the error status flag depends on the current mode.
 *  - Transmit mode: shifter buffer was not written before it was transferred in the shifter (buffer overrun)
 *  - Receive mode: shifter buffer was not read before new data was transferred from the shifter (buffer underrun)
 *  - Match Store mode: match event occurred before the previous match data was read from shifter buffer (buffer overrun)
 *  - Match Continuous mode: match occurred between shifter buffer and shifter
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @return  Error status of the shifters.
 * Implements : FLEXIO_HAL_GetAllShifterErrorStatus_Activity
 */
static inline uint32_t FLEXIO_HAL_GetAllShifterErrorStatus(const FLEXIO_Type *baseAddr)
{
    return baseAddr->SHIFTERR;
}


/*!
 * @brief Clears the error status of the specified shifter.
 *
 * This function clears the error status flag for the specified shifter.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * Implements : FLEXIO_HAL_ClearShifterErrorStatus_Activity
 */
static inline void FLEXIO_HAL_ClearShifterErrorStatus(FLEXIO_Type *baseAddr, uint8_t shifter)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    baseAddr->SHIFTERR = 1UL << shifter;
}


/*!
 * @brief Returns the current status of the specified timer.
 *
 * This function returns the value of the status flag for the specified timer.
 * The meaning of the status flag depends on the current mode.
 *  - 8-bit counter mode: the timer status flag is set when the upper 8-bit counter equals zero
 * and decrements. This also causes the counter to reload with the value in the compare register.
 *  - 8-bit PWM mode: the upper 8-bit counter equals zero and decrements. This also causes the
 * counter to reload with the value in the compare register.
 *  - 16-bit counter mode: the 16-bit counter equals zero and decrements. This also causes the
 * counter to reload with the value in the compare register.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @return  Status of the specified timer.
 * Implements : FLEXIO_HAL_GetTimerStatus_Activity
 */
static inline bool FLEXIO_HAL_GetTimerStatus(const FLEXIO_Type *baseAddr, uint8_t timer)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t regValue = (uint32_t)baseAddr->TIMSTAT;
    regValue = (regValue >> timer) & 1U;
    return (bool)regValue;
}


/*!
 * @brief Returns the current status of all timers.
 *
 * This function returns the value of the status flags for all timers. Each bit in 
 * the returned value specifies the status for one timer, starting with 
 * timer 0 from least significant bit.
 * The meaning of the status flag depends on the current mode.
 *  - 8-bit counter mode: the timer status flag is set when the upper 8-bit counter equals zero
 * and decrements. This also causes the counter to reload with the value in the compare register.
 *  - 8-bit PWM mode: the upper 8-bit counter equals zero and decrements. This also causes the
 * counter to reload with the value in the compare register.
 *  - 16-bit counter mode: the 16-bit counter equals zero and decrements. This also causes the
 * counter to reload with the value in the compare register.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @return  Status of the timers.
 * Implements : FLEXIO_HAL_GetAllTimerStatus_Activity
 */
static inline uint32_t FLEXIO_HAL_GetAllTimerStatus(const FLEXIO_Type *baseAddr)
{
    return baseAddr->TIMSTAT;
}


/*!
 * @brief Clears the status of the specified timer.
 *
 * This function clears the status flag for the specified timer.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * Implements : FLEXIO_HAL_ClearTimerStatus_Activity
 */
static inline void FLEXIO_HAL_ClearTimerStatus(FLEXIO_Type *baseAddr, uint8_t timer)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    baseAddr->TIMSTAT = 1UL << timer;
}


/*!
 * @brief Returns the current status of the shifter interrupts.
 *
 * Returns the state of the interrupt for the specified shifter.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] interruptNo Number of the interrupt (shifter).
 * @return  Status of the specified interrupt.
 * Implements : FLEXIO_HAL_GetShifterInterrupt_Activity
 */
static inline bool FLEXIO_HAL_GetShifterInterrupt(const FLEXIO_Type *baseAddr, uint8_t interruptNo)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(interruptNo < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    uint32_t regValue = (uint32_t)baseAddr->SHIFTSIEN;
    regValue = (regValue >> interruptNo) & 1U;
    return (bool)regValue;
}


/*!
 * @brief Returns the current status of all the shifter interrupts.
 *
 * Returns the state of the interrupt for all shifters. Each bit in 
 * the returned value specifies the interrupt state for one shifter, starting with 
 * shifter 0 from least significant bit.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @return  Status of the interrupts.
 * Implements : FLEXIO_HAL_GetAllShifterInterrupt_Activity
 */
static inline uint32_t FLEXIO_HAL_GetAllShifterInterrupt(const FLEXIO_Type *baseAddr)
{
    return baseAddr->SHIFTSIEN;
}


/*!
 * @brief Enables or disables the shifter interrupts.
 *
 * Enable or disable specified shifter interrupts. The interrupt mask must contain a
 * bit of 1 for each shifter who's interrupt must be enabled or disabled.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] interruptMask Mask of the interrupts to be enabled/disabled.
 * @param[in] enable Specifies whether to enable or disable specified interrupts.
 * Implements : FLEXIO_HAL_SetShifterInterrupt_Activity
 */
static inline void FLEXIO_HAL_SetShifterInterrupt(FLEXIO_Type *baseAddr, uint8_t interruptMask, bool enable)
{
    uint32_t tmp;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT((interruptMask & -(1U << FLEXIO_HAL_GetShifterNum(baseAddr))) == 0U);
#endif

    tmp = baseAddr->SHIFTSIEN;

    if (enable == true)
    {
        tmp |= interruptMask;
    }
    else
    {
        tmp &= ~interruptMask;
    }
    baseAddr->SHIFTSIEN = tmp;
}


/*!
 * @brief Returns the current status of the shifter error interrupts.
 *
 * Returns the state of the error interrupt for the specified shifter.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] interruptNo Number of the interrupt.
 * @return  Status of the specified interrupt.
 * Implements : FLEXIO_HAL_GetShifterErrorInterrupt_Activity
 */
static inline bool FLEXIO_HAL_GetShifterErrorInterrupt(const FLEXIO_Type *baseAddr, uint8_t interruptNo)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(interruptNo < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    uint32_t regValue = (uint32_t)baseAddr->SHIFTEIEN;
    regValue = (regValue >> interruptNo) & 1U;
    return (bool)regValue;
}


/*!
 * @brief Returns the current status of all the shifter error interrupts.
 *
 * Returns the state of the error interrupt for all shifters. Each bit in 
 * the returned value specifies the interrupt state for one shifter, starting with 
 * shifter 0 from least significant bit.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @return  Status of the interrupts.
 * Implements : FLEXIO_HAL_GetAllShifterErrorInterrupt_Activity
 */
static inline uint32_t FLEXIO_HAL_GetAllShifterErrorInterrupt(const FLEXIO_Type *baseAddr)
{
    return baseAddr->SHIFTEIEN;
}


/*!
 * @brief Enables or disables the shifter error interrupts.
 *
 * Enable or disable specified shifter error interrupts. The interrupt mask must contain a
 * bit of 1 for each shifter who's error interrupt must be enabled or disabled.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] interruptMask Mask of the interrupts to be enabled/disabled.
 * @param[in] enable Specifies whether to enable or disable specified interrupts.
 * Implements : FLEXIO_HAL_SetShifterErrorInterrupt_Activity
 */
static inline void FLEXIO_HAL_SetShifterErrorInterrupt(FLEXIO_Type *baseAddr, uint8_t interruptMask, bool enable)
{
    uint32_t tmp;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT((interruptMask & ~((1U << FLEXIO_HAL_GetShifterNum(baseAddr)) - 1U)) == 0U);
#endif

    tmp = baseAddr->SHIFTEIEN;

    if (enable == true)
    {
        tmp |= interruptMask;
    }
    else
    {
        tmp &= ~interruptMask;
    }
    baseAddr->SHIFTEIEN = tmp;
}


/*!
 * @brief Returns the current status of the timer interrupts.
 *
 * Returns the state of the interrupt for the specified timer.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] interruptNo Number of the interrupt.
 * @return  Status of the specified interrupt.
 * Implements : FLEXIO_HAL_GetTimerInterrupt_Activity
 */
static inline bool FLEXIO_HAL_GetTimerInterrupt(const FLEXIO_Type *baseAddr, uint8_t interruptNo)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(interruptNo < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t regValue = (uint32_t)baseAddr->TIMIEN;
    regValue = (regValue >> interruptNo) & 1U;
    return (bool)regValue;
}


/*!
 * @brief Returns the current status of all the timer interrupts.
 *
 * Returns the state of the interrupt for all timers. Each bit in 
 * the returned value specifies the interrupt state for one timer, starting with 
 * timer 0 from least significant bit.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @return  Status of the interrupts.
 * Implements : FLEXIO_HAL_GetAllTimerInterrupt_Activity
 */
static inline uint32_t FLEXIO_HAL_GetAllTimerInterrupt(const FLEXIO_Type *baseAddr)
{
    return baseAddr->TIMIEN;
}


/*!
 * @brief Enables or disables the timer interrupts.
 *
 * Enable or disable specified timer interrupts. The interrupt mask must contain a
 * bit of 1 for each timer who's interrupt must be enabled or disabled.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] interruptMask Mask of the interrupts to be enabled/disabled.
 * @param[in] enable Specifies whether to enable or disable specified interrupts.
 * Implements : FLEXIO_HAL_SetTimerInterrupt_Activity
 */
static inline void FLEXIO_HAL_SetTimerInterrupt(FLEXIO_Type *baseAddr, uint8_t interruptMask, bool enable)
{
    uint32_t tmp;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT((interruptMask & -(1U << FLEXIO_HAL_GetTimerNum(baseAddr))) == 0U);
#endif

    tmp = baseAddr->TIMIEN;

    if (enable == true)
    {
        tmp |= interruptMask;
    }
    else
    {
        tmp &= ~interruptMask;
    }
    baseAddr->TIMIEN = tmp;
}


/*!
 * @brief Returns the current status of the shifter DMA requests.
 *
 * Returns the state of the DMA request for the specified shifter.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] requestNo Number of the DMA request.
 * @return  Status of the specified DMA request.
 * Implements : FLEXIO_HAL_GetShifterDMARequest_Activity
 */
static inline bool FLEXIO_HAL_GetShifterDMARequest(const FLEXIO_Type *baseAddr, uint8_t requestNo)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(requestNo < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    uint32_t regValue = (uint32_t)baseAddr->SHIFTSDEN;
    regValue = (regValue >> requestNo) & 1U;
    return (bool)regValue;
}


/*!
 * @brief Enables or disables the shifter DMA requests.
 *
 * Enable or disable specified shifter DMA requests. The request mask must contain a
 * bit of 1 for each shifter who's DMA requests must be enabled or disabled.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] requestMask Mask of the DMA requests to be enabled/disabled.
 * @param[in] enable Specifies whether to enable or disable specified DMA requests.
 * Implements : FLEXIO_HAL_SetShifterDMARequest_Activity
 */
static inline void FLEXIO_HAL_SetShifterDMARequest(FLEXIO_Type *baseAddr, uint8_t requestMask, bool enable)
{
    uint32_t tmp;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT((requestMask & ~((1U << FLEXIO_HAL_GetShifterNum(baseAddr)) - 1U)) == 0U);
#endif

    tmp = baseAddr->SHIFTSDEN;

    if (enable == true)
    {
        tmp |= requestMask;
    }
    else
    {
        tmp &= ~requestMask;
    }
    baseAddr->SHIFTSDEN = tmp;
}


/*!
 * @brief Returns the timer currently assigned to control the specified shifter.
 *
 * This function returns the timer currently assigned to control the specified shifter, and also its polarity.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @param[out] timer Number of the timer.
 * @param[out] polarity Polarity of the timer.
 * Implements : FLEXIO_HAL_GetShifterTimer_Activity
 */
static inline void FLEXIO_HAL_GetShifterTimer(const FLEXIO_Type *baseAddr,
                                              uint8_t shifter,
                                              uint8_t *timer,
                                              flexio_timer_polarity_t *polarity)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->SHIFTCTL[shifter];
    *timer = (uint8_t)((tmp & FLEXIO_SHIFTCTL_TIMSEL_MASK) >> FLEXIO_SHIFTCTL_TIMSEL_SHIFT);
    tmp = (tmp & FLEXIO_SHIFTCTL_TIMPOL_MASK) >> FLEXIO_SHIFTCTL_TIMPOL_SHIFT;
    *polarity = (flexio_timer_polarity_t)tmp;
}


/*!
 * @brief Assigns the specified timer to control the specified shifter.
 *
 * This function assigns a timer to control the specified shifter, and also configures its polarity.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @param[in] timer Number of the timer.
 * @param[in] polarity Polarity of the timer.
 * Implements : FLEXIO_HAL_SetShifterTimer_Activity
 */
static inline void FLEXIO_HAL_SetShifterTimer(FLEXIO_Type *baseAddr,
                                              uint8_t shifter,
                                              uint8_t timer,
                                              flexio_timer_polarity_t polarity)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->SHIFTCTL[shifter];
    tmp &= ~(FLEXIO_SHIFTCTL_TIMSEL_MASK);
    tmp |= FLEXIO_SHIFTCTL_TIMSEL(timer);
    tmp &= ~(FLEXIO_SHIFTCTL_TIMPOL_MASK);
    tmp |= FLEXIO_SHIFTCTL_TIMPOL(polarity);
    baseAddr->SHIFTCTL[shifter] = tmp;
}


/*!
 * @brief Returns the pin currently assigned to the specified shifter, and its configured settings.
 *
 * This function returns the pin currently assigned to the specified shifter, and also its
 * polarity and configuration.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @param[out] pin Number of the pin.
 * @param[out] polarity Polarity of the pin.
 * @param[out] config Pin configuration.
 * Implements : FLEXIO_HAL_GetShifterPin_Activity
 */
static inline void FLEXIO_HAL_GetShifterPin(const FLEXIO_Type *baseAddr,
                                            uint8_t shifter,
                                            uint8_t *pin,
                                            flexio_pin_polarity_t *polarity,
                                            flexio_pin_config_t *config)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->SHIFTCTL[shifter];
    *pin = (uint8_t)((tmp & FLEXIO_SHIFTCTL_PINSEL_MASK) >> FLEXIO_SHIFTCTL_PINSEL_SHIFT);
    *polarity = (flexio_pin_polarity_t)((tmp & FLEXIO_SHIFTCTL_PINPOL_MASK) >> FLEXIO_SHIFTCTL_PINPOL_SHIFT);
    *config = (flexio_pin_config_t)((tmp & FLEXIO_SHIFTCTL_PINCFG_MASK) >> FLEXIO_SHIFTCTL_PINCFG_SHIFT);
}


/*!
 * @brief Assigns the specified pin to the specified shifter.
 *
 * This function assigns the specified pin to the specified shifter, and also sets
 * its polarity and configuration.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @param[in] pin Number of the pin.
 * @param[in] polarity Polarity of the pin.
 * @param[in] config Pin configuration.
 * Implements : FLEXIO_HAL_SetShifterPin_Activity
 */
static inline void FLEXIO_HAL_SetShifterPin(FLEXIO_Type *baseAddr,
                                            uint8_t shifter,
                                            uint8_t pin,
                                            flexio_pin_polarity_t polarity,
                                            flexio_pin_config_t config)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
    DEV_ASSERT(pin < FLEXIO_HAL_GetPinNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->SHIFTCTL[shifter];
    tmp &= ~(FLEXIO_SHIFTCTL_PINSEL_MASK);
    tmp |= FLEXIO_SHIFTCTL_PINSEL(pin);
    tmp &= ~(FLEXIO_SHIFTCTL_PINPOL_MASK);
    tmp |= FLEXIO_SHIFTCTL_PINPOL(polarity);
    tmp &= ~(FLEXIO_SHIFTCTL_PINCFG_MASK);
    tmp |= FLEXIO_SHIFTCTL_PINCFG(config);
    baseAddr->SHIFTCTL[shifter] = tmp;
}


/*!
 * @brief Returns the mode of the specified shifter.
 *
 * This function returns the currently configured mode for the specified shifter.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @return Mode assigned to the shifter.
 * Implements : FLEXIO_HAL_GetShifterMode_Activity
 */
static inline flexio_shifter_mode_t FLEXIO_HAL_GetShifterMode(const FLEXIO_Type *baseAddr, uint8_t shifter)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->SHIFTCTL[shifter];
    tmp = (tmp & FLEXIO_SHIFTCTL_SMOD_MASK) >> FLEXIO_SHIFTCTL_SMOD_SHIFT;
    return (flexio_shifter_mode_t)tmp;
}


/*!
 * @brief Sets the mode of the specified shifter.
 *
 * This function configures the mode for the specified shifter.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @param[in] mode Mode assigned to the shifter.
 * Implements : FLEXIO_HAL_SetShifterMode_Activity
 */
static inline void FLEXIO_HAL_SetShifterMode(FLEXIO_Type *baseAddr,
                                             uint8_t shifter,
                                             flexio_shifter_mode_t mode)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->SHIFTCTL[shifter];
    tmp &= ~(FLEXIO_SHIFTCTL_SMOD_MASK);
    tmp |= FLEXIO_SHIFTCTL_SMOD(mode);
    baseAddr->SHIFTCTL[shifter] = tmp;
}


/*!
 * @brief Sets all control settings for the specified shifter.
 *
 * This function configures the control settings for the specified shifter: mode settings, 
 * pin settings and timer settings. It sums up all settings 
 * from FLEXIO_HAL_SetShifterMode(), FLEXIO_HAL_SetShifterPin() and FLEXIO_HAL_SetShifterTimer()
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @param[in] mode Mode assigned to the shifter.
 * @param[in] pin Number of the pin.
 * @param[in] pinPolarity Polarity of the pin.
 * @param[in] pinConfig Pin configuration.
 * @param[in] timer Number of the timer.
 * @param[in] timerPolarity Polarity of the timer.
 * Implements : FLEXIO_HAL_SetShifterControl_Activity
 */
static inline void FLEXIO_HAL_SetShifterControl(FLEXIO_Type *baseAddr,
                                                uint8_t shifter,
                                                flexio_shifter_mode_t mode,
                                                uint8_t pin,
                                                flexio_pin_polarity_t pinPolarity,
                                                flexio_pin_config_t pinConfig,
                                                uint8_t timer,
                                                flexio_timer_polarity_t timerPolarity)
{
    baseAddr->SHIFTCTL[shifter] = FLEXIO_SHIFTCTL_SMOD(mode)
                                | FLEXIO_SHIFTCTL_PINPOL(pinPolarity)
                                | FLEXIO_SHIFTCTL_PINSEL(pin)
                                | FLEXIO_SHIFTCTL_PINCFG(pinConfig)
                                | FLEXIO_SHIFTCTL_TIMPOL(timerPolarity)
                                | FLEXIO_SHIFTCTL_TIMSEL(timer);
}


/*!
 * @brief Returns the input source of the specified shifter.
 *
 * This function returns the input source of the specified shifter.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @return Input source selected for the shifter.
 * Implements : FLEXIO_HAL_GetShifterInputSource_Activity
 */
static inline flexio_shifter_source_t FLEXIO_HAL_GetShifterInputSource(const FLEXIO_Type *baseAddr, uint8_t shifter)
{
    uint32_t tmp;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    tmp = baseAddr->SHIFTCFG[shifter];
    tmp = (tmp & FLEXIO_SHIFTCFG_INSRC_MASK) >> FLEXIO_SHIFTCFG_INSRC_SHIFT;
    return (flexio_shifter_source_t)tmp;
}


/*!
 * @brief Configures the input source of the specified shifter.
 *
 * This function configures the input source of the specified shifter.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @param[in] source Input source selected for the shifter.
 * Implements : FLEXIO_HAL_SetShifterInputSource_Activity
 */
static inline void FLEXIO_HAL_SetShifterInputSource(FLEXIO_Type *baseAddr,
                                                    uint8_t shifter,
                                                    flexio_shifter_source_t source)
{
    uint32_t tmp;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    tmp = baseAddr->SHIFTCFG[shifter];
    tmp &= ~(FLEXIO_SHIFTCFG_INSRC_MASK);
    tmp |= FLEXIO_SHIFTCFG_INSRC(source);
    baseAddr->SHIFTCFG[shifter] = tmp;
}


/*!
 * @brief Returns the stop bit configuration for the specified shifter.
 *
 * This function returns the current configuration for sending or receiving
 * a stop bit in Transmit, Receive or Match Store modes.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @return Stop bit configuration for the shifter.
 * Implements : FLEXIO_HAL_GetShifterStopBit_Activity
 */
static inline flexio_shifter_stop_t FLEXIO_HAL_GetShifterStopBit(const FLEXIO_Type *baseAddr, uint8_t shifter)
{
    uint32_t tmp;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    tmp = baseAddr->SHIFTCFG[shifter];
    tmp = (tmp & FLEXIO_SHIFTCFG_SSTOP_MASK) >> FLEXIO_SHIFTCFG_SSTOP_SHIFT;
    return (flexio_shifter_stop_t)tmp;
}


/*!
 * @brief Configures the stop bit of the specified shifter.
 *
 * This function configures the sending or receiving of
 * a stop bit in Transmit, Receive or Match Store modes.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @param[in] stop Stop bit configuration for the shifter.
 * Implements : FLEXIO_HAL_SetShifterStopBit_Activity
 */
static inline void FLEXIO_HAL_SetShifterStopBit(FLEXIO_Type *baseAddr, uint8_t shifter, flexio_shifter_stop_t stop)
{
    uint32_t tmp;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    tmp = baseAddr->SHIFTCFG[shifter];
    tmp &= ~(FLEXIO_SHIFTCFG_SSTOP_MASK);
    tmp |= FLEXIO_SHIFTCFG_SSTOP(stop);
    baseAddr->SHIFTCFG[shifter] = tmp;
}


/*!
 * @brief Returns the start bit configuration for the specified shifter.
 *
 * This function returns the current configuration for sending or receiving
 * a start bit in Transmit, Receive or Match Store modes.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @return Start bit configuration for the shifter.
 * Implements : FLEXIO_HAL_GetShifterStartBit_Activity
 */
static inline flexio_shifter_start_t FLEXIO_HAL_GetShifterStartBit(const FLEXIO_Type *baseAddr, uint8_t shifter)
{
    uint32_t tmp;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    tmp = baseAddr->SHIFTCFG[shifter];
    tmp = (tmp & FLEXIO_SHIFTCFG_SSTART_MASK) >> FLEXIO_SHIFTCFG_SSTART_SHIFT;
    return (flexio_shifter_start_t)tmp;
}


/*!
 * @brief Configures the start bit of the specified shifter.
 *
 * This function configures the sending or receiving of
 * a start bit in Transmit, Receive or Match Store modes.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @param[in] start Start bit configuration for the shifter.
 * Implements : FLEXIO_HAL_SetShifterStartBit_Activity
 */
static inline void FLEXIO_HAL_SetShifterStartBit(FLEXIO_Type *baseAddr,
                                                 uint8_t shifter,
                                                 flexio_shifter_start_t start)
{
    uint32_t tmp;
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    tmp = baseAddr->SHIFTCFG[shifter];
    tmp &= ~(FLEXIO_SHIFTCFG_SSTART_MASK);
    tmp |= FLEXIO_SHIFTCFG_SSTART(start);
    baseAddr->SHIFTCFG[shifter] = tmp;
}


/*!
 * @brief Sets all configuration settings for specified shifter.
 *
 * This function sets the following configurations for the specified shifter:
 * start bit, stop bit, input source
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @param[in] start Start bit configuration for the shifter.
 * @param[in] stop Stop bit configuration for the shifter.
 * @param[in] source Input source selected for the shifter.
 * Implements : FLEXIO_HAL_SetShifterConfig_Activity
 */
static inline void FLEXIO_HAL_SetShifterConfig(FLEXIO_Type *baseAddr,
                                               uint8_t shifter,
                                               flexio_shifter_start_t start,
                                               flexio_shifter_stop_t stop,
                                               flexio_shifter_source_t source)
{
    baseAddr->SHIFTCFG[shifter] = FLEXIO_SHIFTCFG_SSTART(start)
                                | FLEXIO_SHIFTCFG_SSTOP(stop)
                                | FLEXIO_SHIFTCFG_INSRC(source);
}


/*!
 * @brief Reads the value from the specified shifter buffer.
 *
 * This function reads data from the specified shifter buffer. The data can be
 * read in any of the four ways allowed by the hardware - see description of type
 * flexio_shifter_buffer_mode_t for details.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @param[in] mode Read mode.
 * @return Value read from the shifter buffer.
 * Implements : FLEXIO_HAL_ReadShifterBuffer_Activity
 */
static inline uint32_t FLEXIO_HAL_ReadShifterBuffer(const FLEXIO_Type *baseAddr,
                                                    uint8_t shifter,
                                                    flexio_shifter_buffer_mode_t mode)
{
    uint32_t data;

#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    switch (mode)
    {
        case FLEXIO_SHIFTER_RW_MODE_NORMAL:
            data = baseAddr->SHIFTBUF[shifter];
            break;
        case FLEXIO_SHIFTER_RW_MODE_BIT_SWAP:
            data = baseAddr->SHIFTBUFBIS[shifter];
            break;
        case FLEXIO_SHIFTER_RW_MODE_BYTE_SWAP:
            data = baseAddr->SHIFTBUFBYS[shifter];
            break;
        case FLEXIO_SHIFTER_RW_MODE_BB_SWAP:
            data = baseAddr->SHIFTBUFBBS[shifter];
            break;
        default:
            /* bad read mode */
            data = 0U;
            break;
    }
    return data;
}


/*!
 * @brief Writes a value in the specified shifter buffer.
 *
 * This function writes data in the specified shifter buffer. The data can be
 * written in any of the four ways allowed by the hardware - see description of type
 * flexio_shifter_buffer_mode_t for details.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] shifter Number of the shifter.
 * @param[in] value Value to write in the shifter buffer.
 * @param[in] mode Write mode.
 * Implements : FLEXIO_HAL_WriteShifterBuffer_Activity
 */
static inline void FLEXIO_HAL_WriteShifterBuffer(FLEXIO_Type *baseAddr,
                                                 uint8_t shifter,
                                                 uint32_t value,
                                                 flexio_shifter_buffer_mode_t mode)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(shifter < FLEXIO_HAL_GetShifterNum(baseAddr));
#endif
    switch (mode)
    {
        case FLEXIO_SHIFTER_RW_MODE_NORMAL:
            baseAddr->SHIFTBUF[shifter] = value;
            break;
        case FLEXIO_SHIFTER_RW_MODE_BIT_SWAP:
            baseAddr->SHIFTBUFBIS[shifter] = value;
            break;
        case FLEXIO_SHIFTER_RW_MODE_BYTE_SWAP:
            baseAddr->SHIFTBUFBYS[shifter] = value;
            break;
        case FLEXIO_SHIFTER_RW_MODE_BB_SWAP:
            baseAddr->SHIFTBUFBBS[shifter] = value;
            break;
        default:
            /* bad write mode */
            break;
    }
}


/*!
 * @brief Returns the currently configured trigger for the specified timer.
 *
 * This function returns the currently configured trigger for the specified timer,
 * and also its source (internal or external) and polarity settings.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[out] trigger Number of the trigger.
 * @param[out] polarity Polarity of the trigger.
 * @param[out] source Trigger source.
 * Implements : FLEXIO_HAL_GetTimerTrigger_Activity
 */
static inline void FLEXIO_HAL_GetTimerTrigger(const FLEXIO_Type *baseAddr,
                                              uint8_t timer,
                                              uint8_t *trigger,
                                              flexio_trigger_polarity_t *polarity,
                                              flexio_trigger_source_t *source)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCTL[timer];
    *trigger = (uint8_t)((tmp & FLEXIO_TIMCTL_TRGSEL_MASK) >> FLEXIO_TIMCTL_TRGSEL_SHIFT);
    *polarity = (flexio_trigger_polarity_t)((tmp & FLEXIO_TIMCTL_TRGPOL_MASK) >> FLEXIO_TIMCTL_TRGPOL_SHIFT);
    *source = (flexio_trigger_source_t)((tmp & FLEXIO_TIMCTL_TRGSRC_MASK) >> FLEXIO_TIMCTL_TRGSRC_SHIFT);
}


/*!
 * @brief Configures the trigger for the specified timer.
 *
 * This function configures the trigger for the specified timer, and also its
 * source (internal or external) and polarity settings.
 * For internal triggers, the selection is as follows:
 *  - 4*N - Pin 2*N input
 *  - 4*N+1 - Shifter N status flag
 *  - 4*N+2 - Pin 2*N+1 input
 *  - 4*N+3 - Timer N trigger output
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[in] trigger Number of the trigger.
 * @param[in] polarity Polarity of the trigger.
 * @param[in] source Trigger source.
 * Implements : FLEXIO_HAL_SetTimerTrigger_Activity
 */
static inline void FLEXIO_HAL_SetTimerTrigger(FLEXIO_Type *baseAddr,
                                                 uint8_t timer,
                                                 uint8_t trigger,
                                                 flexio_trigger_polarity_t polarity,
                                                 flexio_trigger_source_t source)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCTL[timer];
    tmp &= ~(FLEXIO_TIMCTL_TRGSEL_MASK);
    tmp |= FLEXIO_TIMCTL_TRGSEL(trigger);
    tmp &= ~(FLEXIO_TIMCTL_TRGPOL_MASK);
    tmp |= FLEXIO_TIMCTL_TRGPOL(polarity);
    tmp &= ~(FLEXIO_TIMCTL_TRGSRC_MASK);
    tmp |= FLEXIO_TIMCTL_TRGSRC(source);
    baseAddr->TIMCTL[timer] = tmp;
}


/*!
 * @brief Returns the currently configured pin for the specified timer.
 *
 * This function returns the pin currently assigned to the specified timer, and also its
 * polarity and configuration.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[out] pin Number of the pin.
 * @param[out] polarity Polarity of the pin.
 * @param[out] config Pin configuration.
 * Implements : FLEXIO_HAL_GetTimerPin_Activity
 */
static inline void FLEXIO_HAL_GetTimerPin(const FLEXIO_Type *baseAddr,
                                              uint8_t timer,
                                              uint8_t *pin,
                                              flexio_pin_polarity_t *polarity,
                                              flexio_pin_config_t *config)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCTL[timer];
    *pin = (uint8_t)((tmp & FLEXIO_TIMCTL_PINSEL_MASK) >> FLEXIO_TIMCTL_PINSEL_SHIFT);
    *polarity = (flexio_pin_polarity_t)((tmp & FLEXIO_TIMCTL_PINPOL_MASK) >> FLEXIO_TIMCTL_PINPOL_SHIFT);
    *config = (flexio_pin_config_t)((tmp & FLEXIO_TIMCTL_PINCFG_MASK) >> FLEXIO_TIMCTL_PINCFG_SHIFT);
}


/*!
 * @brief Configures the pin for the specified timer.
 *
 * This function assigns the specified pin to the specified timer, and also sets
 * its polarity and configuration.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[in] pin Number of the pin.
 * @param[in] polarity Polarity of the pin.
 * @param[in] config Pin configuration.
 * Implements : FLEXIO_HAL_SetTimerPin_Activity
 */
static inline void FLEXIO_HAL_SetTimerPin(FLEXIO_Type *baseAddr,
                                                 uint8_t timer,
                                                 uint8_t pin,
                                                 flexio_pin_polarity_t polarity,
                                                 flexio_pin_config_t config)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
    DEV_ASSERT(pin < FLEXIO_HAL_GetPinNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCTL[timer];
    tmp &= ~(FLEXIO_TIMCTL_PINSEL_MASK);
    tmp |= FLEXIO_TIMCTL_PINSEL(pin);
    tmp &= ~(FLEXIO_TIMCTL_PINPOL_MASK);
    tmp |= FLEXIO_TIMCTL_PINPOL(polarity);
    tmp &= ~(FLEXIO_TIMCTL_PINCFG_MASK);
    tmp |= FLEXIO_TIMCTL_PINCFG(config);
    baseAddr->TIMCTL[timer] = tmp;
}


/*!
 * @brief Returns the mode of the specified timer.
 *
 * This function returns the currently configured mode for the specified timer.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @return Mode assigned to the timer.
 * Implements : FLEXIO_HAL_GetTimerMode_Activity
 */
static inline flexio_timer_mode_t FLEXIO_HAL_GetTimerMode(const FLEXIO_Type *baseAddr, uint8_t timer)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCTL[timer];
    tmp = (tmp & FLEXIO_TIMCTL_TIMOD_MASK) >> FLEXIO_TIMCTL_TIMOD_SHIFT;
    return (flexio_timer_mode_t)tmp;
}


/*!
 * @brief Sets the mode of the specified timer.
 *
 * This function configures the mode for the specified timer.
 * In 8-bit counter mode, the lower 8-bits of the counter and compare register are used to
 * configure the baud rate of the timer shift clock and the upper 8-bits are used to configure
 * the shifter bit count.
 * In 8-bit PWM mode, the lower 8-bits of the counter and compare
 * register are used to configure the high period of the timer shift clock and the upper
 * 8-bits are used to configure the low period of the timer shift clock. The shifter bit count
 * is configured using another timer or external signal.
 * In 16-bit counter mode, the full 16-bits of the counter and compare register are used to
 * configure either the baud rate of the shift clock or the shifter bit count.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[in] mode Mode assigned to the timer.
 * Implements : FLEXIO_HAL_SetTimerMode_Activity
 */
static inline void FLEXIO_HAL_SetTimerMode(FLEXIO_Type *baseAddr,
                                             uint8_t timer,
                                             flexio_timer_mode_t mode)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCTL[timer];
    tmp &= ~(FLEXIO_TIMCTL_TIMOD_MASK);
    tmp |= FLEXIO_TIMCTL_TIMOD(mode);
    baseAddr->TIMCTL[timer] = tmp;
}


/*!
 * @brief Sets all control settings for specified timer.
 *
 * This function configures the control settings for the specified timer: mode settings, 
 * pin settings and trigger settings. It sums up all settings 
 * from FLEXIO_HAL_SetTimerMode(), FLEXIO_HAL_SetTimerPin() and FLEXIO_HAL_SetTimerTrigger()
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[in] trigger Number of the trigger.
 * @param[in] triggerPolarity Polarity of the trigger.
 * @param[in] triggerSource Trigger source.
 * @param[in] pin Number of the pin.
 * @param[in] pinPolarity Polarity of the pin.
 * @param[in] pinConfig Pin configuration.
 * @param[in] mode Mode assigned to the timer.
 * Implements : FLEXIO_HAL_SetTimerControl_Activity
 */
static inline void FLEXIO_HAL_SetTimerControl(FLEXIO_Type *baseAddr,
                                              uint8_t timer,
                                              uint8_t trigger,
                                              flexio_trigger_polarity_t triggerPolarity,
                                              flexio_trigger_source_t triggerSource,
                                              uint8_t pin,
                                              flexio_pin_polarity_t pinPolarity,
                                              flexio_pin_config_t pinConfig,
                                              flexio_timer_mode_t mode)
{
    baseAddr->TIMCTL[timer] = FLEXIO_TIMCTL_TIMOD(mode)
                            | FLEXIO_TIMCTL_PINSEL(pin)
                            | FLEXIO_TIMCTL_PINPOL(pinPolarity)
                            | FLEXIO_TIMCTL_PINCFG(pinConfig)
                            | FLEXIO_TIMCTL_TRGSRC(triggerSource)
                            | FLEXIO_TIMCTL_TRGPOL(triggerPolarity)
                            | FLEXIO_TIMCTL_TRGSEL(trigger);
}



/*!
 * @brief Returns the initial output configuration of the specified timer.
 *
 * This function returns the initial output configuration of the specified timer.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @return Output configuration for the timer.
 * Implements : FLEXIO_HAL_GetTimerInitialOutput_Activity
 */
static inline flexio_timer_output_t FLEXIO_HAL_GetTimerInitialOutput(const FLEXIO_Type *baseAddr, uint8_t timer)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCFG[timer];
    tmp = (tmp & FLEXIO_TIMCFG_TIMOUT_MASK) >> FLEXIO_TIMCFG_TIMOUT_SHIFT;
    return (flexio_timer_output_t)tmp;
}


/*!
 * @brief Configures the initial output of the specified timer.
 *
 * This function configures the initial output of the specified timer and whether it is
 * affected by the Timer reset.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[in] output Output configuration for the timer.
 * Implements : FLEXIO_HAL_SetTimerInitialOutput_Activity
 */
static inline void FLEXIO_HAL_SetTimerInitialOutput(FLEXIO_Type *baseAddr,
                                             uint8_t timer,
                                             flexio_timer_output_t output)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCFG[timer];
    tmp &= ~(FLEXIO_TIMCFG_TIMOUT_MASK);
    tmp |= FLEXIO_TIMCFG_TIMOUT(output);
    baseAddr->TIMCFG[timer] = tmp;
}


/*!
 * @brief Returns the decrement configuration of the specified timer.
 *
 * This function returns the decrement configuration for the specified timer.
 * See description of type flexio_timer_decrement_t for a list of options.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @return Decrement configuration for the timer.
 * Implements : FLEXIO_HAL_GetTimerDecrement_Activity
 */
static inline flexio_timer_decrement_t FLEXIO_HAL_GetTimerDecrement(const FLEXIO_Type *baseAddr, uint8_t timer)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCFG[timer];
    tmp = (tmp & FLEXIO_TIMCFG_TIMDEC_MASK) >> FLEXIO_TIMCFG_TIMDEC_SHIFT;
    return (flexio_timer_decrement_t)tmp;
}


/*!
 * @brief Configures the decrement condition for the specified timer.
 *
 * This function configures the decrement condition for the specified timer and the source
 * of the shift clock.
 * See description of type flexio_timer_decrement_t for a list of options.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[in] decrement Decrement configuration for the timer.
 * Implements : FLEXIO_HAL_SetTimerDecrement_Activity
 */
static inline void FLEXIO_HAL_SetTimerDecrement(FLEXIO_Type *baseAddr,
                                             uint8_t timer,
                                             flexio_timer_decrement_t decrement)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCFG[timer];
    tmp &= ~(FLEXIO_TIMCFG_TIMDEC_MASK);
    tmp |= FLEXIO_TIMCFG_TIMDEC(decrement);
    baseAddr->TIMCFG[timer] = tmp;
}


/*!
 * @brief Returns the reset configuration of the specified timer.
 *
 * This function returns the reset configuration for the specified timer.
 * See description of type flexio_timer_reset_t for a list of options.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @return Reset configuration for the timer.
 * Implements : FLEXIO_HAL_GetTimerReset_Activity
 */
static inline flexio_timer_reset_t FLEXIO_HAL_GetTimerReset(const FLEXIO_Type *baseAddr, uint8_t timer)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCFG[timer];
    tmp = (tmp & FLEXIO_TIMCFG_TIMRST_MASK) >> FLEXIO_TIMCFG_TIMRST_SHIFT;
    return (flexio_timer_reset_t)tmp;
}


/*!
 * @brief Configures the reset condition for the specified timer.
 *
 * This function configures the conditions that cause the timer counter (and optionally
 * output) to be reset. In 8-bit counter mode, the timer reset will only reset the
 * lower 8-bits that configure the baud rate. In all other modes, the timer reset
 * will reset the full 16-bits of the counter.
 * See description of type flexio_timer_decrement_t for a list of options.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[in] reset Reset configuration for the timer.
 * Implements : FLEXIO_HAL_SetTimerReset_Activity
 */
static inline void FLEXIO_HAL_SetTimerReset(FLEXIO_Type *baseAddr,
                                             uint8_t timer,
                                             flexio_timer_reset_t reset)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCFG[timer];
    tmp &= ~(FLEXIO_TIMCFG_TIMRST_MASK);
    tmp |= FLEXIO_TIMCFG_TIMRST(reset);
    baseAddr->TIMCFG[timer] = tmp;
}


/*!
 * @brief Returns the disable configuration of the specified timer.
 *
 * This function returns the condition that cause the specified timer to be disabled.
 * See description of type flexio_timer_disable_t for a list of options.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @return Disable configuration for the timer.
 * Implements : FLEXIO_HAL_GetTimerDisable_Activity
 */
static inline flexio_timer_disable_t FLEXIO_HAL_GetTimerDisable(const FLEXIO_Type *baseAddr, uint8_t timer)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCFG[timer];
    tmp = (tmp & FLEXIO_TIMCFG_TIMDIS_MASK) >> FLEXIO_TIMCFG_TIMDIS_SHIFT;
    return (flexio_timer_disable_t)tmp;
}


/*!
 * @brief Configures the disable condition for the specified timer.
 *
 * This function configures the condition that cause the specified timer to be disabled.
 * See description of type flexio_timer_disable_t for a list of options.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[in] disable Disable configuration for the timer.
 * Implements : FLEXIO_HAL_SetTimerDisable_Activity
 */
static inline void FLEXIO_HAL_SetTimerDisable(FLEXIO_Type *baseAddr,
                                             uint8_t timer,
                                             flexio_timer_disable_t disable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCFG[timer];
    tmp &= ~(FLEXIO_TIMCFG_TIMDIS_MASK);
    tmp |= FLEXIO_TIMCFG_TIMDIS(disable);
    baseAddr->TIMCFG[timer] = tmp;
}


/*!
 * @brief Returns the enable condition configuration of the specified timer.
 *
 * This function returns the condition that cause the specified timer to be enabled
 * and start decrementing.
 * See description of type flexio_timer_disable_t for a list of options.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @return Enable condition configuration for the timer.
 * Implements : FLEXIO_HAL_GetTimerEnable_Activity
 */
static inline flexio_timer_enable_t FLEXIO_HAL_GetTimerEnable(const FLEXIO_Type *baseAddr, uint8_t timer)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCFG[timer];
    tmp = (tmp & FLEXIO_TIMCFG_TIMENA_MASK) >> FLEXIO_TIMCFG_TIMENA_SHIFT;
    return (flexio_timer_enable_t)tmp;
}


/*!
 * @brief Configures the enable condition for the specified timer.
 *
 * This function configures the condition that cause the specified timer to be enabled
 * and start decrementing.
 * See description of type flexio_timer_disable_t for a list of options.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[in] enable Enable condition configuration for the timer.
 * Implements : FLEXIO_HAL_SetTimerEnable_Activity
 */
static inline void FLEXIO_HAL_SetTimerEnable(FLEXIO_Type *baseAddr,
                                             uint8_t timer,
                                             flexio_timer_enable_t enable)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCFG[timer];
    tmp &= ~(FLEXIO_TIMCFG_TIMENA_MASK);
    tmp |= FLEXIO_TIMCFG_TIMENA(enable);
    baseAddr->TIMCFG[timer] = tmp;
}


/*!
 * @brief Returns the stop bit configuration of the specified timer.
 *
 * This function returns the current stop bit configuration for the specified timer.
 * The stop bit can be added on a timer compare (between each word) or on a timer disable.
 * When stop bit is enabled, configured shifters will output the contents of the stop bit
 * when the timer is disabled. When stop bit is enabled on timer disable, the timer remains
 * disabled until the next rising edge of the shift clock. If configured for both timer
 * compare and timer disable, only one stop bit is inserted on timer disable.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @return Stop bit configuration for the timer.
 * Implements : FLEXIO_HAL_GetTimerStop_Activity
 */
static inline flexio_timer_stop_t FLEXIO_HAL_GetTimerStop(const FLEXIO_Type *baseAddr, uint8_t timer)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCFG[timer];
    tmp = (tmp & FLEXIO_TIMCFG_TSTOP_MASK) >> FLEXIO_TIMCFG_TSTOP_SHIFT;
    return (flexio_timer_stop_t)tmp;
}


/*!
 * @brief Configures the stop bit for the specified timer.
 *
 * This function configures stop bit insertion for the specified timer.
 * The stop bit can be added on a timer compare (between each word) or on a timer disable.
 * When stop bit is enabled, configured shifters will output the contents of the stop bit
 * when the timer is disabled. When stop bit is enabled on timer disable, the timer remains
 * disabled until the next rising edge of the shift clock. If configured for both timer
 * compare and timer disable, only one stop bit is inserted on timer disable.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[in] stop Stop bit configuration for the timer.
 * Implements : FLEXIO_HAL_SetTimerStop_Activity
 */
static inline void FLEXIO_HAL_SetTimerStop(FLEXIO_Type *baseAddr,
                                             uint8_t timer,
                                             flexio_timer_stop_t stop)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t tmp = baseAddr->TIMCFG[timer];
    tmp &= ~(FLEXIO_TIMCFG_TSTOP_MASK);
    tmp |= FLEXIO_TIMCFG_TSTOP(stop);
    baseAddr->TIMCFG[timer] = tmp;
}


/*!
 * @brief Returns the start bit configuration of the specified timer.
 *
 * This function returns the current start bit configuration for the specified timer.
 * When start bit is enabled, configured shifters will output the contents of the start
 * bit when the timer is enabled and the timer counter will reload from the compare
 * register on the first rising edge of the shift clock.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @return Start bit configuration for the timer.
 * Implements : FLEXIO_HAL_GetTimerStart_Activity
 */
static inline flexio_timer_start_t FLEXIO_HAL_GetTimerStart(const FLEXIO_Type *baseAddr, uint8_t timer)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t regValue = (uint32_t)baseAddr->TIMCFG[timer];
    regValue = (regValue & FLEXIO_TIMCFG_TSTART_MASK) >> FLEXIO_TIMCFG_TSTART_SHIFT;
    return (flexio_timer_start_t)regValue;
}


/*!
 * @brief Configures the start bit for the specified timer.
 *
 * This function configures start bit insertion for the specified timer.
 * When start bit is enabled, configured shifters will output the contents of the start
 * bit when the timer is enabled and the timer counter will reload from the compare
 * register on the first rising edge of the shift clock.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[in] start Start bit configuration for the timer.
 * Implements : FLEXIO_HAL_SetTimerStart_Activity
 */
static inline void FLEXIO_HAL_SetTimerStart(FLEXIO_Type *baseAddr,
                                             uint8_t timer,
                                             flexio_timer_start_t start)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    uint32_t regValue = (uint32_t)baseAddr->TIMCFG[timer];
    regValue &= (uint32_t)(~(FLEXIO_TIMCFG_TSTART_MASK));
    regValue |= FLEXIO_TIMCFG_TSTART(start);
    baseAddr->TIMCFG[timer] = (uint32_t)regValue;
}


/*!
 * @brief Sets all configuration settings for specified timer.
 *
 * This function sets the following configurations for the specified timer:
 * start bit, stop bit, enable condition, disable condition, reset condition, 
 * decrement condition, initial output
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[in] start Start bit configuration for the timer.
 * @param[in] stop Stop bit configuration for the timer.
 * @param[in] enable Enable condition configuration for the timer.
 * @param[in] disable Disable configuration for the timer.
 * @param[in] reset Reset configuration for the timer.
 * @param[in] decrement Decrement configuration for the timer.
 * @param[in] output Output configuration for the timer.
 * Implements : FLEXIO_HAL_SetTimerConfig_Activity
 */
static inline void FLEXIO_HAL_SetTimerConfig(FLEXIO_Type *baseAddr,
                                             uint8_t timer,
                                             flexio_timer_start_t start,
                                             flexio_timer_stop_t stop,
                                             flexio_timer_enable_t enable,
                                             flexio_timer_disable_t disable,
                                             flexio_timer_reset_t reset,
                                             flexio_timer_decrement_t decrement,
                                             flexio_timer_output_t output)
{

    baseAddr->TIMCFG[timer] = FLEXIO_TIMCFG_TSTART(start)
                            | FLEXIO_TIMCFG_TSTOP(stop)
                            | FLEXIO_TIMCFG_TIMENA(enable)
                            | FLEXIO_TIMCFG_TIMDIS(disable)
                            | FLEXIO_TIMCFG_TIMRST(reset)
                            | FLEXIO_TIMCFG_TIMDEC(decrement)
                            | FLEXIO_TIMCFG_TIMOUT(output);
}


/*!
 * @brief Returns the compare value of the specified timer.
 *
 * This function returns the compare value currently set for the specified timer.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @return Compare value for the specified timer.
 * Implements : FLEXIO_HAL_GetTimerCompare_Activity
 */
static inline uint16_t FLEXIO_HAL_GetTimerCompare(const FLEXIO_Type *baseAddr, uint8_t timer)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    return (uint16_t)(baseAddr->TIMCMP[timer]);
}


/*!
 * @brief Configures the compare value for the specified timer.
 *
 * This function configures the compare value for the specified timer.
 * The timer compare value is loaded into the timer counter when the timer is first enabled,
 * when the timer is reset and when the timer decrements down to zero.
 * In dual 8-bit counters baud/bit mode, the lower 8-bits configure the baud rate divider
 * and the upper 8-bits configure the number of bits in each word.
 * In dual 8-bit counters PWM mode, the lower 8-bits configure the high period of the output
 * and the upper 8-bits configure the low period.
 * In 16-bit counter mode, the compare value can be used to generate the baud rate divider
 * (if shift clock source is timer output) or the number of bits in each word (when the shift
 * clock source is a pin or trigger input).
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 * @param[in] timer Number of the timer.
 * @param[in] value Compare value for the specified timer.
 * Implements : FLEXIO_HAL_SetTimerCompare_Activity
 */
static inline void FLEXIO_HAL_SetTimerCompare(FLEXIO_Type *baseAddr,
                                             uint8_t timer,
                                             uint16_t value)
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(timer < FLEXIO_HAL_GetTimerNum(baseAddr));
#endif
    baseAddr->TIMCMP[timer] = (uint32_t)value;
}


/*!
 * @brief Initializes the FlexIO module to a known state.
 *
 * This function initializes all the registers of the FlexIO module to
 * their reset value.
 *
 * @param[in] baseAddr Base address of the FlexIO module.
 */
void FLEXIO_HAL_Init(FLEXIO_Type *baseAddr);


/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* FSL_FLEXIO_HAL_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
