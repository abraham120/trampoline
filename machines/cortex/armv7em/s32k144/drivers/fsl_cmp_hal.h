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
/**
* @page misra_violations MISRA-C:2012 violations
*
* @section [global]
* Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially 
* unsigned' to 'essentially Boolean'.
* This is required by the conversion of a bit into a bool.
*
* @section [global]
* Violates MISRA 2012 Advisory Rule 10.5, Impermissible cast; cannot cast from 'essentially
* unsigned' to 'essentially enum<i>'.
* This is required by the conversion of a bitfiel of a register into a enum.
*
* @section [global]
* Violates MISRA 2012 Required Rule 10.8, Impermissible cast of composite expression 
*(different essential type categories).
* This is required by the conversion of a bit/bitfield of a register into boolean or a enum type.
*
* @section [global]
* Violates MISRA 2012 Required Rule 10.3, Expression assigned to a narrower 
* or different essential type [MISRA 2012 Rule 10.3, required]
* This is required by the conversion of a unsigned value of a bitfield/bit into a enum value.
*/

#ifndef FSL_CMP_HAL_H
#define FSL_CMP_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"

/*!
 * @addtogroup cmp_hal Comparator HAL
 * @ingroup cmp 
 * @brief Comparator Hardware Abstraction Layer 
 * @{
 */

#define CMP_INPUT_FLAGS_MASK 0xFF0000
#define CMP_INPUT_FLAGS_SHIFT 16U
#define CMP_ROUND_ROBIN_CHANNELS_MASK 0xFF0000
#define CMP_ROUND_ROBIN_CHANNELS_SHIFT 16U

/*******************************************************************************
 * Enumerations.
 ******************************************************************************/
 /*! @brief Power Modes selection
  * Implements : cmp_power_mode_t_Class
  */
typedef enum
{
    CMP_LOW_SPEED   = 0U,   /*!< Module in low speed mode. */
    CMP_HIGH_SPEED  = 1U    /*!< Module in high speed mode. */
} cmp_power_mode_t;

 /*! @brief Voltage Reference selection
  * Implements : cmp_voltage_reference_t_Class
  */
typedef enum
{
    CMP_VIN1 = 0U,  /*!< Use Vin1 as supply reference source for DAC. */
    CMP_VIN2 = 1U   /*!< Use Vin2 as supply reference source for DAC. */
} cmp_voltage_reference_t;

 /*! @brief Port Mux Source selection
  * Implements : cmp_port_mux_t_Class
  */
typedef enum
{
    CMP_DAC = 0U,   /*!< Select DAC as source for the comparator port. */
    CMP_MUX = 1U    /*!< Select MUX8 as source for the comparator port. */
} cmp_port_mux_t;

 /*! @brief Comparator output invert selection
  * Implements : cmp_inverter_t_Class
  */
typedef enum
{
    CMP_NORMAL = 0U,    /*!< Output signal isn't inverted. */
    CMP_INVERT = 1U     /*!< Output signal is inverted. */
} cmp_inverter_t;

 /*! @brief Comparator output select selection
  * Implements : cmp_output_select_t_Class
  */
typedef enum
{
    CMP_COUT  = 0U,     /*!< Select COUT as comparator output signal. */
    CMP_COUTA = 1U      /*!< Select COUTA as comparator output signal. */
} cmp_output_select_t;

 /*! @brief Comparator output pin enable selection
  * Implements : cmp_output_enable_t_Class
  */
typedef enum
{
    CMP_UNAVAILABLE  = 0U,  /*!< Comparator output isn't available to a specific pin*/
    CMP_AVAILABLE    = 1U   /*!< Comparator output is available to a specific pin*/
} cmp_output_enable_t;

 /*! @brief Comparator hard block offset control
  * Implements : cmp_offset_t_Class
  */
typedef enum
{
    CMP_LEVEL_OFFSET_0 = 0U,
    CMP_LEVEL_OFFSET_1 = 1U
} cmp_offset_t;

 /*! @brief Comparator hysteresis control
  * Implements : cmp_hysteresis_t_Class
  */
typedef enum
{
    CMP_LEVEL_HYS_0 = 0U,
    CMP_LEVEL_HYS_1 = 1U,
    CMP_LEVEL_HYS_2 = 2U,
    CMP_LEVEL_HYS_3 = 3U
} cmp_hysteresis_t;

 /*! @brief Comparator Round-Robin fixed port
  * Implements : cmp_fixed_port_t_Class
  */
typedef enum
{
    CMP_PLUS_FIXED      = 0U,   /*!< The Plus port is fixed. Only the inputs to the Minus port are swept in each round. */
    CMP_MINUS_FIXED     = 1U    /*!< The Minus port is fixed. Only the inputs to the Plus port are swept in each round. */
} cmp_fixed_port_t;

 /*! @brief Comparator output interrupt configuration
  * Implements : cmp_output_trigger_t_Class
  */
typedef enum
{
    CMP_NO_EVENT            = 0U,   /*!< Comparator output interrupts are disabled OR no event occurred. */
    CMP_FALLING_EDGE        = 1U,   /*!< Comparator output interrupts will be generated only on falling edge OR only falling edge event occurred. */
    CMP_RISING_EDGE         = 2U,   /*!< Comparator output interrupts  will be generated only on rising edge OR only rising edge event occurred. */
    CMP_BOTH_EDGES          = 3U    /*!< Comparator output interrupts  will be generated on both edges OR both edges event occurred. */
} cmp_output_trigger_t;

 /*! @brief Comparator functional modes
  * Implements : cmp_mode_t_Class
  */
typedef enum
{
    CMP_DISABLED                        = 0U,
    CMP_CONTINUOUS                      = 1U,
    CMP_SAMPLED_NONFILTRED_INT_CLK      = 2U,
    CMP_SAMPLED_NONFILTRED_EXT_CLK      = 3U,
    CMP_SAMPLED_FILTRED_INT_CLK         = 4U,
    CMP_SAMPLED_FILTRED_EXT_CLK         = 5U,
    CMP_WINDOWED                        = 6U,
    CMP_WINDOWED_RESAMPLED              = 7U,
    CMP_WINDOWED_FILTRED                = 8U
} cmp_mode_t;

 /*! @brief Comparator channels list (1bit/channel)
   |---------|---------|-----|---------|---------|
   |CH7_state|CH6_state|.....|CH1_state|CH0_state|
   |---------|---------|-----|---------|---------|
  * Implements : cmp_ch_list_t_Class
  */
typedef  uint8_t cmp_ch_list_t;

 /*! @brief Number of channel
  * Implements : cmp_ch_number_t_Class  
  */
typedef  uint8_t cmp_ch_number_t;

 

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @name CMP_HAL.
 * @{
 */

 #if defined(__cplusplus)
extern "C" {
#endif
/*!
 * @brief Initializes the comparator registers with reset values
 * @param baseAddr - cmp base pointer
 * @return - void
 */
void CMP_HAL_Init(CMP_Type* baseAddr);

/*!
 * @brief Gets the comparator functional mode. If you want to get filter count and filter period please use CMP_HAL_GetFilterSamplePeriod
 * and CMP_HAL_GetSamplingState.
 * @param baseAddr - cmp base pointer
 * @return - functional mode
 *   CMP_DISABLED
 *   CMP_CONTINUOUS
 *   CMP_SAMPLED_NONFILTRED_INT_CLK 
 *   CMP_SAMPLED_NONFILTRED_EXT_CLK
 *   CMP_SAMPLED_FILTRED_INT_CLK
 *   CMP_SAMPLED_FILTRED_EXT_CLK
 *   CMP_WINDOWED
 *   CMP_WINDOWED_RESAMPLED
 *   CMP_WINDOWED_FILTRED
 */
cmp_mode_t CMP_HAL_GetFunctionalMode(const CMP_Type* baseAddr);

/*!
 * @brief Sets the comparator functional mode (mode, filter count, filter period)
 * @param baseAddr - cmp base pointer
 * @param mode - functional mode
 *  CMP_DISABLED
 *  CMP_CONTINUOUS
 *  CMP_SAMPLED_NONFILTRED_INT_CLK 
 *  CMP_SAMPLED_NONFILTRED_EXT_CLK
 *  CMP_SAMPLED_FILTRED_INT_CLK
 *  CMP_SAMPLED_FILTRED_EXT_CLK
 *  CMP_WINDOWED
 *  CMP_WINDOWED_RESAMPLED
 *  CMP_WINDOWED_FILTRED
 * @param filter_sample_count - number of consecutive samples that must agree prior to the comparator ouput filter
 * accepting a new output state
 * @param filter_sample_period - sampling period
 * @return -void
 */
void CMP_HAL_SetFunctionalMode(CMP_Type* baseAddr, cmp_mode_t mode, uint8_t filter_sample_count, uint8_t filter_sample_period);

/*!
 * @brief Verify if the DMA transfer trigger is enabled
 * @param baseAddr - cmp base pointer
 * @return - DMA transfer trigger state
 *  true - DMA trigger is enabled
 *  false - DAM trigger is disabled
 * Implements : CMP_HAL_GetDMATriggerState_Activity
 */
static inline bool CMP_HAL_GetDMATriggerState(const CMP_Type* baseAddr)
{
    return (bool)(((baseAddr->C0) >> (CMP_C0_DMAEN_SHIFT)) & 1U);
}

/*!
 * @brief Configure the DMA transfer trigger
 * @param baseAddr - cmp base pointer
 * @param to_set - DMA transfer trigger state
 *  true - DMA trigger is enabled
 *  false - DAM trigger is disabled
 * @return - void
 * Implements : CMP_HAL_SetDMATriggerState_Activity
 */
static inline void CMP_HAL_SetDMATriggerState(CMP_Type* baseAddr, bool to_set)
{
    baseAddr->C0 = (baseAddr->C0 & (~(CMP_C0_DMAEN_MASK))) | ((uint32_t)to_set << CMP_C0_DMAEN_SHIFT);
}

/*!
 * @brief Return the comparator output interrupts source configuration(none, rising edge, falling edge or both edges) 
 * @param baseAddr - cmp base pointer
 * @return - comparator output interrupts configuration
 *  CMP_NO_EVENT
 *  CMP_FALLING_EDGE
 *  CMP_RISING_EDGE
 *  CMP_BOTH_EDGES
 * Implements : CMP_HAL_GetOutputInterruptTrigger_Activity
 */
static inline cmp_output_trigger_t CMP_HAL_GetOutputInterruptTrigger(const CMP_Type* baseAddr)
{
    bool rising_enabled = (bool)(((baseAddr->C0) >> CMP_C0_IER_SHIFT) & 1U);
    bool falling_enabled = (bool)(((baseAddr->C0) >> CMP_C0_IEF_SHIFT) & 1U);
    return (cmp_output_trigger_t) (((uint32_t)rising_enabled << (uint32_t)1U) | ((uint32_t)falling_enabled));
}

/*!
 * @brief Set the comparator output interrupts source configuration(none, rising edge, falling edge or both edges) 
 * @param baseAddr - cmp base pointer
 * @param to_set - comparator output interrupts configuration
 *  CMP_NO_EVENT
 *  CMP_FALLING_EDGE
 *  CMP_RISING_EDGE
 *  CMP_BOTH_EDGES
 * @return - void
 * Implements : CMP_HAL_SetOutputInterruptTrigger_Activity
 */
static inline void CMP_HAL_SetOutputInterruptTrigger(CMP_Type* baseAddr, cmp_output_trigger_t to_set)
{
    uint32_t tmp = baseAddr->C0;
    tmp &= ~(CMP_C0_IER_MASK) & ~(CMP_C0_IEF_MASK);
    tmp |= CMP_C0_IER((uint32_t)to_set>>1) | CMP_C0_IEF((uint32_t)to_set & (uint32_t)0x01);
    baseAddr->C0 = tmp;
}

/*!
 * @brief Return type of event occurred at the comparator output
 * @param baseAddr - cmp base pointer
 * @return - comparator output flags
 *  CMP_NO_EVENT
 *  CMP_FALLING_EDGE
 *  CMP_RISING_EDGE
 *  CMP_BOTH_EDGES 
 * Implements : CMP_HAL_GetOutputEvent_Activity
 */
static inline cmp_output_trigger_t CMP_HAL_GetOutputEvent(const CMP_Type* baseAddr)
{
    bool rising_enabled = (bool)(((baseAddr->C0) >> CMP_C0_CFR_SHIFT) & 1U);
    bool falling_enabled = (bool)(((baseAddr->C0) >> CMP_C0_CFF_SHIFT) & 1U);
    return (cmp_output_trigger_t) (((uint32_t)rising_enabled << (uint32_t)1U) | ((uint32_t)falling_enabled));
}

/*!
 * @brief Clear all output flags
 * @param baseAddr - cmp base pointer
 * @return - void
 * Implements : CMP_HAL_ClearOutputEvent_Activity
 */
static inline void CMP_HAL_ClearOutputEvent(CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C0;
    tmp &= ~(CMP_C0_CFR_MASK);
    tmp |= CMP_C0_CFR(1);
    tmp &= ~(CMP_C0_CFF_MASK);
    tmp |= CMP_C0_CFF(1);
    baseAddr->C0 = tmp;
    
    #ifdef ERRATA_E9005
        (void)baseAddr->C0;
    #endif
}

/*!
 * @brief Verify if a rising edge occurred on COUT
 * @param baseAddr - cmp base pointer
 * @return - rising-edge flag state
 *  true - rising-edge event occurred on COUT
 *  false - rising-edge event doesn't occurred on COUT
 * Implements : CMP_HAL_GetOutputRisingFlag_Activity
 */
static inline bool CMP_HAL_GetOutputRisingFlag(const CMP_Type* baseAddr)
{
    return (bool)(((baseAddr->C0) >> CMP_C0_CFR_SHIFT) & 1U);
}

/*!
 * @brief Clear rising edge flag
 * @param baseAddr - cmp base pointer
 * @return - void
 * Implements : CMP_HAL_ClearOutputRisingFlag_Activity
 */
static inline void CMP_HAL_ClearOutputRisingFlag(CMP_Type* baseAddr)
{
    (baseAddr->C0) |= ((uint32_t)1U << CMP_C0_CFR_SHIFT);
    
    #ifdef ERRATA_E9005
        (void)baseAddr->C0;
    #endif
}

/*!
 * @brief Verify if a falling-edge occurred on COUT
 * @param baseAddr cmp base pointer
 * @return -  Falling edge flag state
 *  true - falling-edge event occurred on COUT
 *  false - falling-edge event doesn't occurred on COUT
 * Implements : CMP_HAL_GetOutputFallingFlag_Activity
 */
static inline bool CMP_HAL_GetOutputFallingFlag(const CMP_Type* baseAddr)
{
    return (bool)(((baseAddr->C0) >> CMP_C0_CFF_SHIFT) & 1U);
}

/*!
 * @brief Clear falling edge flag
 * @param baseAddr - cmp base pointer
 * @return - void
 * Implements : CMP_HAL_ClearOutputFallingFlag_Activity
 */
static inline void CMP_HAL_ClearOutputFallingFlag(CMP_Type* baseAddr)
{
    (baseAddr->C0)|=  (uint32_t)1U << CMP_C0_CFF_SHIFT;
    
    #ifdef ERRATA_E9005
        (void)baseAddr->C0;
    #endif
}

/*!
 * @brief Return the analog comparator output value
 * @param baseAddr - cmp base pointer
 * @return - analog comparator output value
 * Implements : CMP_HAL_GetComparatorOutput_Activity
 */
static inline bool CMP_HAL_GetComparatorOutput(const CMP_Type* baseAddr)
{
    return (bool)(((baseAddr->C0) >> CMP_C0_COUT_SHIFT) & 1U);
}

/*!
 * @brief Return the sample period for filter(clock cycles)
 * @param baseAddr - cmp base pointer
 * @return - sampling period(in bus cycles)
 * Implements : CMP_HAL_GetFilterSamplePeriod_Activity
 */
static inline uint8_t CMP_HAL_GetFilterSamplePeriod(const CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C0;
    tmp = (tmp & CMP_C0_FPR_MASK) >> CMP_C0_FPR_SHIFT;
    return ( uint8_t ) (tmp);
}

/*!
 * @brief Set the filter sample period(clock cycles)
 * @param baseAddr -cmp base pointer
 * @param to_set - number of bus cycles 
 * @return - void
 * Implements : CMP_HAL_SetFilterSamplePeriod_Activity
 */
static inline void CMP_HAL_SetFilterSamplePeriod(CMP_Type* baseAddr, uint8_t to_set)
{
    uint32_t tmp = baseAddr->C0;
    baseAddr->C0 &= ~(CMP_C0_FPR_MASK);
    tmp &= ~(CMP_C0_FPR_MASK);
    tmp |= CMP_C0_FPR(to_set);
    baseAddr->C0 = tmp;
}

/*!
 * @brief Verify if the sampling mode is selected
 * @param baseAddr - cmp base pointer
 * @return - sampling mode state
 *  true - sampling mode is used
 *  false - sampling mode isn't used
 * Implements : CMP_HAL_GetSamplingState_Activity
 */
static inline bool CMP_HAL_GetSamplingState(const CMP_Type* baseAddr)
{
    return (bool)(((baseAddr->C0) >> CMP_C0_SE_SHIFT) & 1U);
}

/*!
 * @brief Set the sampling mode state
 * @param baseAddr - cmp the base pointer
 * @param to_set - sampling mode state
 *  true - sampling mode is used
 *  false - sampling mode isn't used
 * @return - void
 * Implements : CMP_HAL_SetSamplingState_Activity
 */
static inline void CMP_HAL_SetSamplingState(CMP_Type* baseAddr, bool to_set)
{
    (baseAddr->C0) = ((baseAddr->C0) & (~(CMP_C0_SE_MASK))) | ((uint32_t)to_set << CMP_C0_SE_SHIFT);
}

/*!
 * @brif Verify if the windowing mode is selected
 * @param baseAddr - cmp base pointer
 * @return - windowing mode state
 *  true - windowing mode is used
 *  false - windowing mode isn't used
 * Implements : CMP_HAL_GetWindowingModeState_Activity
 */
static inline bool CMP_HAL_GetWindowingModeState(const CMP_Type* baseAddr)
{
    return (bool)(((baseAddr->C0) >> CMP_C0_WE_SHIFT) & 1U);
}

/*!
 * @brief Set the windowing mode state
 * @param baseAddr - cmp base pointer
 * @param to_set - windowing mode state;
 *  true - windowing mode is used
 *  false - windowing mode isn't used
 * @return void
 * Implements : CMP_HAL_SetWindowingModeState_Activity
 */
static inline void CMP_HAL_SetWindowingModeState(CMP_Type* baseAddr, bool to_set)
{
    (baseAddr->C0) = ((baseAddr->C0) & (~(CMP_C0_WE_MASK))) | ((uint32_t)to_set << CMP_C0_WE_SHIFT);
}

/*!
 * @brief Return the current power mode
 * @param baseAddr - cmp base pointer
 * @return - current power mode 
 *  CMP_LOW_SPEED
 *  CMP_HIGH_SPEED
 * Implements : CMP_HAL_GetPowerMode_Activity
 */
static inline cmp_power_mode_t CMP_HAL_GetPowerMode(const CMP_Type* baseAddr)
{
    return (cmp_power_mode_t)(((baseAddr->C0) >> CMP_C0_PMODE_SHIFT) & 1U);
}

/*!
 * @brief Set the power mode
 * @param baseAddr - cmp base pointer
 * @param to_set - power mode
 *  CMP_LOW_SPEED
 *  CMP_HIGH_SPEED
 * Implements : CMP_HAL_SetPowerMode_Activity
 */
static inline void CMP_HAL_SetPowerMode(CMP_Type* baseAddr, cmp_power_mode_t to_set)
{
    (baseAddr->C0) = ((baseAddr->C0) & (~(CMP_C0_PMODE_MASK))) | ((uint32_t)to_set << CMP_C0_PMODE_SHIFT);
}

/*!
 * @brief Return the current comparator output inverter
 * @param baseAddr cmp base pointer
 * @return - inverter state
 *  CMP_NORMAL
 *  CMP_INVERT
 * Implements : CMP_HAL_GetInverterState_Activity
 */
static inline cmp_inverter_t CMP_HAL_GetInverterState(const CMP_Type* baseAddr)
{
    return (cmp_inverter_t)(((baseAddr->C0) >> CMP_C0_INVT_SHIFT) & 1U);
}

/*!
 * @brief Configure the comparator output inverter mode
 * @param baseAddr - cmp base pointer
 * @param to_set - comparator output inverter mode
 *  CMP_NORMAL
 *  CMP_INVERT
 * @return - void
 * Implements : CMP_HAL_SetInverterState_Activity
 */
static inline void CMP_HAL_SetInverterState(CMP_Type* baseAddr, cmp_inverter_t to_set)
{
    (baseAddr->C0) = ((baseAddr->C0) & (~(CMP_C0_INVT_MASK))) | ((uint32_t)to_set << CMP_C0_INVT_SHIFT);
}

/*!
 * @brief Return the current comparator output selected
 * @param baseAddr - cmp base pointer
 * @return - comparator output signal source
 *  CMP_COUT
 *  CMP_COUTA
 * Implements : CMP_HAL_GetComparatorOutputSource_Activity
 */
static inline cmp_output_select_t CMP_HAL_GetComparatorOutputSource(const CMP_Type* baseAddr)
{
    return (cmp_output_select_t)(((baseAddr->C0) >> CMP_C0_COS_SHIFT) & 1U);
}

/*!
 * @brief Select the comparator output signal source
 * @param baseAddr - cmp base pointer
 * @param to_set - comparator output signal source
 *  CMP_COUT
 *  CMP_COUTA
 * @return void
 * Implements : CMP_HAL_SetComparatorOutputSource_Activity
 */
static inline void CMP_HAL_SetComparatorOutputSource(CMP_Type* baseAddr, cmp_output_select_t to_set)
{
    (baseAddr->C0) = ((baseAddr->C0) & (~(CMP_C0_COS_MASK))) | ((uint32_t)to_set << CMP_C0_COS_SHIFT);
}

/*!
 * @brief Verify if the comparator output state(available/not available in a packaged pin)
 * @param baseAddr - cmp base pointer
 * @return - comparator output state
 *  CMP_UNAVAILABLE
 *  CMP_AVAILABLE
 * Implements : CMP_HAL_GetOutputPinState_Activity
 */
static inline cmp_output_enable_t CMP_HAL_GetOutputPinState(const CMP_Type* baseAddr)
{
    return (cmp_output_enable_t)(((baseAddr->C0) >> CMP_C0_OPE_SHIFT) & 1U);
}

/*!
 * @brief Set the comparator output pin state(available/not available in a packaged pin)
 * @param baseAddr - cmp base pointer
 * @param to_set - comparator output state
 *  CMP_UNAVAILABLE
 *  CMP_AVAILABLE
 * @return - void
 * Implements : CMP_HAL_SetOutputPinState_Activity
 */
static inline void CMP_HAL_SetOutputPinState(CMP_Type* baseAddr, cmp_output_enable_t to_set)
{

    (baseAddr->C0) = ((baseAddr->C0) & (~(CMP_C0_OPE_MASK))) | ((uint32_t)to_set << CMP_C0_OPE_SHIFT);
}

/*!
 * @brief Verify if the analog comparator module is enabled
 * @param baseAddr - cmp base pointer
 * @return - module state
 *  true - module is enabled
 *  false - module is disabled
 * Implements : CMP_HAL_GetAnalogComparatorState_Activity
 */
static inline bool CMP_HAL_GetAnalogComparatorState(const CMP_Type* baseAddr)
{
    return (bool)(((baseAddr->C0) >> CMP_C0_EN_SHIFT) & 1U);
}

/*!
 * @brief Set the analog comparator module state
 * @param baseAddr - cmp base pointer
 * @param to_set - analog comparator module state
 *  true - module is enabled
 *  false - module is disabled
 * Implements : CMP_HAL_SetAnalogComparatorState_Activity
 */
static inline void CMP_HAL_SetAnalogComparatorState(CMP_Type* baseAddr, bool to_set)
{
    (baseAddr->C0) = ((baseAddr->C0) & (~(CMP_C0_EN_MASK))) | ((uint32_t)to_set << CMP_C0_EN_SHIFT);
}

/*!
 * @brief Return the number of consecutive samples that must agree prior to the comparator output filter
accepting a new output state
 * @param baseAddr - cmp base pointer
 * @return - filter sample count
 * Implements : CMP_HAL_GetFilterSampleCount_Activity
 */
static inline uint8_t CMP_HAL_GetFilterSampleCount(const CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C0;
    tmp = (tmp & CMP_C0_FILTER_CNT_MASK) >> CMP_C0_FILTER_CNT_SHIFT;
    return ( uint8_t ) (tmp);
}

/*!
 * @brief Set the number of consecutive samples that must agree prior to the comparator output filter
accepting a new output state
 * @param baseAddr - cmp base pointer
 * @param to_set - filter sample count(min value 0, max value 7)
 * @return - void
 * Implements : CMP_HAL_SetFilterSampleCount_Activity
 */
static inline void CMP_HAL_SetFilterSampleCount(CMP_Type* baseAddr, uint8_t to_set)
{
    uint32_t tmp = baseAddr->C0;
    baseAddr->C0 &= ~(CMP_C0_FILTER_CNT_MASK);
    tmp &= ~(CMP_C0_FILTER_CNT_MASK);
    tmp |= CMP_C0_FILTER_CNT(to_set);
    baseAddr->C0 = tmp;
}

/*!
 * @brief Return the current offset level
 * @param baseAddr - cmp base pointer
 * @return - offset level
 *  CMP_LEVEL_OFFSET_0
 *  CMP_LEVEL_OFFSET_1
 * Implements : CMP_HAL_GetOffset_Activity
 */
static inline cmp_offset_t CMP_HAL_GetOffset(const CMP_Type* baseAddr)
{
    return (cmp_offset_t)(((baseAddr->C0) >> CMP_C0_OFFSET_SHIFT) & 1U);
}

/*!
 * @brief Set the offset level
 * @param baseAddr - cmp base pointer
 * @param to_set - offset level
 *  CMP_LEVEL_OFFSET_0
 *  CMP_LEVEL_OFFSET_1
 * @return - void
 * Implements : CMP_HAL_SetOffset_Activity
 */
static inline void CMP_HAL_SetOffset(CMP_Type* baseAddr, cmp_offset_t to_set)
{
    uint32_t tmp = baseAddr->C0;
    tmp &= ~(CMP_C0_OFFSET_MASK);
    tmp |= CMP_C0_OFFSET(to_set);
    baseAddr->C0 = tmp;
}

/*!
 * @brief Return the current hysteresis level
 * @param baseAddr - cmp base pointer
 * @return - current hysteresis level
 *  CMP_LEVEL_HYS_0
 *  CMP_LEVEL_HYS_1
 *  CMP_LEVEL_HYS_2
 *  CMP_LEVEL_HYS_3
 * Implements : CMP_HAL_GetHysteresis_Activity
 */
static inline cmp_hysteresis_t CMP_HAL_GetHysteresis(const CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C0;
    tmp = (tmp & CMP_C0_HYSTCTR_MASK) >> CMP_C0_HYSTCTR_SHIFT;
    return ( cmp_hysteresis_t ) (tmp);
}

/*!
 * @brief Set the hysteresis level
 * @param baseAddr - cmp base pointer
 * @param to_set - hysteresis level
 *  CMP_LEVEL_HYS_0
 *  CMP_LEVEL_HYS_1
 *  CMP_LEVEL_HYS_2
 *  CMP_LEVEL_HYS_3
 * @return - void
 * Implements : CMP_HAL_SetHysteresis_Activity
 */
static inline void CMP_HAL_SetHysteresis(CMP_Type* baseAddr, cmp_hysteresis_t to_set)
{
    uint32_t tmp = baseAddr->C0;
    tmp &= ~(CMP_C0_HYSTCTR_MASK);
    tmp |= CMP_C0_HYSTCTR(to_set);
    baseAddr->C0 = tmp;
}

/*!
 * @brief Set if the DAC output is enabled to go outside of this block
 * @param baseAddr - cmp base pointer
 * @param to_set - DAC output state
 *  true - DAC output go outside of DAC block(to packaged pin)
 *  false - DAC output doesn't go outside of DAC block(to packaged pin)
 * @return - void
 * Implements : CMP_HAL_SetDACOutputState_Activity
 */
static inline void CMP_HAL_SetDACOutputState(CMP_Type* baseAddr, bool to_set)
{
    (baseAddr->C0) = ((baseAddr->C0) & (~(CMP_C0_EN_MASK))) | ((uint32_t)to_set << CMP_C0_EN_SHIFT);
}

/*!
 * @brief Get if the DAC output is enabled to go outside of this block
 * @param baseAddr - cmp base pointer
 * @param to_set - DAC output state
 *  true - DAC output go outside of DAC block(to packaged pin)
 *  false - DAC output doesn't go outside of DAC block(to packaged pin)
 * @return - void
  * Implements : CMP_HAL_GetDACOutputState_Activity
 */
static inline bool CMP_HAL_GetDACOutputState(const CMP_Type* baseAddr)
{
    return (bool)(((baseAddr->C0) & (CMP_C0_EN_MASK)) >> (uint32_t)CMP_C0_EN_SHIFT);
}

/*!
 * @brief Return the current source for positive port of the comparator
 * @param baseAddr - cmp base pointer
 * @return - signal source
 *  CMP_DAC
 *  CMP_MUX
 * Implements : CMP_HAL_GetPositivePortInput_Activity
 */
static inline cmp_port_mux_t CMP_HAL_GetPositivePortInput(const CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C1;
    tmp = (tmp & CMP_C1_INPSEL_MASK) >> CMP_C1_INPSEL_SHIFT;
    return ( cmp_port_mux_t ) (tmp);
}

/*!
 * @brief Set the source for positive port of the comparator
 * @param baseAddr cmp base pointer
 * @param to_set - signal source
 *  CMP_DAC
 *  CMP_MUX
 * @return - void
 * Implements : CMP_HAL_SetPositivePortInput_Activity
 */
static inline void CMP_HAL_SetPositivePortInput(CMP_Type* baseAddr, cmp_port_mux_t to_set)
{
    uint32_t tmp = baseAddr->C1;
    tmp &= ~(CMP_C1_INPSEL_MASK);
    tmp |= CMP_C1_INPSEL(to_set);
    baseAddr->C1 = tmp;
}

/*!
 * @brief Return the current source for negative port of the comparator
 * @param baseAddr - cmp base pointer
 * @return - signal source
 *  CMP_DAC
 *  CMP_MUX
 * Implements : CMP_HAL_GetNegativePortInput_Activity
 */
static inline cmp_port_mux_t CMP_HAL_GetNegativePortInput(const CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C1;
    tmp = (tmp & CMP_C1_INNSEL_MASK) >> CMP_C1_INNSEL_SHIFT;
    return ( cmp_port_mux_t ) (tmp);
}

/*!
 * @brief Set the source for negative port of the comparator
 * @param baseAddr - cmp base pointer
 * @param to_set - signal source
 *  CMP_DAC
 *  CMP_MUX
 * @return - void
 * Implements : CMP_HAL_SetNegativePortInput_Activity
 */
static inline void CMP_HAL_SetNegativePortInput(CMP_Type* baseAddr, cmp_port_mux_t to_set)
{
    uint32_t tmp = baseAddr->C1;
    tmp &= ~(CMP_C1_INNSEL_MASK);
    tmp |= CMP_C1_INNSEL(to_set);
    baseAddr->C1 = tmp;
}

/*!
 * @brief Return which channels are used for round-robin checker
 * @param baseAddr - cmp base pointer
 * @return - channels states, one bite for each channel state 
 * |---------|---------|-----|---------|---------|
 * |CH7_state|CH6_state|.....|CH1_state|CH0_state|
 * |---------|---------|-----|---------|---------|
 * Implements : CMP_HAL_GetRoundRobinChannels_Activity
 */
static inline cmp_ch_list_t CMP_HAL_GetRoundRobinChannels(const CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C1;
    tmp =  tmp & (uint32_t)CMP_ROUND_ROBIN_CHANNELS_MASK;
    return(cmp_ch_list_t) (tmp >> CMP_ROUND_ROBIN_CHANNELS_SHIFT);
}

/*!
 * @brief Set which channels are use for round-robin checker
 * @param baseAddr - cmp base pointer
 * @param to_set - channels states, one bite for each channel state
 * |---------|---------|-----|---------|---------|
 * |CH7_state|CH6_state|.....|CH1_state|CH0_state|
 * |---------|---------|-----|---------|---------|
 * @return - void
 * Implements : CMP_HAL_SetRoundRobinChannels_Activity
 */
static inline void CMP_HAL_SetRoundRobinChannels(CMP_Type* baseAddr, cmp_ch_list_t to_set)
{
    uint32_t tmp = baseAddr->C1;
    tmp &= ~(uint32_t)(CMP_ROUND_ROBIN_CHANNELS_MASK);
    tmp |= (uint32_t)to_set << (uint32_t)CMP_ROUND_ROBIN_CHANNELS_SHIFT;
    baseAddr->C1 = tmp;
}

/*!
 * @brief Verify if the DAC is enabled
 * @param baseAddr - cmp base pointer
 * @return - dac state
 *  true - DAC is enabled
 *  false - DAC is disabled
 * Implements : CMP_HAL_GetDACState_Activity
 */
static inline bool CMP_HAL_GetDACState(const CMP_Type* baseAddr)
{
    return (bool)(((baseAddr->C1) >> CMP_C1_DACEN_SHIFT) & 1U);
}

/*!
 * @brief Set the DAC state (enabled/disabled)
 * @param baseAddr - cmp base pointer
 * @param to_set - DAC state
 *  true - DAC is enabled
 *  false - DAC is disabled
 * @return - void
 * Implements : CMP_HAL_SetDACState_Activity
 */
static inline void CMP_HAL_SetDACState(CMP_Type* baseAddr, bool to_set)
{
    (baseAddr->C1) = ((baseAddr->C1) & (~(CMP_C1_DACEN_MASK))) | ((uint32_t)to_set << CMP_C1_DACEN_SHIFT);
}

/*!
 * @brief Return the current voltage reference
 * @param baseAddr - cmp base pointer
 * @return - voltage referece
 *  CMP_VIN1
 *  CMP_VIN2
 * Implements : CMP_HAL_GetVoltageReference_Activity
 */
static inline cmp_voltage_reference_t CMP_HAL_GetVoltageReference(const CMP_Type* baseAddr)
{
    return (cmp_voltage_reference_t)(((baseAddr->C1) >> CMP_C1_VRSEL_SHIFT) & 1U);
}

/*!
 * @brief Set the voltage reference
 * @param baseAddr - cmp base pointer
 * @param to_set - voltage reference
 *  CMP_VIN1
 *  CMP_VIN2
 * @return - void
 * Implements : CMP_HAL_SetVoltageReference_Activity
 */
static inline void CMP_HAL_SetVoltageReference(CMP_Type* baseAddr, cmp_voltage_reference_t to_set)
{
    (baseAddr->C1) = ((baseAddr->C1) & (~(CMP_C1_VRSEL_SHIFT))) | ((uint32_t)to_set << CMP_C1_VRSEL_SHIFT); 
}

/*!
 * @brief Determine which input is selected for the plus mux
 * @param baseAddr - cmp base pointer
 * @return - channel for the plus mux
 * Implements : CMP_HAL_GetPlusMUXControl_Activity
 */
static inline cmp_ch_number_t CMP_HAL_GetPlusMUXControl(const CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C1;
    tmp = (tmp & CMP_C1_PSEL_MASK) >> CMP_C1_PSEL_SHIFT;
    return ( cmp_ch_number_t ) (tmp);
}

/*!
 * @brief Select input for the plus mux
 * @param baseAddr cmp base pointer
 * @param to_set - channel for the plus mux
 * @return - void
 * Implements : CMP_HAL_SetPlusMuxControl_Activity
 */
static inline void CMP_HAL_SetPlusMuxControl(CMP_Type* baseAddr, cmp_ch_number_t to_set)
{
    uint32_t tmp = baseAddr->C1;
    tmp &= ~(CMP_C1_PSEL_MASK);
    tmp |= CMP_C1_PSEL(to_set);
    baseAddr->C1 = tmp;
}

/*!
 * @brief Determine which input is selected for the minus mux
 * @param baseAddr - cmp base pointer
 * @return - channel for the minus mux
 * Implements : CMP_HAL_GetMinusMUXControl_Activity
 */
static inline cmp_ch_number_t CMP_HAL_GetMinusMUXControl(const CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C1;
    tmp = (tmp & CMP_C1_MSEL_MASK) >> CMP_C1_MSEL_SHIFT;
    return ( cmp_ch_number_t ) (tmp);
}

/*!
 * @brief Select input for the minus mux
 * @param baseAddr - cmp base pointer
 * @param to_set - channel for the minus mux
 * @return - void
 * Implements : CMP_HAL_SetMinusMUXControl_Activity
 */
static inline void CMP_HAL_SetMinusMUXControl(CMP_Type* baseAddr, cmp_ch_number_t to_set)
{
    uint32_t tmp = baseAddr->C1;
    tmp &= ~(CMP_C1_MSEL_MASK);
    tmp |= CMP_C1_MSEL(to_set);
    baseAddr->C1 = tmp;
}

/*!
 * @brief Return the current output voltage level(0-255)
 * @param baseAddr - cmp base pointer
 * @return - voltage level
 * Implements : CMP_HAL_GetVoltage_Activity
 */
static inline uint8_t CMP_HAL_GetVoltage(const CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C1;
    tmp = (tmp & CMP_C1_VOSEL_MASK) >> CMP_C1_VOSEL_SHIFT;
    return ( uint8_t ) (tmp);
}

/*!
 * @brief Set the output voltage level
 * @param baseAddr - cmp base pointer
 * @param to_set - voltage level
 * @return - void
 * Implements : CMP_HAL_SetVoltage_Activity
 */
static inline void CMP_HAL_SetVoltage(CMP_Type* baseAddr, uint8_t to_set)
{
    uint32_t tmp = baseAddr->C1;
    tmp &= ~(CMP_C1_VOSEL_MASK);
    tmp |= CMP_C1_VOSEL(to_set);
    baseAddr->C1 = tmp;
}

/*!
 * @brief Verify if the round robin operation is enabled
 * @param baseAddr -cmp base pointer
 * @return - round-robin operation state
 *  true - round robin operation is enabled
 *  false - round robin operation is disabled
 * Implements : CMP_HAL_GetRoundRobinState_Activity
 */
static inline bool CMP_HAL_GetRoundRobinState(const CMP_Type* baseAddr)
{
    return (bool)(((baseAddr->C2) >> CMP_C2_RRE_SHIFT) & 1U);
}

/*!
 * @brief Set the round robin operation state
 * @param baseAddr cmp base pointer
 * @param to_set - round robin operation state
 *  true - round robin operation is enabled
 *  false - round robin operation is disabled
 * @return - void
 * Implements : CMP_HAL_SetRoundRobinState_Activity
 */
static inline void CMP_HAL_SetRoundRobinState(CMP_Type* baseAddr, bool to_set)
{
    (baseAddr->C2) = ((baseAddr->C2) &(~(CMP_C2_RRE_MASK))) | ((uint32_t)to_set << CMP_C2_RRE_SHIFT);
}

/*!
 * @brief Verify if the round robin interrupt is enabled
 * @param baseAddr - cmp base pointer
 * @return - round-robin interrupt state
 *  true - round robin interrupt is enabled
 *  false - round robin interrupt is disabled
 * Implements : CMP_HAL_GetRoundRobinInterruptState_Activity
 */
static inline bool CMP_HAL_GetRoundRobinInterruptState(const CMP_Type* baseAddr)
{
    return (bool)(((baseAddr->C2) >> CMP_C2_RRIE_SHIFT) & 1U);
}

/*!
 * @brief Set the round robin interrupt state
 * @param baseAddr - cmp base pointer
 * @param to_set - round robin interrupt state
 *  true - round robin interrupt is enabled
 *  false - round robin interrupt is disabled
 * @return - void
 * Implements : CMP_HAL_SetRoundRobinInterruptState_Activity
 */
static inline void CMP_HAL_SetRoundRobinInterruptState(CMP_Type* baseAddr, bool to_set)
{
    (baseAddr->C2) = ((baseAddr->C2) & (~(CMP_C2_RRIE_SHIFT))) | ((uint32_t)to_set << CMP_C2_RRIE_SHIFT);
}

/*!
 * @brief Return the port fixed for round-robin operation
 * @param baseAddr - cmp base pointer
 * @return - fixed port
 * Implements : CMP_HAL_GetFixedPort_Activity
 */
static inline cmp_fixed_port_t CMP_HAL_GetFixedPort(const CMP_Type* baseAddr)
{
    return (cmp_fixed_port_t)(((baseAddr->C2) >> CMP_C2_FXMP_SHIFT) & 1U);
}

/*!
 * @brief Set the fixed port for round-robin operation
 * @param baseAddr - cmp base pointer
 * @param to_set - fixed port
 *  CMP_PLUS_FIXED
 *  CMP_MINUS_FIXED
 * @return - void
 * Implements : CMP_HAL_SetFixedPort_Activity
 */
static inline void CMP_HAL_SetFixedPort(CMP_Type* baseAddr, cmp_fixed_port_t to_set)
{
    (baseAddr->C2) = ((baseAddr->C2) & (~(CMP_C2_FXMP_MASK))) | ((uint32_t)to_set << CMP_C2_FXMP_SHIFT);
}

/*!
 * @brief Return which channel is selected for fixed mux port(as fixed reference)
 * @param baseAddr - cmp base pointer
 * @return - fixed channel 
 *  CMP_PLUS_FIXED
 *  CMP_MINUS_FIXED
 * Implements : CMP_HAL_GetFixedChannel_Activity
 */
static inline cmp_ch_number_t CMP_HAL_GetFixedChannel(const CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C2;
    tmp = (tmp & CMP_C2_FXMXCH_MASK) >> CMP_C2_FXMXCH_SHIFT;
    return ( cmp_ch_number_t ) (tmp);
}

/*!
 * @brief Set which channel is used as the fixed reference input for the fixed mux port
 * @param baseAddr - cmp base pointer
 * @param to_set - fixed channel
 * @return - void
 * Implements : CMP_HAL_SetFixedChannel_Activity
 */
static inline void CMP_HAL_SetFixedChannel(CMP_Type* baseAddr, cmp_ch_number_t to_set)
{
    uint32_t tmp = baseAddr->C2;
    tmp &= ~(CMP_C2_FXMXCH_MASK);
    tmp |= CMP_C2_FXMXCH(to_set);
    baseAddr->C2 = tmp;
}

/*!
 * @brief Return all input changed flags
 * @param baseAddr - cmp base pointer
 * @return - flags status
 * |--------|--------|-----|--------|--------|
 * |CH7_flag|CH6_flag|.....|CH1_flag|CH0_flag|
 * |--------|--------|-----|--------|--------|
 * Implements : CMP_HAL_GetInputChangedFlags_Activity
 */
static inline cmp_ch_list_t CMP_HAL_GetInputChangedFlags(const CMP_Type* baseAddr)
{   
    uint32_t tmp = baseAddr->C2;
    tmp = (tmp & (uint32_t)CMP_INPUT_FLAGS_MASK) >> CMP_INPUT_FLAGS_SHIFT;
    return ( cmp_ch_list_t ) (tmp);
}

/*!
 * @brief Clear all input changed flags
 * @param baseAddr - cmp base pointer
 * @return - void
 * Implements : CMP_HAL_ClearInputChangedFlags_Activity
 */
static inline void CMP_HAL_ClearInputChangedFlags(CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C2;
    tmp |= (uint32_t)CMP_INPUT_FLAGS_MASK;
    baseAddr->C2 = tmp;
    #ifdef ERRATA_E9005
        (void)baseAddr->C2;
    #endif    
}

/*!
 * @brief Return how many round-robin clock cycles takes sampling
 * @param baseAddr - cmp base pointer
 * @return - number of sample clocks
 * Implements : CMP_HAL_GetRoundRobinSamplesNumber_Activity
 */
static inline uint8_t CMP_HAL_GetRoundRobinSamplesNumber(const CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C2;
    tmp = (tmp & CMP_C2_NSAM_MASK) >> CMP_C2_NSAM_SHIFT;
    return ( uint8_t ) (tmp);
}

/*!
 * @brief Set how many round-robin clock cycles takes sampling
 * @param baseAddr - cmp base pointer
 * @param to_set - number of sample clocks(min value 0, max value 3)
 * @return - void
 * Implements : CMP_HAL_SetRoundRobinSamplesNumber_Activity
 */
static inline void CMP_HAL_SetRoundRobinSamplesNumber(CMP_Type* baseAddr, uint8_t to_set)
{
    uint32_t tmp = baseAddr->C2;
    tmp &= ~(CMP_C2_NSAM_MASK);
    tmp |= CMP_C2_NSAM(to_set);
    baseAddr->C2 = tmp;
}

/*!
 * @brief Return the comparator and DAC initialization delay
 * @param baseAddr - cmp base pointer
 * @return - delay(round-robin clock period)
 * Implements : CMP_HAL_GetInitDelay_Activity
 */
static inline uint8_t CMP_HAL_GetInitDelay(const CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C2;
    tmp = (tmp & CMP_C2_INITMOD_MASK) >> CMP_C2_INITMOD_SHIFT;
    return ( uint8_t ) (tmp);
}

/*!
 * @brief Set the comparator and  DAC initialization delay
 * @param baseAddr - cmp base pointer
 * @param to_set - delay (min value 0, max value 63)
 * @return - void
 * Implements : CMP_HAL_SetInitDelay_Activity
 */
static inline void CMP_HAL_SetInitDelay(CMP_Type* baseAddr, uint8_t to_set)
{
    uint32_t tmp = baseAddr->C2;
    tmp &= ~(CMP_C2_INITMOD_MASK);
    tmp |= CMP_C2_INITMOD(to_set);
    baseAddr->C2 = tmp;
}

/*!
 * @brief Return last input comparison results for all channels
 * @param baseAddr - cmp base pointer
 * @return - comparison results
 * Implements : CMP_HAL_GetLastComparisonResult_Activity
 */
static inline cmp_ch_list_t CMP_HAL_GetLastComparisonResult(const CMP_Type* baseAddr)
{
    uint32_t tmp = baseAddr->C2;
    tmp = (tmp & CMP_C2_ACOn_MASK) >> CMP_C2_ACOn_SHIFT;
    return ( cmp_ch_list_t ) (tmp);
}

/*!
 * @brief Defines the pre-set state of input channels.
 * @param baseAddr cmp base pointer
 * @param to_set - state
 * @return void
 * Implements : CMP_HAL_SetPresetState_Activity 
 */
static inline void CMP_HAL_SetPresetState(CMP_Type* baseAddr, cmp_ch_list_t to_set)
{
    uint32_t tmp = baseAddr->C2;
    tmp &= ~(CMP_C2_ACOn_MASK);
    tmp |= CMP_C2_ACOn(to_set);
    baseAddr->C2 = tmp;
}

/*! @}*/
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_CMP_HAL_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

