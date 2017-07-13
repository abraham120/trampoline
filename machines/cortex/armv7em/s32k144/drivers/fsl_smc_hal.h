/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
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

#if !defined(FSL_SMC_HAL_H)
#define FSL_SMC_HAL_H

#include "fsl_device_registers.h"
#include <stdbool.h>

  /*! @file fsl_smc_hal.h */

/*!
 * @defgroup fsl_smc_hal System Mode Controller (SMC)
 * @ingroup power_manager
 * @brief This module covers the functionality of the System Mode Controller (SMC) peripheral.
 * <p>
 *  SMC HAL provides the API for reading and writing register bit-fields belonging to the SMC module.
 * </p>
 * <p>
 *  For higher-level functionality, use the Power Manager driver.
 * </p>
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 *  @brief Power Modes
 *  Implements: power_modes_t_Class
 */
typedef enum
{
    POWER_MODE_RUN,                            /*!< RUN power mode */
    POWER_MODE_WAIT,                           /*!< WAIT power mode */
    POWER_MODE_STOP,                           /*!< STOP power mode */
    POWER_MODE_VLPR,                           /*!< VLPR power mode */
    POWER_MODE_VLPW,                           /*!< VLPW power mode */
    POWER_MODE_VLPS,                           /*!< VLPS power mode */
    POWER_MODE_HSRUN,                          /*!< HSRUN power mode */
    POWER_MODE_MAX                             /*!< The total number of power modes */
} power_modes_t;

/*!
 *  @brief Error code definition for the system mode controller manager APIs
 *  Implements: smc_hal_error_code_t_Class
 */
typedef enum
{
    SMC_HAL_SUCCESS,                           /*!< Success */
    SMC_HAL_NO_SUCH_MODE_NAME,                 /*!< Cannot find the mode name specified*/
    SMC_HAL_ALREADY_IN_THE_STATE,              /*!< Already in the required state*/
    SMC_HAL_NOT_ALLOWED_MODE,                  /*!< The specified mode is not allowed */
    SMC_HAL_TIMEOUT_MODE_CHANGE,               /*!< Power mode change operation failed */
    SMC_HAL_FAILED                             /*!< Unknown error, operation failed*/
} smc_hal_error_code_t;

/*!
 *  @brief Power Modes in PMSTAT
 *  Implements: power_mode_stat_t_Class
 */
typedef enum
{
    STAT_RUN     = 0x01,             /*!< 0000_0001 - Current power mode is RUN*/
    STAT_STOP    = 0x02,             /*!< 0000_0010 - Current power mode is STOP*/
    STAT_VLPR    = 0x04,             /*!< 0000_0100 - Current power mode is VLPR*/
    STAT_VLPW    = 0x08,             /*!< 0000_1000 - Current power mode is VLPW*/
    STAT_VLPS    = 0x10,             /*!< 0001_0000 - Current power mode is VLPS*/
    STAT_HSRUN   = 0x80,              /*!< 1000_0000 - Current power mode is HSRUN*/
    STAT_INVALID = 0xFF               /*!< 1111_1111 - Non-existing power mode*/
} power_mode_stat_t;

/*!
 *  @brief Power Modes Protection
 *  Implements: power_modes_protect_t_Class
 */
typedef enum
{
    ALLOW_HSRUN,                    /*!< Allow High Speed Run mode*/
    ALLOW_VLP,                      /*!< Allow Very-Low-Power Modes*/
    ALLOW_MAX
} power_modes_protect_t;

/*!
 *  @brief Run mode definition
 *  Implements: smc_run_mode_t_Class
 */
typedef enum
{
    SMC_RUN,                                /*!< normal RUN mode*/
    SMC_RESERVED_RUN,
    SMC_VLPR,                               /*!< Very-Low-Power RUN mode*/
    SMC_HSRUN                               /*!< High Speed Run mode (HSRUN)*/
} smc_run_mode_t;

/*!
 *  @brief Stop mode definition
 *  Implements: smc_stop_mode_t_Class
 */
typedef enum
{
    SMC_STOP            = 0U,    /*!< Normal STOP mode*/
    SMC_RESERVED_STOP1  = 1U,    /*!< Reserved*/
    SMC_VLPS            = 2U     /*!< Very-Low-Power STOP mode*/
} smc_stop_mode_t;

/*!
 *  @brief STOP option
 *  Implements: smc_stop_option_t_Class
 */
typedef enum 
{
    SMC_STOP_RESERVED = 0x00,               /*!< Reserved stop mode */ 
    SMC_STOP1         = 0x01,               /*!< Stop with both system and bus clocks disabled */ 
    SMC_STOP2         = 0x02                /*!< Stop with system clock disabled and bus clock enabled */

} smc_stop_option_t;

/*!
 *  @brief Power mode protection configuration
 *  Implements: smc_power_mode_protection_config_t_Class
 */
typedef struct
{
    bool                vlpProt;            /*!< VLP protect*/
#if FSL_FEATURE_SMC_HAS_HIGH_SPEED_RUN_MODE 
    bool                hsrunProt;          /*!< HSRUN protect */
#endif
} smc_power_mode_protection_config_t;

/*!
 *  @brief Power mode control configuration used for calling the SMC_SYS_SetPowerMode API
 *  Implements: smc_power_mode_config_t_Class
 */
typedef struct {
    power_modes_t       powerModeName;      /*!< Power mode(enum), see power_modes_t */
    bool                stopOption;         /*!< If STOPO option is needed */
    smc_stop_option_t   stopOptionValue;    /*!< STOPO option(enum), see smc_stop_option_t */
} smc_power_mode_config_t;

/*!
 *  @brief SMC module version number
 *  Implements: smc_version_info_t_Class
 */
typedef struct
{
    uint32_t  majorNumber;       /**< Major Version Number */
    uint32_t  minorNumber;       /**< Minor Version Number */
    uint32_t  featureNumber;     /**< Feature Specification Number */
} smc_version_info_t;



/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name System mode controller APIs*/
/*@{*/

/*!
 * @brief Get the version of the SMC module
 * 
 * @param[in]  baseAddr  base address of the SMC module
 * @param[out] versionInfo  Device Version Number
 */
void SMC_HAL_GetVersion(const SMC_Type* const baseAddr, smc_version_info_t* const versionInfo);

/*!
 * @brief Configures the power mode.
 *
 * This function configures the power mode control for both run, stop, and
 * stop sub mode if needed. Also it configures the power options for a specific
 * power mode. An application should follow the proper procedure to configure and 
 * switch power modes between  different run and stop modes. For proper procedures 
 * and supported power modes, see an appropriate chip reference
 * manual. See the smc_power_mode_config_t for required
 * parameters to configure the power mode and the supported options. Other options
 * may need to be individually configured through the HAL driver. See the HAL driver
 * header file for details.
 *
 * @param baseAddr  Base address for current SMC instance.
 * @param powerModeConfig Power mode configuration structure smc_power_mode_config_t 
 * @return errorCode SMC error code
 */
smc_hal_error_code_t SMC_HAL_SetPowerMode(SMC_Type* const baseAddr, 
                                                                                    const smc_power_mode_config_t* const powerModeConfig);

/*!
 * @brief Configures all power mode protection settings.
 *
 * This function  configures the power mode protection settings for
 * supported power modes in the specified chip family. The available power modes
 * are defined in the smc_power_mode_protection_config_t. An application should provide
 * the protect settings for all supported power modes on the chip. This
 * should be done at an early system level initialization stage. See the reference manual
 * for details. This register can only write once after the power reset. If the user has 
 * only a single option to set,
 * either use this function or use the individual set function.
 * 
 * 
 * @param[in] baseAddr  Base address for current SMC instance.
 * @param[in] protectConfig Configurations for the supported power mode protect settings
 *                      - See smc_power_mode_protection_config_t for details.
 */
void SMC_HAL_SetProtectionMode(SMC_Type* const baseAddr, 
                                                              const smc_power_mode_protection_config_t* const protectConfig);
/*!
 * @brief Gets the the current power mode protection setting.
 *
 * This function  gets the current power mode protection settings for
 * a specified power mode.
 *
 * @param baseAddr[in]  Base address for current SMC instance.
 * @param protect[in]   Power mode to set for protection
 * @return state  Status of the protection setting
 *                - true: Allowed
 *                - false: Not allowed
*/
bool SMC_HAL_GetProtectionMode(const SMC_Type* const baseAddr, const power_modes_protect_t protect);

/*!
 * @brief Configures the the RUN mode control setting.
 *
 * This function  sets the run mode settings, for example, normal run mode,
 * very lower power run mode, etc. See the smc_run_mode_t for supported run
 * mode on the chip family and the reference manual for details about the 
 * run mode.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @param[in] runMode   Run mode setting defined in smc_run_mode_t
 * Implements SMC_HAL_SetRunModeControl_Activity
 */
static inline void SMC_HAL_SetRunModeControl(SMC_Type* const baseAddr, const smc_run_mode_t runMode)
{
    uint32_t regValue = baseAddr->PMCTRL;
    regValue &= ~(SMC_PMCTRL_RUNM_MASK);
    regValue |= SMC_PMCTRL_RUNM(runMode);
    baseAddr->PMCTRL = regValue;
}

/*!
 * @brief Gets  the current RUN mode configuration setting.
 *
 * This function  gets the run mode settings. See the smc_run_mode_t 
 * for a supported run mode on the chip family and the reference manual for 
 * details about the run mode.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @return setting Run mode configuration setting
 * Implements SMC_HAL_GetRunModeControl_Activity
 */
static inline smc_run_mode_t SMC_HAL_GetRunModeControl(const SMC_Type* const baseAddr)
{
    smc_run_mode_t retValue;
    uint32_t regValue = baseAddr->PMCTRL;
    regValue = (regValue & SMC_PMCTRL_RUNM_MASK) >> SMC_PMCTRL_RUNM_SHIFT;
    switch(regValue)
    {
        case 0UL:
            retValue = SMC_RUN;
            break;
        case 1UL:
            retValue = SMC_RESERVED_RUN;
            break;
        case 2UL:
            retValue = SMC_VLPR;
            break;
        case 3UL:
        default:
            retValue = SMC_HSRUN;
            break;
    }
    return retValue;
}

/*!
 * @brief Configures  the STOP mode control setting.
 *
 * This function  sets the stop mode settings, for example, normal stop mode,
 * very lower power stop mode, etc. See the  smc_stop_mode_t for supported stop
 * mode on the chip family and the reference manual for details about the 
 * stop mode.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @param[in] stopMode  Stop mode defined in smc_stop_mode_t
 * Implements SMC_HAL_SetStopModeControl_Activity
 */
static inline void SMC_HAL_SetStopModeControl(SMC_Type* const baseAddr, const smc_stop_mode_t stopMode)
{
    uint32_t regValue = baseAddr->PMCTRL;
    regValue &= ~(SMC_PMCTRL_STOPM_MASK);
    regValue |= SMC_PMCTRL_STOPM(stopMode);
    baseAddr->PMCTRL = regValue;
}

/*!
 * @brief Checks whether the last very low power stop sequence has been aborted.
 *
 * This function checks whether the last very low power stop sequence has been aborted.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @return aborted      Aborted or not
 * Implements SMC_HAL_GetVlpsaModeControl_Activity
 */
static inline smc_stop_mode_t SMC_HAL_GetVlpsaModeControl(const SMC_Type* const baseAddr)
{
    smc_stop_mode_t retValue;
    uint32_t regValue = baseAddr->PMCTRL;
    regValue = (regValue & SMC_PMCTRL_VLPSA_MASK) >> SMC_PMCTRL_VLPSA_SHIFT;
    switch(regValue)  {
        case 0UL:
            retValue = SMC_STOP;
            break;
        case 2UL:
            retValue = SMC_VLPS;
            break;
        case 1UL:
        /* pass-through */
        default:
            retValue = SMC_RESERVED_STOP1;
            break;
    }
    return retValue;
}

/*!
 * @brief Gets the current STOP mode control settings.
 *
 * This function  gets the stop mode settings, for example, normal stop mode,
 * very lower power stop mode, etc. See the  smc_stop_mode_t for supported stop
 * mode on the chip family and the reference manual for details about the 
 * stop mode.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @return setting Current stop mode configuration setting
 * Implements SMC_HAL_GetStopModeControl_Activity
 */
static inline smc_stop_mode_t SMC_HAL_GetStopModeControl(const SMC_Type* const baseAddr)
{
    smc_stop_mode_t retValue;
    uint32_t regValue = baseAddr->PMCTRL;
    regValue = (regValue & SMC_PMCTRL_STOPM_MASK) >> SMC_PMCTRL_STOPM_SHIFT;
    switch(regValue)  {
        case 0UL:
            retValue = SMC_STOP;
            break;
        case 2UL:
            retValue = SMC_VLPS;
            break;
        case 1UL:
        /* pass-through */
        default:
            retValue = SMC_RESERVED_STOP1;
            break;
    }
    return retValue;
}

#if FSL_FEATURE_SMC_HAS_STOPO
/*!
 * @brief Configures the STOPO (Stop Option).
 *
 * It controls the type of the stop operation when STOPM=STOP. When entering Stop mode
 * from RUN mode, the PMC, SCG and flash remain fully powered, allowing the device to
 * wakeup almost instantaneously at the expense of higher power consumption. In STOP2,
 * only system clocks are gated allowing peripherals running on bus clock to remain fully
 * functional. In STOP1, both system and bus clocks are gated.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @param[in] option STOPO option setting defined in smc_stop_option_t
 * Implements SMC_HAL_SetStopOption_Activity
 */
static inline void SMC_HAL_SetStopOption(SMC_Type* const baseAddr, const smc_stop_option_t option)
{
    uint32_t regValue = baseAddr->STOPCTRL;
    regValue &= ~(SMC_STOPCTRL_STOPO_MASK);
    regValue |= SMC_STOPCTRL_STOPO(option);
    baseAddr->STOPCTRL = regValue;
}


/*!
 * @brief Gets the configuration of the STOPO option.
 *
 * This function  gets the current STOPO option setting. See the  configuration
 * function for more details.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @return option Current STOPO option setting
 * Implements SMC_HAL_GetStopOption_Activity
 */
static inline smc_stop_option_t SMC_HAL_GetStopOption(const SMC_Type* const baseAddr)
{
    smc_stop_option_t retValue;
    uint32_t regValue = baseAddr->STOPCTRL;
    regValue = (regValue & SMC_STOPCTRL_STOPO_MASK) >> SMC_STOPCTRL_STOPO_SHIFT;
    switch(regValue)  {
        case 1UL:
            retValue = SMC_STOP1;
            break;
        case 2UL:
            retValue = SMC_STOP2;
            break;
        case 0UL:
        /* Pass-through */
        default:
            retValue = SMC_STOP_RESERVED;
            break;
    }
    return retValue;
}

#endif

/*!
 * @brief Gets the current power mode stat.
 *
 * This function  returns the current power mode stat. Once application
 * switches the power mode, it should always check the stat to check whether it 
 * runs into the specified mode or not. An application  should  check 
 * this mode before switching to a different mode. The system  requires that
 * only certain modes can switch to other specific modes. See the 
 * reference manual for details and the _power_mode_stat for information about
 * the power stat.
 *
 * @param[in] baseAddr  Base address for current SMC instance.
 * @return stat  Current power mode stat
 * Implements SMC_HAL_GetPowerModeStatus_Activity
 */
static inline power_mode_stat_t SMC_HAL_GetPowerModeStatus(const SMC_Type* const baseAddr)
{
    power_mode_stat_t retValue;
    uint32_t regValue = baseAddr->PMSTAT;
    regValue = (regValue & SMC_PMSTAT_PMSTAT_MASK) >> SMC_PMSTAT_PMSTAT_SHIFT;    
    switch(regValue)  {
        case 1UL:
            retValue = STAT_RUN;
            break;
        case 2UL:
            retValue = STAT_STOP;
            break;
        case 4UL:
            retValue = STAT_VLPR;
            break;
        case 8UL:
            retValue = STAT_VLPW;
            break;
        case 16UL:
            retValue = STAT_VLPS;
            break;
        case 128UL:
            retValue = STAT_HSRUN;
            break;
        case 255UL:
        default:
            retValue = STAT_INVALID;
            break;
    }
    return retValue;
}



/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* FSL_SMC_HAL_H*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

