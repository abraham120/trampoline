/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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
/*!
 * @file fsl_ftm_driver.h
 *
 */

#ifndef FSL_FTM_DRIVER_H
#define FSL_FTM_DRIVER_H

#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_ftm_hal.h"

/*!
 * @addtogroup ftm_driver FTM Driver
 * @ingroup ftm
 * @brief FlexTimer Peripheral Driver.
 * @addtogroup ftm_driver
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of base addresses for FTM instances. */
extern FTM_Type * const g_ftmBase[FTM_INSTANCE_COUNT];

/*! @brief Interrupt vectors for the FTM peripheral. */
extern const IRQn_Type g_ftmIrqId[FTM_IRQS_ARR_COUNT][FTM_IRQS_CH_COUNT];
extern const IRQn_Type g_ftmFaultIrqId[FTM_IRQS_ARR_COUNT];
extern const IRQn_Type g_ftmOverflowIrqId[FTM_IRQS_ARR_COUNT];
extern const IRQn_Type g_ftmReloadIrqId[FTM_IRQS_ARR_COUNT];

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Max value for PWM duty cycle */
#define FTM_MAX_DUTY_CYCLE      (0x8000U)
/*! @brief Shift value which converts duty to ticks */
#define FTM_DUTY_TO_TICKS_SHIFT (15U)

/*!
 * @brief FlexTimer status
 *
 * Implements: ftm_status_t_Class
 */
typedef enum
{
    FTM_STATUS_SUCCESS  = 0U,   /*!< FTM success status.*/
    FTM_STATUS_ERROR    = 1U    /*!< FTM error status.*/
} ftm_status_t;

/*!
 * @brief FTM status
 *
 * Implements: ftm_input_op_mode_t_Class
 */
typedef enum
{
    FTM_EDGE_DETECT         = 0U,    /*!< FTM edge detect.*/
    FTM_SIGNAL_MEASUREMENT  = 1U,    /*!< FTM signal measurement.*/
    FTM_NO_OPERATION        = 2U     /*!< FTM no operation.*/
} ftm_input_op_mode_t;

/*!
 * @brief FlexTimer input capture measurement type for dual edge input capture
 *
 * Implements: ftm_signal_measurement_mode_t_Class
 */
typedef enum
{
    FTM_NO_MEASUREMENT      = 0x00U,    /*!< No measurement */
    FTM_PERIOD_MEASUREMENT  = 0x01U,    /*!< Period measurement */
    FTM_DUTY_MEASUREMENT    = 0x02U     /*!< Duty measurement */
} ftm_signal_measurement_mode_t;

/*!
 * @brief FlexTimer input capture type of the next output compare value
 *
 * Implements: ftm_output_compare_update_t_Class
 */
typedef enum
{
    FTM_RELATIVE_VALUE = 0x00U,     /*!< Next compared value is relative to current value */
    FTM_ABSOLUTE_VALUE = 0x01U      /*!< Next compared value is absolute */
} ftm_output_compare_update_t;

/*!
 * @brief FlexTimer Configure type of PWM update in the duty cycle or in ticks
 *
 * Implements: ftm_pwm_update_option_t_Class
 */
typedef enum
{
    FTM_PWM_UPDATE_IN_DUTY_CYCLE = 0x00U,     /*!< The type of PWM update in the duty cycle/pulse */
    FTM_PWM_UPDATE_IN_TICKS      = 0x01U      /*!< The type of PWM update in ticks */
} ftm_pwm_update_option_t;

/*!
 * @brief FlexTimer operation mode
 *
 * Implements: ftm_config_mode_t_Class
 */
typedef enum
{
    FTM_MODE_NOT_INITIALIZED    = 0x00U,    /*!< The driver is not initialized */
    FTM_MODE_INPUT_CAPTURE      = 0x01U,    /*!< Input capture */
    FTM_MODE_OUTPUT_COMPARE     = 0x02U,    /*!< Output compare */
    FTM_MODE_EDGE_ALIGNED_PWM   = 0x03U,    /*!< Edge aligned PWM */
    FTM_MODE_CEN_ALIGNED_PWM    = 0x04U,    /*!< Center aligned PWM */
    FTM_DUAL_EDGE_CAPTURE       = 0x05U,    /*!< Dual edge capture */
    FTM_MODE_QUADRATURE_DECODER = 0x06U,    /*!< Quadrature decoder */
    FTM_MODE_UP_TIMER           = 0x07U,    /*!< Timer with up counter */
    FTM_MODE_UP_DOWN_TIMER      = 0x08U     /*!< timer with up-down counter */
} ftm_config_mode_t;

/*!
 * @brief FlexTimer Mode configuration for output compare mode
 *
 * Implements: ftm_output_compare_mode_t_Class
 */
typedef enum
{
    FTM_DISABLE_OUTPUT  = 0x00U,    /*!< No action on output pin */
    FTM_TOGGLE_ON_MATCH = 0x01U,    /*!< Toggle on match */
    FTM_CLEAR_ON_MATCH  = 0x02U,    /*!< Clear on match */
    FTM_SET_ON_MATCH    = 0x03U     /*!< Set on match */
} ftm_output_compare_mode_t;

/*!
 * @brief Channel event callback function
 *
 * Callback functions are called by the FTM driver in Input Capture mode when an event
 * is detected(change in logical state of a pin or measurement complete)
 */
typedef void (* ftm_channel_event_callback_t)(void * userData);

/*!
 * @brief FlexTimer state structure of the driver
 *
 * Implements: ftm_state_t_Class
 */
typedef struct
{
    ftm_clock_source_t ftmClockSource;                                              /*!< Clock source used by FTM counter */
    ftm_config_mode_t ftmMode;                                                      /*!< Mode of operation for FTM */
    uint16_t ftmPeriod;                                                             /*!< This field is used only in PWM mode to store signal period */
    uint32_t ftmSourceClockFrequency;                                               /*!< The clock frequency is used for counting */
    uint16_t measurementResults[FSL_FEATURE_FTM_CHANNEL_COUNT];                     /*!< This field is used only in input capture mode to store edges time stamps */
    void * channelsCallbacksParams[FSL_FEATURE_FTM_CHANNEL_COUNT];                  /*!< Vector of callbacks  parameters for channels events */
    ftm_channel_event_callback_t channelsCallbacks[FSL_FEATURE_FTM_CHANNEL_COUNT];  /*!< Vector of callbacks for channels events */
} ftm_state_t;

/*!
 * @brief FlexTimer Registers sync parameters
 *        Please don't use software and hardware trigger simultaneously
 * Implements: ftm_pwm_sync_t_Class
 */
typedef struct
{
    bool softwareSync;                          /*!< True - enable software sync,
                                                 *   False - disable software sync */
    bool hardwareSync0;                         /*!< True - enable hardware0 sync,
                                                 *   False - disable hardware0 sync */
    bool hardwareSync1;                         /*!< True - enable hardware1 sync,
                                                 *   False - disable hardware1 sync */
    bool hardwareSync2;                         /*!< True - enable hardware2 sync,
                                                 *   False - disable hardware2 sync */
    bool maxLoadingPoint;                       /*!< True - enable maximum loading point,
                                                 *   False - disable maximum loading point */
    bool minLoadingPoint;                       /*!< True - enable minimum loading point,
                                                 *   False - disable minimum loading point */
    ftm_reg_update_t inverterSync;              /*!< Configures INVCTRL sync */
    ftm_reg_update_t outRegSync;                /*!< Configures SWOCTRL sync */
    ftm_reg_update_t maskRegSync;               /*!< Configures OUTMASK sync */
    ftm_reg_update_t initCounterSync;           /*!< Configures CNTIN sync */
    bool autoClearTrigger;                      /*!< Available only for hardware trigger */
    ftm_pwm_sync_mode_t syncPoint;              /*!< Configure synchronization method
                                                 *   (waiting next loading point or immediate) */
} ftm_pwm_sync_t;

/*!
 * @brief Configuration structure that the user needs to set
 *
 * Implements: ftm_user_config_t_Class
 */
typedef struct
{
    ftm_pwm_sync_t syncMethod;              /*!< Register sync options available in the
                                             *   ftm_sync_method_t enumeration  */
    ftm_config_mode_t ftmMode;              /*!< Mode of operation for FTM */
    ftm_clock_ps_t ftmPrescaler;            /*!< Register prescaler options available in the
                                             *   ftm_clock_ps_t enumeration  */
    ftm_clock_source_t ftmClockSource;      /*!< Select clock source for FTM */
    ftm_bdm_mode_t BDMMode;                 /*!< Select FTM behavior in BDM mode */
    bool isTofIsrEnabled;                   /*!< true: enable interrupt,
                                             *   false: write interrupt is disabled */
} ftm_user_config_t;

/*!
 * @brief FlexTimer driver timer mode configuration structure
 *
 * Implements: ftm_timer_param_t_Class
 */
typedef struct
{
    ftm_config_mode_t mode;                 /*!< FTM mode */
    uint16_t initialValue;                  /*!< Initial counter value */
    uint16_t finalValue;                    /*!< Final counter value */
} ftm_timer_param_t;

/*!
 * @brief FlexTimer driver PWM Fault channel parameters
 *
 * Implements: ftm_pwm_ch_fault_param_t_Class
 */
typedef struct
{
    bool faultChannelEnabled;                   /*!< Fault channel state */
    bool faultFilterEnabled;                    /*!< Fault channel filter state */
    ftm_polarity_t ftmFaultPinPolarity;         /*!< Channel output state on fault */
} ftm_pwm_ch_fault_param_t;

/*!
 * @brief FlexTimer driver PWM Fault parameter
 *
 * Implements: ftm_pwm_fault_param_t_Class
 */
typedef struct
{
    bool pwmOutputStateOnFault;             /*!< Output pin state on fault */
    bool pwmFaultInterrupt;                 /*!< PWM fault interrupt state */
    uint8_t faultFilterValue;               /*!< Fault filter value */
    ftm_fault_mode_t faultMode;             /*!< Fault mode */
    ftm_pwm_ch_fault_param_t ftmFaultChannelParam[FTM_FEATURE_FAULT_CHANNELS]; /*!< Fault channels configuration */
} ftm_pwm_fault_param_t;

/*!
 * @brief FlexTimer driver independent PWM parameter
 *
 * Implements: ftm_independent_ch_param_t_Class
 */
typedef struct
{
    uint8_t hwChannelId;            /*!< Physical hardware channel ID*/
    ftm_polarity_t polarity;        /*!< PWM output polarity */
    uint16_t uDutyCyclePercent;     /*!< PWM pulse width, value should be between
                                     *   0 (0%) to FTM_MAX_DUTY_CYCLE (100%). */
} ftm_independent_ch_param_t;

/*!
 * @brief FlexTimer driver combined PWM parameter
 *
 * Implements: ftm_combined_ch_param_t_Class
 */
typedef struct
{
    uint8_t hwChannelId;                                        /*!< Physical hardware channel ID for channel (n) */
    uint16_t firstEdge;                                         /*!< First edge time. This time is relative to signal period. The value for this parameter is
                                                                 *   between 0 and FTM_MAX_DUTY_CYCLE(0 = 0% from period and FTM_MAX_DUTY_CYCLE = 100% from period) */
    uint16_t secondEdge;                                        /*!< Second edge time. This time is relative to signal period. The value for this parameter is
                                                                 *   between 0 and FTM_MAX_DUTY_CYCLE(0 = 0% from period and FTM_MAX_DUTY_CYCLE = 100% from period) */
    bool deadTime;                                              /*!< Enable/disable dead time for channel */
    ftm_polarity_t mainChannelPolarity;                         /*!< Main channel polarity. For FTM_POLARITY_HIGH first output value is 0 and for
                                                                 *   FTM_POLAIRTY first output value is 1. */
    bool enableSecondChannelOutput;                             /*!< Select if channel (n+1)  output is enabled/disabled */
    ftm_second_channel_polarity_t secondChannelPolarity;        /*!< Select channel (n+1) polarity relative to channel (n) */
} ftm_combined_ch_param_t;

/*!
 * @brief FlexTimer driver PWM parameters
 *
 * Implements: ftm_pwm_param_t_Class
 */
typedef struct
{
    uint8_t nNumIndependentPwmChannels;                                     /*!< Number of independent PWM channels */
    uint8_t nNumCombinedPwmChannels;                                        /*!< Number of combined PWM channels */
    ftm_config_mode_t mode;                                                 /*!< FTM mode */
    uint8_t deadTimeValue;                                                  /*!< Dead time value in [ticks] */
    ftm_deadtime_ps_t deadTimePrescaler;                                    /*!< Dead time prescaler value[ticks] */
    uint32_t uFrequencyHZ;                                                  /*!< PWM period in Hz */
    const ftm_independent_ch_param_t * pwmIndependentChannelConfig;         /*!< Configuration for independent PWM channels */
    const ftm_combined_ch_param_t * pwmCombinedChannelConfig;               /*!< Configuration for combined PWM channels */
    const ftm_pwm_fault_param_t * faultConfig;                              /*!< Configuration for PWM fault */
} ftm_pwm_param_t;

/*!
 * @brief FlexTimer input capture edge mode, rising edge, or falling edge
 *
 * Implements: ftm_edge_alignment_mode_t_Class
 */
typedef enum
{
    FTM_NO_PIN_CONTROL          = 0x00U,       /*!< No trigger */
    FTM_RISING_EDGE             = 0x01U,       /*!< Rising edge trigger */
    FTM_FALLING_EDGE            = 0x02U,       /*!< Falling edge trigger */
    FTM_BOTH_EDGES              = 0x03U        /*!< Rising and falling edge trigger */
} ftm_edge_alignment_mode_t;

/*!
 * @brief FlexTimer driver Input capture parameters for each channel
 *
 * Implements: ftm_input_ch_param_t_Class
 */
typedef struct
{
    uint8_t hwChannelId;                                /*!< Physical hardware channel ID*/
    ftm_input_op_mode_t inputMode;                      /*!< FlexTimer module mode of operation  */
    ftm_edge_alignment_mode_t edgeAlignement;           /*!< Edge alignment Mode for signal measurement*/
    ftm_signal_measurement_mode_t measurementType;      /*!< Measurement Mode for signal measurement*/
    uint16_t filterValue;                               /*!< Filter Value */
    bool filterEn;                                      /*!< Input capture filter state */
    bool continuousModeEn;                              /*!< Continuous measurement state */
    void * channelsCallbacksParams;                     /*!< Vector of callbacks  parameters for channels events */
    ftm_channel_event_callback_t channelsCallbacks;     /*!< Vector of callbacks for channels events */
} ftm_input_ch_param_t;

/*!
 * @brief FlexTimer driver input capture parameters
 *
 * Implements: ftm_input_param_t_Class
 */
typedef struct
{
    uint8_t nNumChannels;                           /*!< Number of input capture channel used */
    uint16_t nMaxCountValue;                        /*!< Maximum counter value. Min value is 0 for this mode */
    const ftm_input_ch_param_t * inputChConfig;     /*!< Input capture channels configuration */
} ftm_input_param_t;

/*!
 * @brief FlexTimer driver PWM parameters
 *
 * Implements: ftm_output_cmp_ch_param_t_Class
 */
typedef struct
{
    uint8_t hwChannelId;                        /*!< Physical hardware channel ID*/
    ftm_output_compare_mode_t chMode;           /*!< Channel output mode*/
    uint16_t comparedValue;                     /*!< The compared value */
} ftm_output_cmp_ch_param_t;

/*!
 * @brief FlexTimer driver PWM parameters
 *
 * Implements: ftm_output_cmp_param_t_Class
 */
typedef struct
{
    uint8_t nNumOutputChannels;                             /*!< Number of output compare channels */
    ftm_config_mode_t mode;                                 /*!< FlexTimer PWM operation mode */
    uint16_t maxCountValue;                                 /*!< Maximum count value in ticks */
    const ftm_output_cmp_ch_param_t * outputChannelConfig;  /*!< Output compare channels configuration */
} ftm_output_cmp_param_t;

/*!
 * @brief FlexTimer quadrature decoder channel parameters
 *
 * Implements: ftm_phase_params_t_Class
 */
typedef struct
{
    bool phaseInputFilter;                          /*!< True: disable phase filter,
                                                     *   False: enable phase filter */
    uint8_t phaseFilterVal;                         /*!< Filter value (if input filter is enabled)*/
    ftm_quad_phase_polarity_t phasePolarity;        /*!< Phase polarity */
} ftm_phase_params_t;

/*!
 * @brief FTM quadrature configure structure
 *
 * Implements: ftm_quad_decode_config_t_Class
 */
typedef struct
{
    ftm_quad_decode_mode_t mode;        /*!< FTM_QUAD_PHASE_ENCODE or FTM_QUAD_COUNT_AND_DIR */
    uint16_t initialVal;                /*!< Initial counter value*/
    uint16_t maxVal;                    /*!< Maximum counter value*/
    ftm_phase_params_t phaseAConfig;    /*!< Configuration for the input phase a*/
    ftm_phase_params_t phaseBConfig;    /*!< Configuration for the input phase b*/
} ftm_quad_decode_config_t;

/*!
 * @brief FTM quadrature state(counter value and flags)
 *
 * Implements: ftm_quad_decoder_state_t_Class
 */
typedef struct
{
    uint16_t counter;           /*!< Counter value */
    bool overflowFlag;          /*!< True if overflow occurred,
                                 *   False if overflow doesn't occurred */
    bool overflowDirection;     /*!< False if overflow occurred at minimum value,
                                 *   True if overflow occurred at maximum value */
    bool counterDirection;      /*!< False FTM counter is decreasing,
                                 *   True FTM counter is increasing */
} ftm_quad_decoder_state_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the FTM driver.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @param[in] info The FTM user configuration structure, see #ftm_user_config_t.
 * @param[out] state The FTM state structure of the driver.
 * @return operation status
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_Init(uint32_t instance,
                          const ftm_user_config_t * info,
                          ftm_state_t * state);

/*!
 * @brief Shuts down the FTM driver.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @return operation status
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_Deinit(uint32_t instance);

/*!
 * @brief Initialize the FTM counter.
 *
 * Starts the FTM counter. This function provides access to the
 * FTM counter settings. The counter can be run in Up counting and Up-down counting modes.
 * To run the counter in Free running mode, choose Up counting option and provide
 * 0x0 for the countStartVal and 0xFFFF for countFinalVal. Please call this
 * function only when FTM is used as timer/counter.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @param[in] timer Timer configuration structure.
 * @return operation status
 *        - FTM_STATUS_SUCCESS : Initialized successfully.
 */
ftm_status_t FTM_DRV_InitCounter(uint32_t instance,
                                 const ftm_timer_param_t * timer);

/*!
 * @brief Starts the FTM counter.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @return operation status
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_CounterStart(uint32_t instance);

/*!
 * @brief Stops the FTM counter.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @return operation status
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 */
ftm_status_t FTM_DRV_CounterStop(uint32_t instance);

/*!
 * @brief Reads back the current value of the FTM counter.
 *
 * @param[in] instance The FTM peripheral instance number.
 * @return The current counter value
 */
uint32_t FTM_DRV_CounterRead(uint32_t instance);

/*!
 * @brief Stops all PWM channels .
 *
 * @param [in] instance The FTM peripheral instance number.
 * @return counter the current counter value
 */
ftm_status_t FTM_DRV_DeinitPwm(uint32_t instance);

/*!
 * @brief Configures the duty cycle and frequency and starts outputting the PWM on
 * all channels configured in param.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] param FTM driver PWM parameter to configure PWM options
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_InitPwm(uint32_t instance,
                             const ftm_pwm_param_t * param);

/*!
 * @brief This function updates the waveform output in PWM mode (duty cycle and phase).
 *
 * @param [in]instance The FTM peripheral instance number.
 * @param [in]channel The channel number. In combined mode, the code  finds the channel.
 * @param [in]typeOfUpdate The type of PWM update in the frequency or in ticks.
 * @param [in]firstEdge  Duty cycle or first edge time for PWM mode. Can take value between
 *                       0 - FTM_MAX_DUTY_CYCLE(0 = 0% from period  and FTM_MAX_DUTY_CYCLE = 100% from period)
 *                       Or value in ticks for the fisrt of the PWM mode.
 * @param [in]secondEdge Second edge time - only for combined mode. Can take value
 *                       between 0 - FTM_MAX_DUTY_CYCLE(0 = 0% from period  and FTM_MAX_DUTY_CYCLE = 100% from period).
 *                       Or value in ticks for the second of the PWM mode.
 * @param [in]softwareTrigger - if true a software trigger is generate to update PWM parameters
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_UpdatePwmChannel(uint32_t instance,
                                      uint8_t channel,
                                      ftm_pwm_update_option_t typeOfUpdate,
                                      uint16_t firstEdge,
                                      uint16_t secondEdge,
                                      bool softwareTrigger);

/*!
 * @brief This function will mask the output of the channels and at match events will be ignored
 * by the masked channels.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channelsMask - the mask which will select which channels will ignore match events.
 * @param [in] softwareTrigger - if true a software trigger is generate to update PWM parameters
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_MaskOutputChannels(uint32_t instance,
                                        uint32_t channelsMask,
                                        bool softwareTrigger);

/*!
 * @brief This function configure the initial counter value. The counter will get this
 * value after an overflow event.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] counterValue - initial counter value
 * @param [in] softwareTrigger - if true a software trigger is generate to update parameters
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */

ftm_status_t FTM_DRV_SetInitialCounterValue(uint32_t instance,
                                            uint16_t counterValue,
                                            bool softwareTrigger);

/*!
 * @brief This function configure the value of the counter which will generates an reload point.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] reloadPoint - counter value which generates the reload point
 * @param [in] softwareTrigger - if true a software trigger is generate to update parameters
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_SetHalfCycleReloadPoint(uint32_t instance,
                                             uint16_t reloadPoint,
                                             bool softwareTrigger);

/*!
 * @brief This function will force the output value of a channel to a specific value.
 * Before using this function it's mandatory to mask the match events using
 * FTM_DRV_MaskOutputChannels and to enable software output control using
 * FTM_DRV_SetSoftwareOutputChannelControl.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channelsValues - the values which will be software configured for channels.
 * @param [in] softwareTrigger - if true a software trigger is generate to update registers
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_SetSoftOutChnValue(uint32_t instance,
                                        uint8_t channelsValues,
                                        bool softwareTrigger);

/*!
 * @brief This function will configure which output channel can be software controlled.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channelsMask - the mask which will configure the channels which can be software controlled
 * @param [in] softwareTrigger - if true a software trigger is generate to update registers
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_SetSoftwareOutputChannelControl(uint32_t instance,
                                                     uint8_t channelsMask,
                                                     bool softwareTrigger);

/*!
 * @brief This function will configure if the second channel of a pair will be inverted or not.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] pairOfChannelsMask - the mask which will configure which channel pair will invert the second channel
 * @param [in] softwareTrigger - if true a software trigger is generate to update registers
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_SetInvertingControl(uint32_t instance,
                                         uint8_t channelsPairMask,
                                         bool softwareTrigger);

/*!
 * @brief This function configure the maximum counter value.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] counterValue - maximum counter value
 * @param [in] softwareTrigger - if true a software trigger is generate to update parameters
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_SetModuloCounterValue(uint32_t instance,
                                           uint16_t counterValue,
                                           bool softwareTrigger);

/*!
 * @brief This function configures sync mechanism for some FTM registers (MOD, CNINT, HCR,
 *          CnV, OUTMASK, INVCTRL, SWOCTRL).
 *
 * @param[in] instance The FTM peripheral instance number.
 * @param[in] param The sync configuration structure.
 * @return operation status
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_SetSync(uint32_t instance,
                             const ftm_pwm_sync_t * param);

/*!
 * @brief Configures the FTM to generate timed pulses(Output compare mode).
 *
 * When the FTM counter matches the value of CnV, the channel output is changed based on what is
 * specified in the compareMode argument. The signal period can be modified using
 * param->MaxCountValue. After this function max counter value and CnV are equal.
 * FTM_DRV_SetNextComparematchValue function ca be used to change CnV value.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] param configuration of the output compare channels
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_InitOutputCompare(uint32_t instance,
                                       const ftm_output_cmp_param_t * param);

/*!
 * @brief  Disables compare match output control and clears FTM timer configuration
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] param Configuration of the output compare channel
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_DeinitOutputCompare(uint32_t instance,
                                         const ftm_output_cmp_param_t * param);

/*!
 * @brief Sets the next compare match value based on the current counter value
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channel Configuration of the output compare channel
 * @param [in] nextComparematchValue Timer value in ticks until the next compare match event should appear
 * @param [in] update
 *        - FTM_RELATIVE_VALUE : nextComparemantchValue will be added to current counter value
 *        - FTM_ABSOLUTE_VALUE : nextComparemantchValue will be written in counter register as it is
 * @param [in] softwareTrigger This parameter will be true if software trigger sync is enabled and
 * the user want to generate a software trigger (the value from buffer will be moved to register immediate or
 * at next loading point depending on the sync configuration). Otherwise this parameter must be false
 * and the next compared value will be stored in buffer until a trigger signal will be received.
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_UpdateOutputCompareChannel(uint32_t instance,
                                                uint8_t channel,
                                                uint16_t nextComparematchValue,
                                                ftm_output_compare_update_t update,
                                                bool softwareTrigger);

/*!
 * @brief   Configures Channel Input Capture for either getting time-stamps on edge detection
 * or on signal measurement . When the edge specified in the captureMode
 * argument occurs on the channel the FTM counter is captured into the CnV register.
 * The user will have to read the CnV register separately to get this value. The filter
 * function is disabled if the filterVal argument passed in is 0. The filter function
 * is available only on channels 0,1,2,3.
 *
 * @param [in] instance The FTM peripheral instance number
 * @param [in] param Configuration of the input capture channel
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_InitInputCapture(uint32_t instance,
                                      const ftm_input_param_t * param);

/*!
 * @brief  Disables input capture mode and clears FTM timer configuration
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] param Configuration of the output compare channel
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_DeinitInputCapture(uint32_t instance,
                                        const ftm_input_param_t * param);

/*!
 * @brief  This function is used to calculate the measurement and/or time stamps values
 * which are read from the C(n, n+1)V registers and stored to the static buffers.
 *
 * @param [in] instance The FTM peripheral instance number
 * @param [in] channel  For getting the time stamp of the last edge (in normal input capture) this
 *                    parameter represents the channel number.
 *                    For getting the last measured value (in dual edge input capture) this parameter
 *                    is the lowest channel number of the pair (EX: 0, 2, 4, 6).
 * @return value   The measured value
 */
uint16_t FTM_DRV_GetInputCaptureMeasurement(uint32_t instance,
                                            uint8_t channel);

/*!
 * @brief  Starts new single-shot signal measurement of the given channel.
 *
 * @param [in] instance The FTM peripheral instance number.
 * @param [in] channel Configuration of the output compare channel
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_StartNewSignalMeasurement(uint32_t instance,
                                               uint8_t channel);

/*!
 * @brief Configures the quadrature mode and starts measurement
 *
 * @param [in] instance Instance number of the FTM module.
 * @param [in] config   Configuration structure(quadrature decode mode, polarity for both phases,
 *                      initial and maximum value for the counter, filter configuration)
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_QuadDecodeStart(uint32_t instance,
                                     const ftm_quad_decode_config_t * config);

/*!
 * @brief De-activates the quadrature decode mode.
 *
 * @param [in] instance Instance number of the FTM module.
 * @return success
 *        - FTM_STATUS_SUCCESS : Completed successfully.
 *        - FTM_STATUS_ERROR : Error occurred.
 */
ftm_status_t FTM_DRV_QuadDecodeStop(uint32_t instance);

/*!
 * @brief Return the current quadrature decoder state (counter value, overflow flag and
 * overflow direction)
 *
 * @param [in] instance Instance number of the FTM module.
 * @return The current state of quadrature decoder
 */
ftm_quad_decoder_state_t FTM_DRV_QuadGetState(uint32_t instance);

/*!
 * @brief Retrieves the frequency of the clock source feeding the FTM counter.
 *
 * Function will return a 0 if no clock source is selected and the FTM counter is disabled
 *
 * @param [in] instance The FTM peripheral instance number.
 * @return The frequency of the clock source running the FTM counter (0 if counter is disabled)
 */
uint32_t FTM_DRV_GetFrequency(uint32_t instance);


/*!
 * @brief This function is used to covert the given frequency to period in ticks
 *
 * @param [in] instance The FTM peripheral instance number
 * @param [in] freqencyHz Frequency value in Hz
 *
 * @return uint16_t
 */
uint16_t FTM_DRV_ConvertFreqToPeriodTicks(uint32_t instance,
                                          uint32_t freqencyHz);

#if defined(__cplusplus)
}
#endif

/*! @}*/

/*! @}*/ /* End of addtogroup ftm_driver */

#endif /* FSL_FTM_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
