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

#ifndef FSL_RTC_DRIVER_H
#define FSL_RTC_DRIVER_H

#include <stdbool.h>
#include <stddef.h>

#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_rtc_hal.h"

/*! @file */

/*!
 * @addtogroup rtc_driver Real Time Clock Driver
 * @ingroup rtc
 * @brief Real Time Clock Driver Peripheral Driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SECONDS_IN_A_DAY     (86400UL)
#define SECONDS_IN_A_HOUR    (3600U)
#define SECONDS_IN_A_MIN     (60U)
#define MINS_IN_A_HOUR       (60U)
#define HOURS_IN_A_DAY       (24U)
#define DAYS_IN_A_YEAR       (365U)
#define DAYS_IN_A_LEAP_YEAR  (366U)
#define YEAR_RANGE_START     (1970U)
#define YEAR_RANGE_END       (2099U)

/* Table of month length (in days) for the Un-leap-year*/
static const uint8_t ULY[] = {0U, 31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

/* Table of month length (in days) for the Leap-year*/
static const uint8_t LY[] = {0U, 31U, 29U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

/* Number of days from begin of the non Leap-year*/
static const uint16_t MONTH_DAYS[] = {0U, 0U, 31U, 59U, 90U, 120U, 151U, 181U, 212U, 243U, 273U, 304U, 334U};

/*!
 * @brief RTC Time Date structure
 */
typedef struct
{
    uint16_t year;      /*!< Year       */
    uint16_t month;     /*!< Month      */
    uint16_t day;       /*!< Day        */
    uint16_t hour;      /*!< Hour       */
    uint16_t minutes;   /*!< Minutes    */
    uint8_t seconds;    /*!< Seconds    */
} rtc_timedate_t;

/*!
 * @brief RTC Initialization structure
 */
typedef struct
{
    uint8_t                 compensationInterval;       /*!< Compensation Interval  */
    int8_t                  compensation;               /*!< Compensation Value     */
    rtc_clk_select_t        clockSelect;                /*!< RTC Clock Select       */
    rtc_clk_out_config_t    clockOutConfig;             /*!< RTC Clock Out Source   */
    bool                    updateEnable;               /*!< Enable changing the Time
                                                         * Counter Enable bit even if
                                                         * the Status register is
                                                         * locked                   */
    bool                    nonSupervisorAccessEnable;  /*!< Enable writes to the
                                                         * registers in non Supervisor
                                                         * Mode
                                                         *                          */
} rtc_init_config_t;

/*!
 * @brief RTC alarm configuration
 */
typedef struct
{
    rtc_timedate_t  alarmTime;                  /*!< Alarm time                   */
    uint32_t        repetitionInterval;         /*!< Interval of repetition in sec*/
    uint32_t        numberOfRepeats;            /*!< Number of alarm repeats      */
    bool            repeatForever;              /*!< Repeat forever if set, discard
                                                 * number of repeats              */
    bool            alarmIntEnable;             /*!< Enable alarm interrupt       */
    void            (* alarmCallback)(void *);  /*!< Pointer to the user callback
                                                 * method.
                                                 */
    void            * callbackParams;           /*!< Pointer to the callback
                                                 * parameters.
                                                 */
} rtc_alarm_config_t;

/*!
 * @brief RTC interrupt configuration. It is used to configure interrupt other
 *        than Time Alarm and Time Seconds interrupt
 */
typedef struct
{
    bool                 overflowIntEnable;         /*!< Enable Time Overflow Interrupt*/
    bool                 timeInvalidIntEnable;      /*!< Enable Time Invalid Interrupt */
    void                (* rtcCallback)(void *);    /*!< Pointer to the user callback
                                                     * method.
                                                     */
    void                * callbackParams;           /*!< Pointer to the callback
                                                     * parameters.
                                                     */
} rtc_interrupt_config_t;

/*!
 * @brief RTC Seconds Interrupt Configuration
 */
typedef struct
{
    rtc_second_int_cfg_t secondIntConfig;               /*!< Seconds Interrupt
                                                         *  frequency
                                                         */
    bool                 secondIntEnable;               /*!< Seconds Interrupt
                                                         *  enable
                                                         */
    void                (* rtcSecondsCallback)(void *); /*!< Pointer to the user
                                                         * callback method.
                                                         */
    void                * secondsCallbackParams;        /*!< Pointer to the
                                                         * callback parameters.
                                                         */
} rtc_seconds_int_config_t;

/*!
 * @brief RTC Register Lock Configuration
 */
typedef struct
{
    bool lockRegisterLock;              /*!< Lock state of the Lock Register   */
    bool statusRegisterLock;            /*!< Lock state of the Status Register */
    bool controlRegisterLock;           /*!< Lock state of the Control Register */
    bool timeCompensationRegisterLock;  /*!< Lock state of the Time Compensation
                                         * Register
                                         */
} rtc_register_lock_config_t;

/*******************************************************************************
 * Code
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize RTC instance
 * @param[in] instance The number of the RTC instance used
 * @param[in] rtcUserCfg Pointer to the user's configuration structure
 * @return    RTC_STATUS_SUCCESS if the operation was successful, RTC_STATUS_ERROR
 *            if there was a problem or RTC_STATUS_LOCKED if the control register
 *            is locked
 */
rtc_status_t RTC_DRV_Init(uint32_t instance, const rtc_init_config_t * const rtcUserCfg);

/*!
 * @brief Deinitialize RTC instance
 * @param[in] instance The number of the RTC instance used
 * @return    RTC_STATUS_SUCCESS if the operation was successful, or RTC_STATUS_LOCKED
 *            if the control register is locked
 */
rtc_status_t RTC_DRV_Deinit(uint32_t instance);

/*!
 * @brief Start RTC instance counter. Before calling this function the user
 * should use RTC_DRV_SetTimeDate to configure the start time
 * @param[in] instance The number of the RTC instance used
 * @return    RTC_STATUS_SUCCESS if the operation was successful, RTC_STATUS_ERROR
 *            if there was a problem.
 */
rtc_status_t RTC_DRV_StartCounter(uint32_t instance);

/*!
 * @brief Disable RTC instance
 * @param[in] instance The number of the RTC instance used
 * @return    RTC_STATUS_SUCCESS if the operation was successful, RTC_STATUS_LOCKED
 *            if the control register is locked
 */
rtc_status_t RTC_DRV_StopCounter(uint32_t instance);

/*!
 * @brief Get current time and date from RTC instance
 * @param[in]  instance The number of the RTC instance used
 * @param[out] currentTime Pointer to the variable in which to store the result
 * @return     RTC_STATUS_SUCCESS if the operation was successful, RTC_STATUS_ERROR
 *             if there was a problem
 */
rtc_status_t RTC_DRV_GetCurrentTimeDate(uint32_t instance, rtc_timedate_t * const currentTime);

/*!
 * @brief Set time and date for RTC instance. The user must stop the counter
 * before using this function. Otherwise it will return an error
 * @param[in] instance The number of the RTC instance used
 * @param[in] time Pointer to the variable in which the time is stored
 * @return    RTC_STATUS_SUCCESS if the operation was successful, RTC_STATUS_ERROR
 *            if there was a problem, RTC_STATUS_LOCKED if the status register
 *            is locked
 */
rtc_status_t RTC_DRV_SetTimeDate(uint32_t instance, const rtc_timedate_t * const time);

/*!
 * @brief Configure which registers to lock for RTC instance
 * @param[in] instance The number of the RTC instance used
 * @param[in] lockConfig Pointer to the lock configuration structure
 * @return    RTC_STATUS_SUCCESS if the operation was successful, RTC_STATUS_ERROR
 *            if there was a problem, RTC_STATUS_LOCKED if the lock register
 *            is locked
 */
rtc_status_t RTC_DRV_ConfigureRegisterLock(uint32_t instance, const rtc_register_lock_config_t * const lockConfig);

/*!
 * @brief Get which registers are locked for RTC instance
 * @param[in]  instance The number of the RTC instance used
 * @param[out] lockConfig Pointer to the lock configuration structure in which to
 *             save the data
 * @return None
 */
void RTC_DRV_GetRegisterLock(uint32_t instance, rtc_register_lock_config_t * const lockConfig);

/*!
 * @brief Configure compensation for RTC instance
 * @param[in] instance The number of the RTC instance used
 * @param[in] compInterval Compensation interval
 * @param[in] compensation Compensation value
 * @return    RTC_STATUS_SUCCESS if the operation was successful, RTC_STATUS_ERROR
 *            if there was a problem, RTC_STATUS_LOCKED if the lock register
 *            is locked
 */
rtc_status_t RTC_DRV_ConfigureTimeCompensation(uint32_t instance, uint8_t  compInterval, int8_t   compensation);

/*!
 * @brief Get time compensation for RTC instance
 * @param[in]  instance The number of the RTC instance used
 * @param[out] compInterval Pointer to the variable in which to save the compensation
 *             interval
 * @param[out] compensation Pointer to the variable in which to save the compensation
 *             value
 * @return None
 */
void RTC_DRV_GetTimeCompensation(uint32_t instance, uint8_t * compInterval, int8_t * compensation);

/*!
 * @brief Configure fault interrupts for RTC instance
 * @param[in] instance The number of the RTC instance used
 * @param[in] intConfig Pointer to the structure which holds the configuration
 * @return    None
 */
void RTC_DRV_ConfigureFaultInt(uint32_t instance, rtc_interrupt_config_t * const intConfig);

/*!
 * @brief Configure seconds interrupts for RTC instance
 * @param[in] instance The number of the RTC instance used
 * @param[in] intConfig Pointer to the structure which holds the configuration
 * @return    None
 */
void RTC_DRV_ConfigureSecondsInt(uint32_t instance, rtc_seconds_int_config_t * const intConfig);

/*!
 * @brief Configure alarm for RTC instance
 * @param[in] instance The number of the RTC instance used
 * @param[in] alarmConfig Pointer to the structure which holds the alarm configuration
 * @return    RTC_STATUS_SUCCESS if the operation was successful, RTC_STATUS_ERROR
 *            if there was a problem
 */
rtc_status_t RTC_DRV_ConfigureAlarm(uint32_t instance, rtc_alarm_config_t * const alarmConfig);

/*!
 * @brief Get alarm configuration for RTC instance
 * @param[in]  instance The number of the RTC instance used
 * @param[out] alarmConfig Pointer to the structure in which to store the alarm
 *             configuration
 * @return     None
 */
void RTC_DRV_GetAlarmConfig(uint32_t instance, rtc_alarm_config_t * alarmConfig);

/*!
 * @brief Check if alarm is pending
 * @param[in] instance The number of the RTC instance used
 * @return    True if the alarm has occurred, false if not
 */
bool RTC_DRV_IsAlarmPending(uint32_t instance);

/*!
 * @brief Convert seconds to rtc_timedate_t structure
 * @param[in]  seconds Pointer to the seconds
 * @param[out] timeDate Pointer to the structure in which to store the result
 * @return     None
 */
void RTC_DRV_ConvertSecondsToTimeDate(const uint32_t * seconds, rtc_timedate_t * const timeDate);

/*!
 * @brief Convert seconds to rtc_timedate_t structure
 * @param[in]  timeDate Pointer to the source struct
 * @param[out] seconds Pointer to the variable in which to store the result
 * @return     None
 */
void RTC_DRV_ConvertTimeDateToSeconds(const rtc_timedate_t * const timeDate, uint32_t * const seconds);

/*!
 * @brief Check if the current year is leap
 * @param[in] year Year to check
 * @return    True if the year is leap, false if not
 */
bool RTC_DRV_IsYearLeap(uint16_t year);

/*!
 * @brief Check if the date time struct is configured properly
 * @param[in] timeDate Structure to check to check
 * @return    True if the time date is in the correct format, false if not
 */
bool RTC_DRV_IsTimeDateCorrectFormat(const rtc_timedate_t * const timeDate);

/*!
 * @brief Gets the next alarm time
 * @param[in]  instance The number of the RTC instance used
 * @param[out] alarmTime Pointer to the variable in which to store the data
 * @return     RTC_STATUS_SUCCESS if the next alarm time is valid,
 *             RTC_STATUS_ERROR if there is no new alarm or alarm configuration
 *             specified.
 */

rtc_status_t RTC_DRV_GetNextAlarmTime(uint32_t instance, rtc_timedate_t * const alarmTime);

/*!
 * @brief This method is the API's Interrupt handler for generic and alarm IRQ.
 * It will handle the alarm repetition and calls the user callbacks if they
 * are not NULL.
 * @param[in] instance RTC instance used
 * @return    None
 */
void RTC_DRV_IRQHandler(uint32_t instance);

/*!
 * @brief This method is the API's Interrupt handler for RTC Second
 * interrupt. This ISR will call the user callback if defined.
 * @param[in] instance RTC instance used
 * @return    None
 */
void RTC_DRV_SecondsIRQHandler(uint32_t instance);

#if defined(__cplusplus)
}
#endif


/*! @}*/

#endif /* FSL_RTC_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
