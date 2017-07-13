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


/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, conversion between a pointer and integer type
 * The cast from unsigned integer to (TRGMUX_Type *) is required in order to initialize
 * the table of base addresses for the TRGMUX instances.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, cast from unsigned int to pointer
 * The cast from unsigned integer to (TRGMUX_Type *) is required in order to initialize
 * the table of base addresses for the TRGMUX instances.
 */


#include "fsl_trgmux_driver.h"
#include <stddef.h>

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of base addresses for TRGMUX instances. */
static TRGMUX_Type * const s_trgmuxBase[TRGMUX_INSTANCE_COUNT] = TRGMUX_BASE_PTRS;


/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name :     TRGMUX_DRV_Init
 * Description   :     This function sets first the source trigger of all TRGMUX target modules to their
 * default values, then loops through all in-out mappings defined in the user configuration structure
 * and configures the user defined target modules with the corresponding source triggers.
 * This example shows how to set up the trgmux_user_config_t parameters and how to call the
 * TRGMUX_DRV_Init function by passing in the needed parameters:
 *   trgmux_user_config_t             trgmuxConfig;
 *   trgmux_inout_mapping_config_t    trgmuxInoutMappingConfig[] =
 *   {
 *      {TRGMUX_TRIG_SOURCE_TRGMUX_IN9,     TRGMUX_TARGET_MODULE_DMA_CH0,     false},
 *      {TRGMUX_TRIG_SOURCE_FTM1_EXT_TRIG,  TRGMUX_TARGET_MODULE_TRGMUX_OUT4, true}
 *   };
 *   trgmuxConfig.numInOutMappingConfigs = 2;
 *   trgmuxConfig.inOutMappingConfig     = trgmuxInoutMappingConfig;
 *   TRGMUX_DRV_Init(instance, &trgmuxConfig);
 *
 * Implements    :     TRGMUX_DRV_Init_Activity
 *END**************************************************************************/
trgmux_status_t TRGMUX_DRV_Init(
                                const uint32_t                        instance,
                                const trgmux_user_config_t * const    trgmuxUserConfig
                               )
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < TRGMUX_INSTANCE_COUNT);
    DEV_ASSERT(s_trgmuxBase[instance]);
    DEV_ASSERT(trgmuxUserConfig != NULL);
#endif

    trgmux_status_t status;
    TRGMUX_Type *   base = s_trgmuxBase[instance];
    uint8_t         count;

    /* Reset source triggers of all TRGMUX target modules to default. */
    status = TRGMUX_HAL_Init(base);

    if(status == TRGMUX_STATUS_SUCCESS)
    {
        /* Loop through all in-out mappings in the configuration and apply them in TRGMUX */
        for(count = 0U; count < trgmuxUserConfig->numInOutMappingConfigs; count++)
        {
            TRGMUX_HAL_SetTrigSourceForTargetModule(base,                                                      \
                                                    trgmuxUserConfig->inOutMappingConfig[count].triggerSource, \
                                                    trgmuxUserConfig->inOutMappingConfig[count].targetModule);
        }

        /* Loop through all in-out mappings in the configuration and lock them if required */
        for(count = 0U; count < trgmuxUserConfig->numInOutMappingConfigs; count++)
        {
            if(trgmuxUserConfig->inOutMappingConfig[count].lockTargetModuleReg)
            {
                TRGMUX_HAL_SetLockForTargetModule(  base,                                                       \
                                                    trgmuxUserConfig->inOutMappingConfig[count].targetModule);
            }
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_DRV_Deinit
 * Description   : This function sets the source trigger of all TRGMUX target modules
 * to their default values
 *
 * Implements    : TRGMUX_DRV_Deinit_Activity
 *END**************************************************************************/
trgmux_status_t TRGMUX_DRV_Deinit(
                                     const uint32_t  instance
                                 )
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < TRGMUX_INSTANCE_COUNT);
    DEV_ASSERT(s_trgmuxBase[instance]);
#endif

    TRGMUX_Type *    base = s_trgmuxBase[instance];
    trgmux_status_t  status;

    /* Reset source triggers of all TRGMUX target modules to default. */
    status = TRGMUX_HAL_Init(base);

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_DRV_SetTrigSourceForTargetModule
 * Description   : This function configures in TRGMUX IP the link between an input (source trigger) and
 * and output (target module).
 *
 * Implements    : TRGMUX_DRV_SetTrigSourceForTargetModule_Activity
 *END**************************************************************************/
trgmux_status_t TRGMUX_DRV_SetTrigSourceForTargetModule(
                                                        const uint32_t                instance,
                                                        const trgmux_trigger_source_t triggerSource,
                                                        const trgmux_target_module_t  targetModule
                                                       )
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < TRGMUX_INSTANCE_COUNT);
    DEV_ASSERT(s_trgmuxBase[instance]);
#endif

    TRGMUX_Type *    base = s_trgmuxBase[instance];
    trgmux_status_t  status;
    bool             lock;

    lock = TRGMUX_HAL_GetLockForTargetModule(base, targetModule);

    if(lock == true)
    {
        status = TRGMUX_STATUS_ERROR_TARGET_LOCKED;
    }
    else
    {
        /* Configure link between trigger source and target module. */
        TRGMUX_HAL_SetTrigSourceForTargetModule(base, triggerSource, targetModule);
        status = TRGMUX_STATUS_SUCCESS;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_DRV_GetTrigSourceForTargetModule
 * Description   : This function reads from the TRGMUX IP the input (source trigger) that
 * is linked to a given output (target module).
 *
 * Implements    : TRGMUX_DRV_GetTrigSourceForTargetModule_Activity
 *END**************************************************************************/
trgmux_trigger_source_t TRGMUX_DRV_GetTrigSourceForTargetModule(
                                                        const uint32_t                instance,
                                                        const trgmux_target_module_t  targetModule
                                                               )
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < TRGMUX_INSTANCE_COUNT);
    DEV_ASSERT(s_trgmuxBase[instance]);
#endif

    const TRGMUX_Type * base = s_trgmuxBase[instance];

    return TRGMUX_HAL_GetTrigSourceForTargetModule(base, targetModule);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_DRV_SetLockForTargetModule
 * Description   : This function locks the TRGMUX register that corresponds to the
 * given target module.
 *
 * Implements    : TRGMUX_DRV_SetLockForTargetModule_Activity
 *END**************************************************************************/
trgmux_status_t TRGMUX_DRV_SetLockForTargetModule(
                                                    const uint32_t                instance,
                                                    const trgmux_target_module_t  targetModule
                                                 )
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < TRGMUX_INSTANCE_COUNT);
    DEV_ASSERT(s_trgmuxBase[instance]);
#endif

    TRGMUX_Type * base = s_trgmuxBase[instance];

    TRGMUX_HAL_SetLockForTargetModule(base, targetModule);

    return TRGMUX_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TRGMUX_DRV_GetLockForTargetModule
 * Description   : This function reads the lock status of the TRGMUX register
 * coresponding to the given target module.
 *
 * Implements    : TRGMUX_DRV_GetLockForTargetModule_Activity
 *END**************************************************************************/
bool  TRGMUX_DRV_GetLockForTargetModule(
                                            const uint32_t                instance,
                                            const trgmux_target_module_t  targetModule
                                          )
{
#ifdef DEV_ERROR_DETECT
    DEV_ASSERT(instance < TRGMUX_INSTANCE_COUNT);
    DEV_ASSERT(s_trgmuxBase[instance]);
#endif

    const TRGMUX_Type * base = s_trgmuxBase[instance];

    return TRGMUX_HAL_GetLockForTargetModule(base, targetModule);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
