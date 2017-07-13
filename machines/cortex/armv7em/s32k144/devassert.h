/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#ifndef FSL_DEVASSERT_H
#define FSL_DEVASSERT_H

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, global macro not referenced.
 * The macro is defined to be used by drivers to validate input parameters and can be disabled.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Directive 4.9, Function-like macro defined.
 * The 'DEV_ASSERT' macro is used to validate input parameters to driver functions.
 *
 */

/**
\page Error_detection_and_reporting Error detection and reporting

Application code can disable or enable development-time error checking. These errors, which are related to validation of input parameters, are 
usually resolved at development if the parameters are static (i.e. known at compile time) and the overhead of these checks can be disabled in 
production.

S32 SDK drivers use an assert macro, DEV_ASSERT, to check the validity of input parameters. To enable the macro, one or two steps are necessary:
    - implementing a custom DEV_ASSERT macro in a custom file. This requires to define the CUSTOM_DEVASSERT symbol with the file name in the 
project configuration (for example: -DCUSTOM_DEVASSERT="custom_devassert.h")
    - define the DEV_ERROR_DETECT symbol in the project configuration to enable the assertion checks (for example: -DDEV_ERROR_DETECT).
    
If CUSTOM_DEVASSERT is not defined but DEV_ERROR_DETECT is defined, the DEV_ASSERT macro will use a default implementation.
*/

#if defined (CUSTOM_DEVASSERT)
    /* If the CUSTOM_DEVASSERT symbol is defined, then add the custom implementation */
    /* Note: Application still needs to define DEV_ERROR_DETECT for assertions to be activated */
    #include CUSTOM_DEVASSERT
#elif defined (DEV_ERROR_DETECT)
    /* Implement default assert macro */
    #define DEV_ASSERT(x) if(x) { } else {  __asm("BKPT #0\n\t"); for(;;){} }
#else
    /* Assert macro does nothing */
    #define DEV_ASSERT(x) ((void)0)
#endif

#endif /* FSL_DEVASSERT_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
