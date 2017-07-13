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

#include "fsl_csec_driver.h"

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 1.3, Taking address of near auto variable
 * The code is not dynamically linked. An absolute stack address is obtained when
 * taking the address of the near auto variable. A source of error in writing
 * dynamic code is that the stack segment may be different from the data segment.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and
 * integer type.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define,
 * representing an address.
 */


/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define CSEC_PAGE_SIZE_IN_BYTES    (16U)

#define CSEC_DATA_PAGES_AVAILABLE   (7U)
#define CSEC_DATA_BYTES_AVAILABLE   (112U)

#define CSEC_BYTES_TO_FROM_PAGES_SHIFT   (4U)
#define CSEC_BYTES_TO_FROM_BITS_SHIFT    (3U)

#define CSEC_M1_SIZE_IN_BYTES   (16U)
#define CSEC_M2_SIZE_IN_BYTES   (32U)
#define CSEC_M3_SIZE_IN_BYTES   (16U)
#define CSEC_M4_SIZE_IN_BYTES   (32U)
#define CSEC_M5_SIZE_IN_BYTES   (16U)

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

static inline uint32_t CSEC_DRV_RoundTo(uint32_t value, uint32_t roundTo)
{
    return (value + (roundTo - 1U)) & ~(roundTo - 1U);
}

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_EncryptECB
 * Description   : This function performs the AES-128 encryption in ECB mode of
 * the input plain text buffer.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_EncryptECB(csec_key_id_t keyId,
                                      const uint8_t *plainText,
                                      uint32_t length,
                                      uint8_t *cipherText)
{
    uint32_t numPagesLeft = length >> CSEC_BYTES_TO_FROM_PAGES_SHIFT;
    csec_error_code_t stat = CSEC_NO_ERROR;
    uint8_t index = 0;

    /* Loop and launch commands until the end of the plain text */
    while (numPagesLeft > 0U)
    {
        uint16_t numPages = (uint16_t)((numPagesLeft > CSEC_DATA_PAGES_AVAILABLE) ?
                                        CSEC_DATA_PAGES_AVAILABLE : numPagesLeft);
        uint8_t numBytes = (uint8_t)(numPages << CSEC_BYTES_TO_FROM_PAGES_SHIFT);

        /* Write the plain text */
        CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, &plainText[index], numBytes);
        /* Write the size of the plain text (in pages) */
        CSEC_HAL_WriteCommandHalfWord(FSL_FEATURE_CSEC_PAGE_LENGTH_OFFSET, numPages);
        /* Write the command header. This will trigger the command execution. */
        CSEC_HAL_WriteCommandHeader(CSEC_CMD_ENC_ECB, CSEC_FUNC_FORMAT_COPY, CSEC_CALL_SEQ_FIRST, keyId);

        /* Wait until the execution of the command is complete */
        CSEC_HAL_WaitCommandCompletion();

        /* Read the status of the execution */
        stat = CSEC_HAL_ReadErrorBits();
        if (stat != CSEC_NO_ERROR)
        {
            break;
        }

        /* Read the resulted cipher text */
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, &cipherText[index], numBytes);

        numPagesLeft = numPagesLeft - numPages;
        index = index + numBytes;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_DecryptECB
 * Description   : This function performs the AES-128 decryption in ECB mode of
 * the input cipher text buffer.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_DecryptECB(csec_key_id_t keyId,
                                      const uint8_t *cipherText,
                                      uint32_t length,
                                      uint8_t *plainText)
{
    uint32_t numPagesLeft = length >> CSEC_BYTES_TO_FROM_PAGES_SHIFT;
    csec_error_code_t stat = CSEC_NO_ERROR;
    uint8_t index = 0;

    /* Loop and launch commands until the end of the cipher text */
    while (numPagesLeft > 0U)
    {
        uint16_t numPages = (uint16_t)((numPagesLeft > CSEC_DATA_PAGES_AVAILABLE) ?
                                        CSEC_DATA_PAGES_AVAILABLE : numPagesLeft);
        uint8_t numBytes = (uint8_t)(numPages << CSEC_BYTES_TO_FROM_PAGES_SHIFT);

        /* Write the cipher text */
        CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, &cipherText[index], numBytes);
        /* Write the size of the cipher text (in pages) */
        CSEC_HAL_WriteCommandHalfWord(FSL_FEATURE_CSEC_PAGE_LENGTH_OFFSET, numPages);
        /* Write the command header. This will trigger the command execution. */
        CSEC_HAL_WriteCommandHeader(CSEC_CMD_DEC_ECB, CSEC_FUNC_FORMAT_COPY, CSEC_CALL_SEQ_FIRST, keyId);

        /* Wait until the execution of the command is complete */
        CSEC_HAL_WaitCommandCompletion();

        /* Read the status of the execution */
        stat = CSEC_HAL_ReadErrorBits();
        if (stat != CSEC_NO_ERROR)
        {
            break;
        }

        /* Read the resulted plain text */
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, &plainText[index], numBytes);

        numPagesLeft = numPagesLeft - numPages;
        index = index + numBytes;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_EncryptCBC
 * Description   : This function performs the AES-128 encryption in CBC mode of
 * the input cipher text buffer.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_EncryptCBC(csec_key_id_t keyId,
                                      const uint8_t *plainText,
                                      uint32_t length,
                                      const uint8_t *iv,
                                      uint8_t *cipherText)
{
    csec_call_sequence_t seq = CSEC_CALL_SEQ_FIRST;
    uint16_t numPagesTotal = (uint16_t)(length >> CSEC_BYTES_TO_FROM_PAGES_SHIFT);
    uint16_t numPagesLeft = numPagesTotal;
    uint8_t index = 0;
    csec_error_code_t stat = CSEC_NO_ERROR;

    /* Loop and launch commands until the end of the cipher text */
    while (numPagesLeft > 0U)
    {
        uint16_t numPages;
        uint8_t numBytes;

        /* Write the plain text. The first call of encryption shall also include
        the IV. Further calls only include the plain text */
        if (seq == CSEC_CALL_SEQ_FIRST)
        {
            numPages = (uint16_t)((numPagesLeft > (CSEC_DATA_PAGES_AVAILABLE - 1U)) ?
                                  (CSEC_DATA_PAGES_AVAILABLE - 1U) : numPagesLeft);
            numBytes = (uint8_t)(numPages << CSEC_BYTES_TO_FROM_PAGES_SHIFT);
            CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, iv, CSEC_PAGE_SIZE_IN_BYTES);
            CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_2_OFFSET, &plainText[index], numBytes);
        }
        else
        {
            numPages = (uint16_t)((numPagesLeft > CSEC_DATA_PAGES_AVAILABLE) ?
                                   CSEC_DATA_PAGES_AVAILABLE : numPagesLeft);
            numBytes = (uint8_t)(numPages << CSEC_BYTES_TO_FROM_PAGES_SHIFT);
            CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, &plainText[index], numBytes);
        }
        /* Write the size of the plain text (in pages) */
        CSEC_HAL_WriteCommandHalfWord(FSL_FEATURE_CSEC_PAGE_LENGTH_OFFSET, numPagesTotal);
        /* Write the command header. This will trigger the command execution. */
        CSEC_HAL_WriteCommandHeader(CSEC_CMD_ENC_CBC, CSEC_FUNC_FORMAT_COPY, seq, keyId);

        /* Wait until the execution of the command is complete */
        CSEC_HAL_WaitCommandCompletion();

        /* Read the status of the execution */
        stat = CSEC_HAL_ReadErrorBits();
        if (stat != CSEC_NO_ERROR)
        {
            break;
        }

        /* Read the resulted cipher text. For the first call, start from the
        second page, as the first page is occupied by the IV */
        if (seq == CSEC_CALL_SEQ_FIRST)
        {
            CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_2_OFFSET, &cipherText[index], numBytes);
            seq = CSEC_CALL_SEQ_SUBSEQUENT;
        }
        else
        {
            CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, &cipherText[index], numBytes);
        }

        numPagesLeft = numPagesLeft - numPages;
        index = index + numBytes;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_DecryptCBC
 * Description   : This function performs the AES-128 decryption in CBC mode of
 * the input cipher text buffer.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_DecryptCBC(csec_key_id_t keyId,
                                      const uint8_t *cipherText,
                                      uint16_t length,
                                      const uint8_t* iv,
                                      uint8_t *plainText)
{
    csec_call_sequence_t seq = CSEC_CALL_SEQ_FIRST;
    uint16_t numPagesTotal = length >> CSEC_BYTES_TO_FROM_PAGES_SHIFT;
    uint16_t numPagesLeft = numPagesTotal;
    uint8_t index = 0;
    csec_error_code_t stat = CSEC_NO_ERROR;

    /* Loop and launch commands until the end of the cipher text */
    while (numPagesLeft > 0U)
    {
        uint16_t numPages;
        uint8_t numBytes;

        /* Write the cipher text. The first call of encryption shall also include
        the IV. Further calls only include the plain text */
        if (seq == CSEC_CALL_SEQ_FIRST)
        {
            numPages = (uint16_t)((numPagesLeft > (CSEC_DATA_PAGES_AVAILABLE - 1U)) ?
                                  (CSEC_DATA_PAGES_AVAILABLE - 1U) : numPagesLeft);
            numBytes = (uint8_t)(numPages << CSEC_BYTES_TO_FROM_PAGES_SHIFT);
            CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, iv, CSEC_PAGE_SIZE_IN_BYTES);
            CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_2_OFFSET, &cipherText[index], numBytes);
        }
        else
        {
            numPages = (uint16_t)((numPagesLeft > CSEC_DATA_PAGES_AVAILABLE) ?
                                   CSEC_DATA_PAGES_AVAILABLE : numPagesLeft);
            numBytes = (uint8_t)(numPages << CSEC_BYTES_TO_FROM_PAGES_SHIFT);
            CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, &cipherText[index], numBytes);
        }
        /* Write the size of the cipher text (in pages) */
        CSEC_HAL_WriteCommandHalfWord(FSL_FEATURE_CSEC_PAGE_LENGTH_OFFSET, numPagesTotal);
        /* Write the command header. This will trigger the command execution. */
        CSEC_HAL_WriteCommandHeader(CSEC_CMD_DEC_CBC, CSEC_FUNC_FORMAT_COPY, seq, keyId);

        /* Wait until the execution of the command is complete */
        CSEC_HAL_WaitCommandCompletion();

        /* Read the status of the execution */
        stat = CSEC_HAL_ReadErrorBits();
        if (stat != CSEC_NO_ERROR)
        {
            break;
        }

        /* Read the resulted plain text. For the first call, start from the
        second page, as the first page is occupied by the IV */
        if (seq == CSEC_CALL_SEQ_FIRST)
        {
            CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_2_OFFSET, &plainText[index], numBytes);
            seq = CSEC_CALL_SEQ_SUBSEQUENT;
        }
        else
        {
            CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, &plainText[index], numBytes);
        }

        numPagesLeft = numPagesLeft - numPages;
        index = index + numBytes;
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_GenerateMAC
 * Description   : This function calculates the MAC of a given message using
 * CMAC with AES-128.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_GenerateMAC(csec_key_id_t keyId,
                                       const uint8_t *msg,
                                       uint32_t msgLen,
                                       uint8_t *cmac)
{
    csec_call_sequence_t seq = CSEC_CALL_SEQ_FIRST;
    /* Size of the message is given in bits, compute the number of bytes */
    uint32_t numBytesLeft = CSEC_DRV_RoundTo(msgLen, 0x8) >> CSEC_BYTES_TO_FROM_BITS_SHIFT;
    csec_error_code_t stat = CSEC_NO_ERROR;
    uint8_t index = 0;

    /* Loop and launch commands until the end of the message */
    while (numBytesLeft > 0U)
    {
        uint8_t numBytes = (uint8_t)((numBytesLeft > CSEC_DATA_BYTES_AVAILABLE) ?
                                      CSEC_DATA_BYTES_AVAILABLE : numBytesLeft);

        /* Write the message for which the MAC will be computed  */
        CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, &msg[index], numBytes);
        /* Write the size of the message (in bits) */
        CSEC_HAL_WriteCommandWords(FSL_FEATURE_CSEC_MESSAGE_LENGTH_OFFSET, &msgLen, 1U);
        /* Write the command header. This will trigger the command execution. */
        CSEC_HAL_WriteCommandHeader(CSEC_CMD_GENERATE_MAC, CSEC_FUNC_FORMAT_COPY, seq, keyId);

        /* Wait until the execution of the command is complete */
        CSEC_HAL_WaitCommandCompletion();

        /* Read the status of the execution */
        stat = CSEC_HAL_ReadErrorBits();
        if (stat != CSEC_NO_ERROR)
        {
            break;
        }

        numBytesLeft = numBytesLeft - numBytes;
        index = index + numBytes;
        if (seq == CSEC_CALL_SEQ_FIRST)
        {
            seq = CSEC_CALL_SEQ_SUBSEQUENT;
        }
    }

    /* Read the resulted CMAC */
    if (stat == CSEC_NO_ERROR)
    {
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_2_OFFSET, cmac, CSEC_PAGE_SIZE_IN_BYTES);
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_GenerateMACAddrMode
 * Description   : This function calculates the MAC of a given message using
 * CMAC with AES-128. It is different from the CSEC_DRV_GenerateMAC function in
 * the sense that it does not involve an extra copy of the data on which the
 * CMAC is computed and the message pointer should be a pointer to Flash memory.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_GenerateMACAddrMode(csec_key_id_t keyId,
                                               const uint8_t *msg,
                                               uint32_t msgLen,
                                               uint8_t *cmac)
{
    csec_error_code_t stat;

    /* Write the address of the message */
    CSEC_HAL_WriteCommandWords(FSL_FEATURE_CSEC_FLASH_START_ADDRESS_OFFSET, (uint32_t *)&msg, 1U);
    /* Write the size of the message (in bits) */
    CSEC_HAL_WriteCommandWords(FSL_FEATURE_CSEC_MESSAGE_LENGTH_OFFSET, &msgLen, 1U);
    /* Write the command header. This will trigger the command execution. */
    CSEC_HAL_WriteCmdAndWait(CSEC_CMD_GENERATE_MAC, CSEC_FUNC_FORMAT_ADDR, CSEC_CALL_SEQ_FIRST, keyId);

    /* Read the status of the execution */
    stat = CSEC_HAL_ReadErrorBits();
    /* Read the resulted MAC */
    if (stat == CSEC_NO_ERROR)
    {
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_2_OFFSET, cmac, CSEC_PAGE_SIZE_IN_BYTES);
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_VerifyMAC
 * Description   : This function verifies the MAC of a given message using CMAC
 * with AES-128.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_VerifyMAC(csec_key_id_t keyId,
                                     const uint8_t *msg,
                                     uint32_t msgLen,
                                     const uint8_t *mac,
                                     uint16_t macLen,
                                     bool *verifStatus)
{
    csec_call_sequence_t seq = CSEC_CALL_SEQ_FIRST;
    uint32_t numBytesLeft = CSEC_DRV_RoundTo(msgLen, 0x8) >> CSEC_BYTES_TO_FROM_BITS_SHIFT;
    csec_error_code_t stat = CSEC_NO_ERROR;
    uint8_t index = 0;
    bool macWritten = false;

    /* Loop and launch commands until the end of the message */
    while (!macWritten)
    {
        uint8_t numBytes = (uint8_t)((numBytesLeft > CSEC_DATA_BYTES_AVAILABLE) ?
                                     CSEC_DATA_BYTES_AVAILABLE : numBytesLeft);
        /* Compute the offset of the MAC inside the CSE_PRAM. The MAC parameter
        is located after the message, starting from a new page */
        uint8_t macOffset = (uint8_t)CSEC_DRV_RoundTo(numBytes, 0x10);

        /* Write the message for which the MAC will be verified */
        if (numBytes > 0U)
        {
            CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, &msg[index], numBytes);
        }
        /* If there is available space in CSE_PRAM, write the MAC to be verified */
        if ((macOffset + CSEC_PAGE_SIZE_IN_BYTES) < CSEC_DATA_BYTES_AVAILABLE)
        {
            CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET + macOffset, mac, CSEC_PAGE_SIZE_IN_BYTES);
            macWritten = true;
        }
        /* Write the size of the message (in bits) */
        CSEC_HAL_WriteCommandWords(FSL_FEATURE_CSEC_MESSAGE_LENGTH_OFFSET, &msgLen, 1U);
        /* Write the number of bits of the MAC to be compared */
        CSEC_HAL_WriteCommandHalfWord(FSL_FEATURE_CSEC_MAC_LENGTH_OFFSET, macLen);
        /* Write the command header. This will trigger the command execution. */
        CSEC_HAL_WriteCommandHeader(CSEC_CMD_VERIFY_MAC, CSEC_FUNC_FORMAT_COPY, seq, keyId);

        /* Wait until the execution of the command is complete */
        CSEC_HAL_WaitCommandCompletion();

        /* Read the status of the execution */
        stat = CSEC_HAL_ReadErrorBits();
        if (stat != CSEC_NO_ERROR)
        {
            break;
        }

        numBytesLeft = numBytesLeft - numBytes;
        index = index + numBytes;
        if (seq == CSEC_CALL_SEQ_FIRST)
        {
            seq = CSEC_CALL_SEQ_SUBSEQUENT;
        }
    }

    /* Read the result of the verification */
    if (stat == CSEC_NO_ERROR)
    {
        *verifStatus = (CSEC_HAL_ReadCommandHalfWord(FSL_FEATURE_CSEC_VERIFICATION_STATUS_OFFSET) == 0U);
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_VerifyMACAddrMode
 * Description   : This function verifies the MAC of a given message using CMAC
 * with AES-128. It is different from the CSEC_DRV_VerifyMAC function in the
 * sense that it does not involve an extra copy of the data on which the CMAC is
 * computed and the message pointer should be a pointer to Flash memory.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_VerifyMACAddrMode(csec_key_id_t keyId,
                                             const uint8_t *msg,
                                             uint32_t msgLen,
                                             const uint8_t *mac,
                                             uint16_t macLen,
                                             bool *verifStatus)
{
    csec_error_code_t stat;

    /* Write the address of the message */
    CSEC_HAL_WriteCommandWords(FSL_FEATURE_CSEC_FLASH_START_ADDRESS_OFFSET, (uint32_t *)&msg, 1U);
    /* Write the MAC to be verified */
    CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_2_OFFSET, mac, CSEC_PAGE_SIZE_IN_BYTES);
    /* Write the size of the message (in bits) */
    CSEC_HAL_WriteCommandWords(FSL_FEATURE_CSEC_MESSAGE_LENGTH_OFFSET, &msgLen, 1U);
    /* Write the number of bits of the MAC to be compared */
    CSEC_HAL_WriteCommandHalfWord(FSL_FEATURE_CSEC_MAC_LENGTH_OFFSET, macLen);
    /* Write the command header. This will trigger the command execution. */
    CSEC_HAL_WriteCmdAndWait(CSEC_CMD_VERIFY_MAC, CSEC_FUNC_FORMAT_ADDR, CSEC_CALL_SEQ_FIRST, keyId);
    
    /* Read the status of the execution */
    stat = CSEC_HAL_ReadErrorBits();

    /* Read the result of the MAC verification */
    if (stat == CSEC_NO_ERROR)
    {
        *verifStatus = (CSEC_HAL_ReadCommandHalfWord(FSL_FEATURE_CSEC_VERIFICATION_STATUS_OFFSET) == 0U);
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_LoadKey
 * Description   : This function updates an internal key per the SHE
 * specification.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_LoadKey(csec_key_id_t keyId,
                                   const uint8_t *m1,
                                   const uint8_t *m2,
                                   const uint8_t *m3,
                                   uint8_t *m4,
                                   uint8_t *m5)
{
    csec_error_code_t stat;

    /* Write the values of M1-M3 */
    CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, m1, CSEC_M1_SIZE_IN_BYTES);
    CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_2_OFFSET, m2, CSEC_M2_SIZE_IN_BYTES);
    CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_4_OFFSET, m3, CSEC_M3_SIZE_IN_BYTES);
    /* Write the command header. This will trigger the command execution. */
    CSEC_HAL_WriteCommandHeader(CSEC_CMD_LOAD_KEY, CSEC_FUNC_FORMAT_COPY, CSEC_CALL_SEQ_FIRST, keyId);

    /* Wait until the execution of the command is complete */
    CSEC_HAL_WaitCommandCompletion();

    /* Read the status of the execution */
    stat = CSEC_HAL_ReadErrorBits();

    /* Read the obtained M4 and M5 */
    if (stat == CSEC_NO_ERROR)
    {
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_5_OFFSET, m4, CSEC_M4_SIZE_IN_BYTES);
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_7_OFFSET, m5, CSEC_M5_SIZE_IN_BYTES);
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_LoadPlainKey
 * Description   : Updates the RAM key memory slot with a 128-bit plaintext.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_LoadPlainKey(const uint8_t *plainKey)
{
    /* Write the bytes of the key */
    CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, plainKey, CSEC_PAGE_SIZE_IN_BYTES);
    /* Write the command header. This will trigger the command execution. */
    CSEC_HAL_WriteCommandHeader(CSEC_CMD_LOAD_PLAIN_KEY, CSEC_FUNC_FORMAT_COPY, CSEC_CALL_SEQ_FIRST, CSEC_RAM_KEY);

    /* Wait until the execution of the command is complete */
    CSEC_HAL_WaitCommandCompletion();

    /* Read the status of the execution */
    return CSEC_HAL_ReadErrorBits();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_ExportRAMKey
 * Description   : This function exports the RAM_KEY into a format protected by
 * SECRET_KEY.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_ExportRAMKey(uint8_t *m1,
                                        uint8_t *m2,
                                        uint8_t *m3,
                                        uint8_t *m4,
                                        uint8_t *m5)
{
    csec_error_code_t stat;

    /* Write the command header. This will trigger the command execution. */
    CSEC_HAL_WriteCommandHeader(CSEC_CMD_EXPORT_RAM_KEY, CSEC_FUNC_FORMAT_COPY, CSEC_CALL_SEQ_FIRST, CSEC_RAM_KEY);

    /* Wait until the execution of the command is complete */
    CSEC_HAL_WaitCommandCompletion();

    /* Read the status of the execution */
    stat = CSEC_HAL_ReadErrorBits();
    if (stat == CSEC_NO_ERROR)
    {
        /* Read the M1-M5 values associated with the key */
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, m1, CSEC_M1_SIZE_IN_BYTES);
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_2_OFFSET, m2, CSEC_M2_SIZE_IN_BYTES);
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_4_OFFSET, m3, CSEC_M3_SIZE_IN_BYTES);
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_5_OFFSET, m4, CSEC_M4_SIZE_IN_BYTES);
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_7_OFFSET, m5, CSEC_M5_SIZE_IN_BYTES);
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_InitRNG
 * Description   : The function initializes the seed and derives a key for the
 * PRNG. The function must be called before CMD_RND after every power cycle/reset.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_InitRNG()
{
    /* Write the command header. This will trigger the command execution. */
    CSEC_HAL_WriteCommandHeader(CSEC_CMD_INIT_RNG, CSEC_FUNC_FORMAT_COPY, CSEC_CALL_SEQ_FIRST, CSEC_SECRET_KEY);

    /* Wait until the execution of the command is complete */
    CSEC_HAL_WaitCommandCompletion();

    /* Read the status of the execution */
    return CSEC_HAL_ReadErrorBits();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_ExtendSeed
 * Description   : Extends the seed of the PRNG by compressing the former seed
 * value and the supplied entropy into a new seed. This new seed is then to be
 * used to generate a random number by invoking the CMD_RND command. The random
 * number generator must be initialized by CMD_INIT_RNG before the seed may be
 * extended.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_ExtendSeed(const uint8_t *entropy)
{
    /* Write the entropy parameter */
    CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, entropy, CSEC_PAGE_SIZE_IN_BYTES);
    /* Write the command header. This will trigger the command execution. */
    CSEC_HAL_WriteCommandHeader(CSEC_CMD_EXTEND_SEED, CSEC_FUNC_FORMAT_COPY, CSEC_CALL_SEQ_FIRST, CSEC_SECRET_KEY);

    /* Wait until the execution of the command is complete */
    CSEC_HAL_WaitCommandCompletion();

    /* Read the status of the execution */
    return CSEC_HAL_ReadErrorBits();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_GenerateRND
 * Description   : The function returns a vector of 128 random bits. The random
 * number generator has to be initialized by calling CSEC_DRV_InitRNG before
 * random numbers can be supplied.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_GenerateRND(uint8_t *rnd)
{
    csec_error_code_t stat;

    /* Write the command header. This will trigger the command execution. */
    CSEC_HAL_WriteCommandHeader(CSEC_CMD_RND, CSEC_FUNC_FORMAT_COPY, CSEC_CALL_SEQ_FIRST, CSEC_SECRET_KEY);

    /* Wait until the execution of the command is complete */
    CSEC_HAL_WaitCommandCompletion();

    /* Read the status of the execution */
    stat = CSEC_HAL_ReadErrorBits();
    /* Read the resulted random bytes */
    if (stat == CSEC_NO_ERROR)
    {
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, rnd, CSEC_PAGE_SIZE_IN_BYTES);
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_BootFailure
 * Description   : The function is called during later stages of the boot
 * process to detect a failure.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_BootFailure()
{
    /* Write the command header. This will trigger the command execution. */
    CSEC_HAL_WriteCommandHeader(CSEC_CMD_BOOT_FAILURE, CSEC_FUNC_FORMAT_COPY, CSEC_CALL_SEQ_FIRST, CSEC_SECRET_KEY);

    /* Wait until the execution of the command is complete */
    CSEC_HAL_WaitCommandCompletion();

    /* Read the status of the execution */
    return CSEC_HAL_ReadErrorBits();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_BootOK
 * Description   : The function is called during later stages of the boot
 * process to mark successful boot verification.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_BootOK()
{
    /* Write the command header. This will trigger the command execution. */
    CSEC_HAL_WriteCommandHeader(CSEC_CMD_BOOT_OK, CSEC_FUNC_FORMAT_COPY, CSEC_CALL_SEQ_FIRST, CSEC_SECRET_KEY);

    /* Wait until the execution of the command is complete */
    CSEC_HAL_WaitCommandCompletion();

    /* Read the status of the execution */
    return CSEC_HAL_ReadErrorBits();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_BootDefine
 * Description   : The function implements an extension of the SHE standard to
 * define both the user boot size and boot method.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_BootDefine(uint32_t bootSize,
                                      csec_boot_flavor_t bootFlavor)
{
    uint8_t flavor = (uint8_t) bootFlavor;

    /* Write the boot size and the boot flavor parameters */
    CSEC_HAL_WriteCommandWords(FSL_FEATURE_CSEC_BOOT_SIZE_OFFSET, &bootSize, 1U);
    CSEC_HAL_WriteCommandByte(FSL_FEATURE_CSEC_BOOT_FLAVOR_OFFSET, flavor);
    /* Write the command header. This will trigger the command execution. */
    CSEC_HAL_WriteCommandHeader(CSEC_CMD_BOOT_DEFINE, CSEC_FUNC_FORMAT_COPY, CSEC_CALL_SEQ_FIRST, CSEC_SECRET_KEY);

    /* Wait until the execution of the command is complete */
    CSEC_HAL_WaitCommandCompletion();

    /* Read the status of the execution */
    return CSEC_HAL_ReadErrorBits();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_GetID
 * Description   : This function returns the identity (UID) and the value of the
 * status register protected by a MAC over a challenge and the data.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_GetID(const uint8_t *challenge,
                                 uint8_t *uid,
                                 uint8_t *sreg,
                                 uint8_t *mac)
{
    csec_error_code_t stat;

    /* Write the challenge */
    CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, challenge, CSEC_PAGE_SIZE_IN_BYTES);
    /* Write the command header. This will trigger the command execution. */
    CSEC_HAL_WriteCommandHeader(CSEC_CMD_GET_ID, CSEC_FUNC_FORMAT_COPY, CSEC_CALL_SEQ_FIRST, CSEC_SECRET_KEY);

    /* Wait until the execution of the command is complete */
    CSEC_HAL_WaitCommandCompletion();

    /* Read the status of the execution */
    stat = CSEC_HAL_ReadErrorBits();
    if (stat == CSEC_NO_ERROR)
    {
        /* Read the UID */
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_2_OFFSET, uid, (uint8_t)(CSEC_PAGE_SIZE_IN_BYTES - 1U));
        /* Read the value of the SREG register */
        *sreg = CSEC_HAL_ReadCommandByte(FSL_FEATURE_CSEC_SREG_OFFSET);
        /* Read the MAC over the UID and the SREG */
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_3_OFFSET, mac, CSEC_PAGE_SIZE_IN_BYTES);
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_DbgChal
 * Description   : This function obtains a random number which the user shall
 * use along with the MASTER_ECU_KEY and UID to return an authorization request.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_DbgChal(uint8_t *challenge)
{
    csec_error_code_t stat;

    /* Write the command header. This will trigger the command execution. */
    CSEC_HAL_WriteCommandHeader(CSEC_CMD_DBG_CHAL, CSEC_FUNC_FORMAT_COPY, CSEC_CALL_SEQ_FIRST, CSEC_SECRET_KEY);

    /* Wait until the execution of the command is complete */
    CSEC_HAL_WaitCommandCompletion();

    /* Read the status of the execution */
    stat = CSEC_HAL_ReadErrorBits();
    /* Read the challenge generated by the CSEc module */
    if (stat == CSEC_NO_ERROR)
    {
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, challenge, CSEC_PAGE_SIZE_IN_BYTES);
    }

    return stat;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_DbgAuth
 * Description   : This function erases all keys (actual and outdated) stored in
 * NVM Memory if the authorization is confirmed by CSEc.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_DbgAuth(const uint8_t *authorization)
{
    /* Write the authorization computed from the challenge */
    CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, authorization, CSEC_PAGE_SIZE_IN_BYTES);
    /* Write the command header. This will trigger the command execution. */
    CSEC_HAL_WriteCommandHeader(CSEC_CMD_DBG_AUTH, CSEC_FUNC_FORMAT_COPY, CSEC_CALL_SEQ_FIRST, CSEC_SECRET_KEY);

    /* Wait until the execution of the command is complete */
    CSEC_HAL_WaitCommandCompletion();

    /* Read the status of the execution */
    return CSEC_HAL_ReadErrorBits();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CSEC_DRV_MPCompress
 * Description   : This function accesses a Miyaguchi-Prenell compression
 * feature within the CSEc feature set to compress the given messages.
 *
 *END**************************************************************************/
csec_error_code_t CSEC_DRV_MPCompress(const uint8_t *msg,
                                      uint16_t msgLen,
                                      uint8_t *mpCompress)
{
    csec_call_sequence_t seq = CSEC_CALL_SEQ_FIRST;
    csec_error_code_t stat = CSEC_NO_ERROR;
    uint8_t index = 0;
    uint16_t numPagesLeft = msgLen;

    /* Loop and launch commands until the end of the message */
    while (numPagesLeft > 0U)
    {
        uint8_t numPages = (uint8_t)((msgLen > CSEC_DATA_PAGES_AVAILABLE) ?
                                      CSEC_DATA_PAGES_AVAILABLE : msgLen);
        uint8_t numBytes = (uint8_t)(numPages << CSEC_BYTES_TO_FROM_PAGES_SHIFT);

        /* Write the message */
        CSEC_HAL_WriteCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, &msg[index], numBytes);
        /* Write the size of the message */
        CSEC_HAL_WriteCommandHalfWord(FSL_FEATURE_CSEC_PAGE_LENGTH_OFFSET, msgLen);
        /* Write the command header. This will trigger the command execution. */
        CSEC_HAL_WriteCommandHeader(CSEC_CMD_MP_COMPRESS, CSEC_FUNC_FORMAT_COPY, seq, CSEC_SECRET_KEY);

        /* Wait until the execution of the command is complete */
        CSEC_HAL_WaitCommandCompletion();

        /* Read the status of the execution */
        stat = CSEC_HAL_ReadErrorBits();
        if (stat != CSEC_NO_ERROR)
        {
            break;
        }

        numPagesLeft = numPagesLeft - numPages;
        index = index + numBytes;
        if (seq == CSEC_CALL_SEQ_FIRST)
        {
            seq = CSEC_CALL_SEQ_SUBSEQUENT;
        }
    }

    /* Read the result of the compression */
    if (stat == CSEC_NO_ERROR)
    {
        CSEC_HAL_ReadCommandBytes(FSL_FEATURE_CSEC_PAGE_1_OFFSET, mpCompress, CSEC_PAGE_SIZE_IN_BYTES);
    }

    return stat;
}
/******************************************************************************
 * EOF
 *****************************************************************************/
