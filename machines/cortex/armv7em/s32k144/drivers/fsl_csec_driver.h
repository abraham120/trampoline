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

#ifndef FSL_CSEC_DRV_H
#define FSL_CSEC_DRV_H

#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_csec_hal.h"

/*! @file */

/*!
 * @defgroup csec_driver CSEc Driver
 * @ingroup csec
 * @brief Cryptographic Services Engine Peripheral Driver.
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Specifies the boot type for the BOOT_DEFINE command. */
typedef enum {
    CSEC_BOOT_STRICT,
    CSEC_BOOT_SERIAL,
    CSEC_BOOT_PARALLEL,
    CSEC_BOOT_NOT_DEFINED
} csec_boot_flavor_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Performs the AES-128 encryption in ECB mode.
 *
 * This function performs the AES-128 encryption in ECB mode of the input
 * plain text buffer
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] plainText Pointer to the plain text buffer.
 * @param[in] length Number of bytes of plain text message to be encrypted.
 * It should be multiple of 16 bytes.
 * @param[out] cipherText Pointer to the cipher text buffer. The buffer shall
 * have the same size as the plain text buffer.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_EncryptECB(csec_key_id_t keyId,
    const uint8_t *plainText, uint32_t length, uint8_t *cipherText);

/*!
 * @brief Performs the AES-128 decryption in ECB mode.
 *
 * This function performs the AES-128 decryption in ECB mode of the input
 * cipher text buffer.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation
 * @param[in] cipherText Pointer to the cipher text buffer.
 * @param[in] length Number of bytes of cipher text message to be decrypted.
 * It should be multiple of 16 bytes.
 * @param[out] plainText Pointer to the plain text buffer. The buffer shall
 * have the same size as the cipher text buffer.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_DecryptECB(csec_key_id_t keyId, const uint8_t *cipherText,
    uint32_t length, uint8_t *plainText);

    /*!
 * @brief Performs the AES-128 encryption in CBC mode.
 *
 * This function performs the AES-128 encryption in CCB mode of the input
 * plaintext buffer.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] plainText Pointer to the plain text buffer.
 * @param[in] length Number of bytes of plain text message to be encrypted.
 * It should be multiple of 16 bytes.
 * @param[in] iv Pointer to the initialization vector buffer.
 * @param[out] cipherText Pointer to the cipher text buffer. The buffer shall
 * have the same size as the plain text buffer.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_EncryptCBC(csec_key_id_t keyId,
    const uint8_t *plainText, uint32_t length,
    const uint8_t *iv, uint8_t *cipherText);

/*!
 * @brief Performs the AES-128 decryption in CBC mode.
 *
 * This function performs the AES-128 decryption in CBC mode of the input
 * cipher text buffer.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] cipherText Pointer to the cipher text buffer.
 * @param[in] length Number of bytes of cipher text message to be decrypted.
 * It should be multiple of 16 bytes.
 * @param[in] iv Pointer to the initialization vector buffer.
 * @param[out] plainText Pointer to the plain text buffer. The buffer shall
 * have the same size as the cipher text buffer.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_DecryptCBC(csec_key_id_t keyId, const uint8_t *cipherText,
    uint16_t length, const uint8_t* iv, uint8_t *plainText);

/*!
 * @brief Calculates the MAC of a given message using CMAC with AES-128.
 *
 * This function calculates the MAC of a given message using CMAC with AES-128.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer.
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[out] cmac Pointer to the buffer containing the result of the CMAC
 * computation.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_GenerateMAC(csec_key_id_t keyId, const uint8_t *msg,
    uint32_t msgLen, uint8_t *cmac);

/*!
 * @brief Calculates the MAC of a given message (located in Flash) using CMAC
 * with AES-128.
 *
 * This function calculates the MAC of a given message using CMAC with AES-128.
 * It is different from the CSEC_DRV_GenerateMAC function in the sense that it
 * does not involve an extra copy of the data on which the CMAC is computed and
 * the message pointer should be a pointer to Flash memory.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer (pointing to Flash memory).
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[out] cmac Pointer to the buffer containing the result of the CMAC
 * computation.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_GenerateMACAddrMode(csec_key_id_t keyId,
    const uint8_t *msg, uint32_t msgLen, uint8_t *cmac);

/*!
 * @brief Verifies the MAC of a given message using CMAC with AES-128.
 *
 * This function verifies the MAC of a given message using CMAC with AES-128.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer.
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[in] mac Pointer to the buffer containing the CMAC to be verified.
 * @param[in] macLen Number of bits of the CMAC to be compared. A macLength
 * value of zero indicates that all 128-bits are compared.
 * @param[out] verifStatus Status of MAC verification command (true:
 * verification operation passed, false: verification operation failed).
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_VerifyMAC(csec_key_id_t keyId, const uint8_t *msg,
    uint32_t msgLen, const uint8_t *mac, uint16_t macLen, bool *verifStatus);

/*!
 * @brief Verifies the MAC of a given message (located in Flash) using CMAC with
 * AES-128.
 *
 * This function verifies the MAC of a given message using CMAC with AES-128.
 * It is different from the CSEC_DRV_VerifyMAC function in the sense that it
 * does not involve an extra copy of the data on which the CMAC is computed and
 * the message pointer should be a pointer to Flash memory.
 *
 * @param[in] keyId KeyID used to perform the cryptographic operation.
 * @param[in] msg Pointer to the message buffer (pointing to Flash memory).
 * @param[in] msgLen Number of bits of message on which CMAC will be computed.
 * @param[in] mac Pointer to the buffer containing the CMAC to be verified.
 * @param[in] macLen Number of bits of the CMAC to be compared. A macLength
 * value of zero indicates that all 128-bits are compared.
 * @param[out] verifStatus Status of MAC verification command (true:
 * verification operation passed, false: verification operation failed).
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_VerifyMACAddrMode(csec_key_id_t keyId, const uint8_t *msg,
    uint32_t msgLen, const uint8_t *mac, uint16_t macLen, bool *verifStatus);

/*!
 * @brief Updates an internal key per the SHE specification.
 *
 * This function updates an internal key per the SHE specification.
 *
 * @param[in] keyId KeyID of the key to be updated.
 * @param[in] m1 Pointer to the 128-bit M1 message containing the UID, Key ID
 * and Authentication Key ID.
 * @param[in] m2 Pointer to the 256-bit M2 message contains the new security
 * flags, counter and the key value all encrypted using a derived key generated
 * from the Authentication Key.
 * @param[in] m3 Pointer to the 128-bit M3 message is a MAC generated over
 * messages M1 and M2.
 * @param[out] m4 Pointer to a 256 bits buffer where the computed M4 parameter
 * is stored.
 * @param[out] m5 Pointer to a 128 bits buffer where the computed M5 parameters
 * is stored.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_LoadKey(csec_key_id_t keyId, const uint8_t *m1,
    const uint8_t *m2, const uint8_t *m3, uint8_t *m4, uint8_t *m5);

/*!
 * @brief Updates the RAM key memory slot with a 128-bit plaintext.
 *
 * The function updates the RAM key memory slot with a 128-bit plaintext. The
 * key is loaded without encryption and verification of the key, i.e. the key is
 * handed over in plaintext. A plain key can only be loaded into the RAM_KEY
 * slot.
 *
 * @param[in] plainKey Pointer to the 128-bit buffer containing the key that
 * needs to be copied in RAM_KEY slot.
 * @return Error Code after command execution.
 */
csec_error_code_t CSEC_DRV_LoadPlainKey(const uint8_t *plainKey);

/*!
 * @brief Exports the RAM_KEY into a format protected by SECRET_KEY.
 *
 * This function exports the RAM_KEY into a format protected by SECRET_KEY.
 *
 * @param[out] m1 Pointer to a buffer where the M1 parameter will be exported.
 * @param[out] m2 Pointer to a buffer where the M2 parameter will be exported.
 * @param[out] m3 Pointer to a buffer where the M3 parameter will be exported.
 * @param[out] m4 Pointer to a buffer where the M4 parameter will be exported.
 * @param[out] m5 Pointer to a buffer where the M5 parameter will be exported.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_ExportRAMKey(uint8_t *m1, uint8_t *m2,
    uint8_t *m3, uint8_t *m4, uint8_t *m5);

/*!
 * @brief Initializes the seed and derives a key for the PRNG.
 *
 * The function initializes the seed and derives a key for the PRNG.
 * The function must be called before CMD_RND after every power cycle/reset.
 *
 * @return Error Code after command execution.
 */
csec_error_code_t CSEC_DRV_InitRNG(void);

/*!
 * @brief Extends the seed of the PRNG.
 *
 * Extends the seed of the PRNG by compressing the former seed value and the
 * supplied entropy into a new seed. This new seed is then to be used to
 * generate a random number by invoking the CMD_RND command. The random number
 * generator must be initialized by CMD_INIT_RNG before the seed may be
 * extended.
 *
 * @param[in] entropy Pointer to a 128-bit buffer containing the entropy.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_ExtendSeed(const uint8_t *entropy);

/*!
 * @brief Generates a vector of 128 random bits.
 *
 * The function returns a vector of 128 random bits. The random number generator
 * has to be initialized by calling CSEC_DRV_InitRNG before random numbers can
 * be supplied.
 *
 * @param[out] rnd Pointer to a 128-bit buffer where the generated random number
 * has to be stored.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_GenerateRND(uint8_t *rnd);

/*!
 * @brief Signals a failure detected during later stages of the boot process.
 *
 * The function is called during later stages of the boot process to detect a
 * failure.
 *
 * @return Error Code after command execution.
 */
csec_error_code_t CSEC_DRV_BootFailure(void);

/*!
 * @brief Marks a successful boot verification during later stages of the boot
 * process.
 *
 * The function is called during later stages of the boot process to mark
 * successful boot verification.
 *
 * @return Error Code after command execution.
 */
csec_error_code_t CSEC_DRV_BootOK(void);

/*!
 * @brief Implements an extension of the SHE standard to define both the user
 * boot size and boot method.
 *
 * The function implements an extension of the SHE standard to define both the
 * user boot size and boot method.
 *
 * @param[in] bootSize Number of blocks of 128-bit data to check on boot.
 * Maximum size is 512kBytes.
 * @param[in] bootFlavor The boot method.
 * @return Error Code after command execution.
 */
csec_error_code_t CSEC_DRV_BootDefine(uint32_t bootSize, csec_boot_flavor_t bootFlavor);

/*!
 * @brief Returns the content of the status register.
 *
 * The function shall return the content of the status register.
 *
 * @return Value of the status register.
 */
static inline csec_status_t CSEC_DRV_GetStatus(void)
{
    return CSEC_HAL_ReadStatus();
}

/*!
 * @brief Returns the identity (UID) and the value of the status register
 * protected by a MAC over a challenge and the data.
 *
 * This function returns the identity (UID) and the value of the status register
 * protected by a MAC over a challenge and the data.
 *
 * @param[in] challenge Pointer to the 128-bit buffer containing Challenge data.
 * @param[out] uid Pointer to 120 bit buffer where the UID will be stored.
 * @param[out] sreg Value of the status register.
 * @param[out] mac Pointer to the 128 bit buffer where the MAC generated over
 * challenge and UID and status  will be stored.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_GetID(const uint8_t *challenge, uint8_t *uid,
    uint8_t *sreg, uint8_t *mac);

/*!
 * @brief Obtains a random number which the user shall use along with the
 * MASTER_ECU_KEY and UID to return an authorization request.
 *
 * This function obtains a random number which the user shall use along with the
 * MASTER_ECU_KEY and UID to return an authorization request.
 *
 * @param[out] challenge Pointer to the 128-bit buffer where the challenge data
 * will be stored.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_DbgChal(uint8_t *challenge);

/*!
 * @brief Erases all keys (actual and outdated) stored in NVM Memory if the
 * authorization is confirmed by CSEc.
 *
 * This function erases all keys (actual and outdated) stored in NVM Memory if
 * the authorization is confirmed by CSEc.
 *
 * @param[in] authorization Pointer to the 128-bit buffer containing the
 * authorization value.
 * @return Error Code after command execution.
 */
csec_error_code_t CSEC_DRV_DbgAuth(const uint8_t *authorization);

/*!
 * @brief Compresses the given messages by accessing  the Miyaguchi-Prenell
 * compression feature with in the CSEc feature set.
 *
 * This function accesses a Miyaguchi-Prenell compression feature within the
 * CSEc feature set to compress the given messages.
 *
 * @param[in] msg Pointer to the messages to be compressed. Messages must be
 * pre-processed per SHE specification if they do not already meet the full
 * 128-bit block size requirement.
 * @param[in] msgLen The number of 128 bit messages to be compressed.
 * @param[out] mpCompress Pointer to the 128 bit buffer storing the compressed
 * data.
 * @return Error Code after command execution. Output parameters are valid if
 * the error code is CSEC_NO_ERROR.
 */
csec_error_code_t CSEC_DRV_MPCompress(const uint8_t *msg, uint16_t msgLen,
    uint8_t *mpCompress);


#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* FSL_CSEC_DRV_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
