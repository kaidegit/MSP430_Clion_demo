/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#ifndef __MSP430WARE_AES256_H__
#define __MSP430WARE_AES256_H__

//*****************************************************************************
//
//The following are the defines to include the required modules for this
//peripheral in msp430xgeneric.h file
//
//*****************************************************************************
#define __MSP430_HAS_AES256__

//*****************************************************************************
//
//The following value can be passed to AES256_clearInterruptFlag(),
//AES256_enableInterrupt(), AES256_disableInterrupt()
//
//*****************************************************************************
#define AES256_READY_INTERRUPT AESRDYIE

//*****************************************************************************
//
//The following value can be returned by AES256_getErrorFlagStatus
//
//*****************************************************************************
#define AES256_ERROR_OCCURRED	AESERRFG
#define AES256_NO_ERROR			0x00

//*****************************************************************************
//
//The following value can be passed to AES256_enableInterrupt()
//
//*****************************************************************************
#define		AES_getEncryptedData	AES_getDataOut
#define		AES_getDecryptedData	AES_getDataOut

//*****************************************************************************
//
//The following value is returned by the AES256_isBusy() API
//
//*****************************************************************************
#define AES256_BUSY		AESBUSY
#define AES256_NOT_BUSY	0x00

//*****************************************************************************
//
//The following value can be passed to AES256 functions as KeyLength
//
//*****************************************************************************
#define Key_128BIT	128
#define Key_192BIT	192
#define Key_256BIT	256

//*****************************************************************************
//
//The following value can be passed to AES_enableInterrupt()
//
//*****************************************************************************
#define AES_INTERRUPT_ENABLE AESRDYI

//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************
unsigned char AES256_setCipherKey (unsigned int baseAddress,
	const unsigned char * CipherKey, 
 unsigned int KeyLength);
unsigned char AES256_encryptData (unsigned int baseAddress,
	const unsigned char * Data, 
	unsigned char * encryptedData);
unsigned char AES256_decryptDataUsingEncryptionKey (unsigned int baseAddress,
	const unsigned char * Data, 
	unsigned char * decryptedData);
unsigned char AES256_decryptData (unsigned int baseAddress,
	const unsigned char * Data, 
	unsigned char * decryptedData);
unsigned char AES256_generateFirstRoundKey (unsigned int baseAddress,
	const unsigned char * CipherKey);
void AES256_clearInterruptFlag (unsigned int baseAddress );
unsigned long AES256_getInterruptFlagStatus (unsigned int baseAddress);
void AES256_enableInterrupt (unsigned int baseAddress);
void AES256_disableInterrupt (unsigned int baseAddress);
void AES256_reset (unsigned int baseAddress);
unsigned char AES256_startEncryptData (unsigned int baseAddress,
	const unsigned char * Data, 
	unsigned char * encryptedData);
unsigned char AES256_startDecryptDataUsingEncryptionKey (
	unsigned int baseAddress, 
	const unsigned char * Data);
unsigned char AES256_startDecryptData (unsigned int baseAddress,
	const unsigned char * Data);
unsigned char AES256_startGenerateFirstRoundKey (unsigned int baseAddress,
	const unsigned char * CipherKey);
unsigned char  AES256_getDataOut(unsigned int baseAddress,
							unsigned char *OutputData
							);
unsigned char AES256_isBusy (unsigned int baseAddress);
void AES256_clearErrorFlag (unsigned int baseAddress );
unsigned long AES256_getErrorFlagStatus (unsigned int baseAddress);
#endif

