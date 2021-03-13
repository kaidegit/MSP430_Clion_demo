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
//*****************************************************************************
//
//aes.c - Driver for the AES Module.
//
//*****************************************************************************
#include "inc/hw_types.h"
#include "driverlib/5xx_6xx/debug.h"
#include "driverlib/5xx_6xx/aes.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "deprecated/CCS/msp430xgeneric.h"
#endif

//*****************************************************************************
//
//! Loads a 128 bit cipher key to AES module.
//!
//! \param baseAddress is the base address of the AES module.
//! \param CipherKey is a pointer to an unsigned char array with a length of 16
//!      bytes that contains a 128 bit cipher key.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
unsigned char AES_setCipherKey (unsigned int baseAddress, 
	 const unsigned char * CipherKey
	 )
{
	unsigned char i = 0;
	unsigned int tempVariable = 0;

	// Wait until AES accelerator is busy
	while(AESBUSY == (HWREG(baseAddress + OFS_AESASTAT) & AESBUSY) );
	
	for (i = 0; i < 16; i = i + 2)
	{
		//HWREG(baseAddress + OFS_AESAKEY) = ( unsigned int)(( unsigned int)CipherKey[i] | ( unsigned int) (CipherKey[i + 1] << 8));
		tempVariable = (unsigned int)(CipherKey[i]);
		tempVariable = tempVariable | ((unsigned int)(CipherKey[i + 1]) << 8);
		HWREG(baseAddress + OFS_AESAKEY) = tempVariable;
	}
    
    // Wait until key is written
	while(0x00 == (HWREG(baseAddress + OFS_AESASTAT) & AESKEYWR ));
  	
    return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Encrypts a block of data using the AES module.
//! The cipher key that is used for encryption should be loaded in advance
//! by using function \param AES_setCipherKey().
//!
//! \param baseAddress is the base address of the AES module.
//! \param Data is a pointer to an unsigned char array with a length of 16
//!      bytes that contains data to be encrypted.
//! \param encryptedData is a pointer to an unsigned char array with a length
//!      of 16 bytes in that the encrypted data will be written.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
unsigned char AES_encryptData (unsigned int baseAddress, 
	const unsigned char * Data, 
	unsigned char * encryptedData)
{
	unsigned char i;
	unsigned int tempData = 0;
	unsigned int tempVariable = 0;
	
	// Set module to encrypt mode
	HWREG(baseAddress + OFS_AESACTL0) &= ~AESOP_3;
	
			
	// Write data to encrypt to module
	for (i = 0; i < 16; i = i + 2)
	{
		//HWREG(baseAddress + OFS_AESADIN) = ( unsigned int)(( unsigned int)Data[i] | ( unsigned int) (Data[i + 1] << 8));
		tempVariable = (unsigned int)(Data[i+1]);
		tempVariable = tempVariable | ((unsigned int)(Data[i ]) << 8);
		HWREG(baseAddress + OFS_AESADIN) = tempVariable;
	}
	
	// Key that is already written shall be used
	// Encryption is initialized by setting AESKEYWR to 1
	HWREG(baseAddress + OFS_AESASTAT) |= AESKEYWR;

	// Wait unit finished ~167 MCLK
	while(AESBUSY == (HWREG(baseAddress + OFS_AESASTAT) & AESBUSY) );
	
	// Write encrypted data back to variable
	for (i = 0; i < 16; i = i + 2)
	{
		tempData = HWREG(baseAddress + OFS_AESADOUT);
		*(encryptedData + i +1) = (unsigned char)tempData;
		*(encryptedData +i) = (unsigned char)(tempData >> 8);


	}
	
    return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Decryptes a block of data using the AES module.
//! This function can be used to decrypt data by using the same key as used
//! for a previous performed encryption.
//! The decryption takes 214 MCLK.
//!
//! \param baseAddress is the base address of the AES module.
//! \param Data is a pointer to an unsigned char array with a length of 16
//!      bytes that contains encrypted data to be decrypted.
//! \param decryptedData is a pointer to an unsigned char array with a length
//!      of 16 bytes in that the decrypted data will be written.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
unsigned char AES_decryptDataUsingEncryptionKey (unsigned int baseAddress, 
	const unsigned char * Data, 
	unsigned char * decryptedData)
{
	unsigned char i;
	unsigned int tempData = 0;
	unsigned int tempVariable = 0;
	
	// Set module to decrypt mode
	HWREG(baseAddress + OFS_AESACTL0) &= ~(AESOP1);
	HWREG(baseAddress + OFS_AESACTL0) |= AESOP0;
	
	// Write data to decrypt to module
	for (i = 0; i < 16; i = i + 2)
	{
		//HWREG(baseAddress + OFS_AESADIN) = ( unsigned int)(( unsigned int)Data[i] | ( unsigned int) (Data[i + 1] << 8));
		tempVariable = (unsigned int)(Data[i]  << 8);
		tempVariable = tempVariable | ((unsigned int)(Data[i + 1]));
		HWREG(baseAddress + OFS_AESADIN) = tempVariable;
	}

	// Key that is already written shall be used
	// Now decryption starts
	HWREG(baseAddress + OFS_AESASTAT) |= AESKEYWR;

	// Wait unit finished ~214 MCLK
	while(AESBUSY == (HWREG(baseAddress + OFS_AESASTAT) & AESBUSY) );
	
	// Write encrypted data back to variable
	for (i = 0; i < 16; i = i + 2)
	{
		tempData = HWREG(baseAddress + OFS_AESADOUT);
		*(decryptedData + i + 1) = (unsigned char)tempData;
		*(decryptedData +i) = (unsigned char)(tempData >> 8);
	}
	
	return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Decryptes a block of data using the AES module.
//! This function requires a pregenerated decryption key. A key can be loaded
//! and pregenerated by using function \param AES_generateFirstRoundKey() .
//! The decryption takes 167 MCLK.
//!
//! \param baseAddress is the base address of the AES module.
//! \param Data is a pointer to an unsigned char array with a length of 16
//!      bytes that contains encrypted data to be decrypted.
//! \param decryptedData is a pointer to an unsigned char array with a length
//!      of 16 bytes in that the decrypted data will be written.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
unsigned char AES_decryptData (unsigned int baseAddress, 
	const unsigned char * Data, 
	unsigned char * decryptedData)
{
	unsigned char i;
	unsigned int tempData = 0;
	unsigned int tempVariable = 0;
	
	// Set module to decrypt mode
	HWREG(baseAddress + OFS_AESACTL0) |= (AESOP_3);
		
	// Write data to decrypt to module
	for (i = 0; i < 16; i = i + 2)
	{
		//HWREG(baseAddress + OFS_AESADIN) = ( unsigned int)(( unsigned int)Data[i] | ( unsigned int) (Data[i + 1] << 8));
		tempVariable = (unsigned int)(Data[i]  << 8);
		tempVariable = tempVariable | ((unsigned int)(Data[i + 1]));
		HWREG(baseAddress + OFS_AESADIN) = tempVariable;
	}
	
	// Key that is already written shall be used
	// Now decryption starts
	HWREG(baseAddress + OFS_AESASTAT) |= AESKEYWR;

	// Wait unit finished ~167 MCLK
	while(AESBUSY == (HWREG(baseAddress + OFS_AESASTAT) & AESBUSY ));
	
	// Write encrypted data back to variable
	for (i = 0; i < 16; i = i + 2)
	{
		tempData = HWREG(baseAddress + OFS_AESADOUT);
		*(decryptedData + i + 1) = (unsigned char)tempData;
		*(decryptedData +i) = (unsigned char)(tempData >> 8);
	}
	
	return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Loads and generates first round key required for decryption
//!
//! \param baseAddress is the base address of the AES module.
//! \param CipherKey is a pointer to an unsigned char array with a length of 16
//!      bytes that contains the initial AES key.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
unsigned char AES_generateFirstRoundKey (unsigned int baseAddress, 
	const unsigned char * CipherKey)
{
	unsigned char i;
	unsigned int tempVariable = 0;
	
	// Set module to decrypt mode
	HWREG(baseAddress + OFS_AESACTL0) &= ~(AESOP0);
	HWREG(baseAddress + OFS_AESACTL0) |= AESOP1;
	
	// Write cipher key to key register
	for (i = 0; i < 16; i = i + 2)
	{
		//HWREG(baseAddress + OFS_AESAKEY) = ( unsigned int)(( unsigned int)CipherKey[i] | ( unsigned int) (CipherKey[i + 1] << 8));
	//	tempVariable = (unsigned int)((unsigned int)(CipherKey[i]) << 8);
		//tempVariable = tempVariable | (unsigned int)(CipherKey[i+1]);
		//HWREG(baseAddress + OFS_AESAKEY) = tempVariable;

		tempVariable = (unsigned int)(CipherKey[i]);
		tempVariable = tempVariable | ((unsigned int)(CipherKey[i + 1]) << 8);
		HWREG(baseAddress + OFS_AESAKEY) = tempVariable;
	}
	
	// Wait until key is processed ~52 MCLK
	while((HWREG(baseAddress + OFS_AESASTAT) & AESBUSY) == AESBUSY);
	
	return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Clears the AES ready interrupt flag.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! Modified bit is \b AESRDYIFG of \b AESACTL0 register.
//!
//! \return None
//
//*****************************************************************************
void AES_clearInterruptFlag (unsigned int baseAddress )
{
	HWREGB(baseAddress + OFS_AESACTL0) &=  ~AESRDYIFG;
}


//*****************************************************************************
//
//! Gets the AES ready interrupt flag status.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! \return unsigned long - AES_READY_INTERRUPT or 0x00.
//
//*****************************************************************************
unsigned long AES_getInterruptFlagStatus (unsigned int baseAddress)
{
    return ((HWREGB(baseAddress + OFS_AESACTL0) & AESRDYIFG) << 0x04);
}
//*****************************************************************************
//
//! Enables AES ready interrupt.
//!
//! \param baseAddress is the base address of the AES module.
//! Modified bit is \b AESRDYIE of \b AESACTL0 register.
//!
//! \return None.
//
//*****************************************************************************
void AES_enableInterrupt (unsigned int baseAddress)
{
	HWREGB(baseAddress + OFS_AESACTL0) |=  AESRDYIE;
}

//*****************************************************************************
//
//! Disables AES ready interrupt.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! Modified bit is \b AESRDYIE of \b AESACTL0 register.
//!
//! \return None.
//
//*****************************************************************************
void AES_disableInterrupt (unsigned int baseAddress)
{
	HWREGB(baseAddress + OFS_AESACTL0) &=  ~AESRDYIE;
}

//*****************************************************************************
//
//! Resets AES Module immediately.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! Modified bit is \b AESSWRST of \b AESACTL0 register.
//!
//! \return None.
//
//*****************************************************************************
void AES_reset (unsigned int baseAddress)
{
	HWREGB(baseAddress + OFS_AESACTL0) |=  AESSWRST;
}

//*****************************************************************************
//
//! Starts an encryption process on the AES module.
//! This is the non-blocking equivalant of AES_encryptData.
//! The cipher key that is used for decryption should be loaded in advance
//! by using function \param AES_setCipherKey().
//!
//! \param baseAddress is the base address of the AES module.
//! \param Data is a pointer to an unsigned char array with a length of 16
//!      bytes that contains data to be encrypted.
//! \param encryptedData is a pointer to an unsigned char array with a length
//!      of 16 bytes in that the encrypted data will be written.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
unsigned char AES_startEncryptData (unsigned int baseAddress, 
	const unsigned char * Data, 
	unsigned char * encryptedData)
{
	unsigned char i;
	unsigned int tempVariable = 0;
	
	// Set module to encrypt mode
	HWREG(baseAddress + OFS_AESACTL0) &= ~AESOP_3;
	

	// Write data to encrypt to module
	for (i = 0; i < 16; i = i + 2)
	{
		//HWREG(baseAddress + OFS_AESADIN) = ( unsigned int)(( unsigned int)Data[i] | ( unsigned int) (Data[i + 1] << 8));
		tempVariable = (unsigned int)(Data[i+1]);
		tempVariable = tempVariable | ((unsigned int)(Data[i ]) << 8);
		HWREG(baseAddress + OFS_AESADIN) = tempVariable;
	}
	
	// Key that is already written shall be used
	// Encryption is initialized by setting AESKEYWR to 1
	HWREG(baseAddress + OFS_AESASTAT) |= AESKEYWR;

	return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Starts an decryption process on the AES module.
//! This is the non-blocking equivalant of AES_decryptDataUsingEncryptionKey.
//! This function can be used to decrypt data by using the same key as used
//! for a previous performed encryption.
//! The decryption takes 214 MCLK.
//!
//! \param baseAddress is the base address of the AES module.
//! \param Data is a pointer to an unsigned char array with a length of 16
//!      bytes that contains encrypted data to be decrypted.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
unsigned char AES_startDecryptDataUsingEncryptionKey (
	unsigned int baseAddress, 
	const unsigned char * Data)
{
	unsigned char i;
	unsigned int tempVariable = 0;
	
	// Set module to decrypt mode
	HWREG(baseAddress + OFS_AESACTL0) &= ~(AESOP1);
	HWREG(baseAddress + OFS_AESACTL0) |= AESOP0;
	
	// Write data to decrypt to module
	for (i = 0; i < 16; i = i + 2)
	{
		//HWREG(baseAddress + OFS_AESADIN) = ( unsigned int)(( unsigned int)Data[i] | ( unsigned int) (Data[i + 1] << 8));
		tempVariable = (unsigned int)(Data[i]  << 8);
		tempVariable = tempVariable | ((unsigned int)(Data[i + 1]));
		HWREG(baseAddress + OFS_AESADIN) = tempVariable;
	}

	// Key that is already written shall be used
	// Now decryption starts
	HWREG(baseAddress + OFS_AESASTAT) |= AESKEYWR;
	
	return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Decryptes a block of data using the AES module.
//! This is the non-blocking equivalant of AES_decryptData.
//! This function requires a pregenerated decryption key. A key can be loaded
//! and pregenerated by using function \param AES_generateFirstRoundKey() .
//! The decryption takes 167 MCLK.
//!
//! \param baseAddress is the base address of the AES module.
//! \param Data is a pointer to an unsigned char array with a length of 16
//!      bytes that contains encrypted data to be decrypted.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
unsigned char AES_startDecryptData (unsigned int baseAddress, 
	const unsigned char * Data)
{
	unsigned char i;
	unsigned int tempVariable = 0;
	
	// Set module to decrypt mode
	HWREG(baseAddress + OFS_AESACTL0) |= (AESOP_3);

	// Write data to decrypt to module
	for (i = 0; i < 16; i = i + 2)
	{
		//HWREG(baseAddress + OFS_AESADIN) = ( unsigned int)(( unsigned int)Data[i] | ( unsigned int) (Data[i + 1] << 8));
		tempVariable = (unsigned int)(Data[i]  << 8);
		tempVariable = tempVariable | ((unsigned int)(Data[i + 1]));
		HWREG(baseAddress + OFS_AESADIN) = tempVariable;
	}
	
	// Key that is already written shall be used
	// Now decryption starts
	HWREG(baseAddress + OFS_AESASTAT) |= AESKEYWR;


	return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Loads and generates first round key required for decryption
//! This is the non-blocking equivalant of AES_generateFirstRoundKey.
//!
//! \param baseAddress is the base address of the AES module.
//! \param CipherKey is a pointer to an unsigned char array with a length of 16
//!      bytes that contains the initial AES key.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
unsigned char AES_startGenerateFirstRoundKey (unsigned int baseAddress, 
	const unsigned char * CipherKey)
{
	unsigned char i;
	unsigned int tempVariable = 0;
	
	// Set module to generate first round key required for decryption
	HWREG(baseAddress + OFS_AESACTL0) &= ~(AESOP0);
	HWREG(baseAddress + OFS_AESACTL0) |= AESOP1;
	
	// Write cipher key to key register
	for (i = 0; i < 16; i = i + 2)
	{
		//HWREG(baseAddress + OFS_AESAKEY) = ( unsigned int)(( unsigned int)CipherKey[i] | ( unsigned int) (CipherKey[i + 1] << 8));
		tempVariable = (unsigned int)(CipherKey[i]);
		tempVariable = tempVariable | ((unsigned int)(CipherKey[i+1]) << 8);
		HWREG(baseAddress + OFS_AESAKEY) = tempVariable;
	}
	
	return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Reads back the output data from AES module.
//! This function is meant to use after an encryption or decryption process
//! that was started and finished by initiating an interrupt.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! \return  is a pointer to an unsigned char array with a length of 16
//!      bytes in which the output data of the AES module is available.
//!		If AES module is busy returns NULL
//
//*****************************************************************************
unsigned char  AES_getDataOut(unsigned int baseAddress,
							unsigned char *OutputData
							)
{
	unsigned char i;
	unsigned int tempData = 0;

	// If module is busy, exit and return failure
	if( AESBUSY == (HWREG(baseAddress + OFS_AESASTAT) & AESBUSY)) 
		return STATUS_FAIL;
	
	// Write encrypted data back to variable
	for (i = 0; i < 16; i = i + 2)
	{
		tempData = HWREG(baseAddress + OFS_AESADOUT);
		*(OutputData + i + 1) = (unsigned char)tempData;
		*(OutputData +i) = (unsigned char)(tempData >> 8);
	}
	
	return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! Gets the AES module busy status.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! \return unsigned char \b AES_BUSY or \b AES_NOT_BUSY
//
//*****************************************************************************
unsigned char AES_isBusy (unsigned int baseAddress)
{
    return (HWREG(baseAddress + OFS_AESASTAT) & AESBUSY);
}

//*****************************************************************************
//
//! Clears the AES error flag.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! Modified bit is \b AESERRFG of \b AESACTL0 register.
//!
//! \return None
//
//*****************************************************************************
void AES_clearErrorFlag (unsigned int baseAddress )
{
	HWREGB(baseAddress + OFS_AESACTL0) &=  ~AESERRFG;
}

//*****************************************************************************
//
//! Gets the AES error flag status.
//!
//! \param baseAddress is the base address of the AES module.
//!
//! \return unsigned long - AES_ERROR_OCCURRED or AES_NO_ERROR.
//
//*****************************************************************************
unsigned long AES_getErrorFlagStatus (unsigned int baseAddress)
{
    return (HWREGB(baseAddress + OFS_AESACTL0) & AESERRFG);
}
