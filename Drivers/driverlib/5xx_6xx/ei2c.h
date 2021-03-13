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
#ifndef __MSP430WARE_EUSCI_I2C_H__
#define __MSP430WARE_EUSCI_I2C_H__

//*****************************************************************************
//
//The following are the defines to include the required modules for this
//peripheral in msp430xgeneric.h file
//
//*****************************************************************************
#define __MSP430_HAS_EUSCI_Bx__

//*****************************************************************************
//
//The following are values that can be passed to the I2C_masterInit() API
//as the selectClockSource parameter.
//
//*****************************************************************************
#define eI2C_CLOCKSOURCE_ACLK    UCSSEL__ACLK
#define eI2C_CLOCKSOURCE_SMCLK   UCSSEL__SMCLK

//*****************************************************************************
//
//The following are values that can be passed to the eI2C_enableInterrupt(),
//eI2C_disableInterrupt(), eI2C_clearInterrupt(), eI2C_getInterruptStatus(), API
//as the mask parameter.
//
//*****************************************************************************
#define eI2C_NAK_INTERRUPT                UCNACKIE
#define eI2C_ARBITRATIONLOST_INTERRUPT    UCALIE
#define eI2C_STOP_INTERRUPT               UCSTPIE
#define eI2C_START_INTERRUPT              UCSTTIE
#define eI2C_TRANSMIT_INTERRUPT0          UCTXIE0
#define eI2C_TRANSMIT_INTERRUPT1          UCTXIE1
#define eI2C_TRANSMIT_INTERRUPT2          UCTXIE2
#define eI2C_TRANSMIT_INTERRUPT3          UCTXIE3
#define eI2C_RECEIVE_INTERRUPT0           UCRXIE0
#define eI2C_RECEIVE_INTERRUPT1           UCRXIE1
#define eI2C_RECEIVE_INTERRUPT2           UCRXIE2
#define eI2C_RECEIVE_INTERRUPT3           UCRXIE3
#define eI2C_BIT9_POSITION_INTERRUPT		 UCBIT9IE
#define eI2C_CLOCK_LOW_TIMEOUT_INTERRUPT	 UCCLTOIE
#define eI2C_BYTE_COUNTER_INTERRUPT	 	 UCBCNTIE

//*****************************************************************************
//
//The following are values that can be passed to the eI2C_masterInit() API
//as the dataRate parameter.
//
//*****************************************************************************
#define eI2C_SET_DATA_RATE_400KBPS             400000
#define eI2C_SET_DATA_RATE_100KBPS             100000

//*****************************************************************************
//
//The following are values that is returned by the eI2C_isBusy()  and
//eI2C_isBusBusy() API
//
//*****************************************************************************
#define eI2C_BUS_BUSY     UCBBUSY
#define eI2C_BUS_NOT_BUSY 0x00

//*****************************************************************************
//
//The following are values that is passed to the slaveOwnAddressEnable
//parameter of the eI2C_slaveInit() API
//
//*****************************************************************************
#define eI2C_OWN_ADDRESS_DISABLE	0x00
#define eI2C_OWN_ADDRESS_ENABLE	UCOAEN

//*****************************************************************************
//
//The following are values that is passed to the autoSTOPGeneration
//parameter of the eI2C_masterInit() API
//
//*****************************************************************************
#define eI2C_NO_AUTO_STOP									UCASTP_0
#define eI2C_SET_BYTECOUNT_THRESHOLD_FLAG					UCASTP_1
#define eI2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD	UCASTP_2

//*****************************************************************************
//
//The following are values that is passed to the selectEnvironment
//parameter of the eI2C_selectMasterEnvironmentSelect() API
//
//*****************************************************************************
#define eI2C_SINGLE_MASTER_ENVIRONMENT		0x00
#define eI2C_MULTI_MASTER_ENVIRONMENT		UCMM

//*****************************************************************************
//
//The following are values that is passed to eI2C_slaveInit() API as
// slaveAddressOffset parameter
//
//*****************************************************************************
#define eI2C_OWN_ADDRESS_OFFSET0 0x00
#define eI2C_OWN_ADDRESS_OFFSET1 0x02
#define eI2C_OWN_ADDRESS_OFFSET2 0x04
#define eI2C_OWN_ADDRESS_OFFSET3 0x06

//*****************************************************************************
//
//The following are values that can be passed to the eI2C_setMode() API
//as the mode parameter.
//
//*****************************************************************************
#define eI2C_TRANSMIT_MODE       UCTR
#define eI2C_RECEIVE_MODE        0x00

//*****************************************************************************
//
//The following are values that can be returned by the eI2C_masterIsSTOPSent()
//API
//
//*****************************************************************************

#define eI2C_SENDING_STOP		UCTXSTP
#define eI2C_STOP_SEND_COMPLETE	0x00

//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************
extern void eI2C_masterInit (unsigned int baseAddress,
	    unsigned char selectClockSource,
	    unsigned long i2cClk,
	    unsigned long dataRate,
	    unsigned char byteCounterThreshold,
	    unsigned char autoSTOPGeneration
	    );
extern void eI2C_slaveInit (unsigned int baseAddress,
	    unsigned char slaveAddress,
	    unsigned char slaveAddressOffset,
	    unsigned long slaveOwnAddressEnable
	    );
extern void eI2C_enable (unsigned int baseAddress);
extern void eI2C_disable (unsigned int baseAddress);
extern void eI2C_setSlaveAddress (unsigned int baseAddress,
    unsigned char slaveAddress
    );
extern void eI2C_setMode (unsigned int baseAddress,
    unsigned char mode
    );
extern void eI2C_slaveDataPut (unsigned int baseAddress,
    unsigned char transmitData
    );
extern unsigned char eI2C_slaveDataGet (unsigned int baseAddress);
extern unsigned char eI2C_isBusBusy (unsigned int baseAddress);
extern unsigned char eI2C_isBusy (unsigned int baseAddress);
extern void eI2C_enableInterrupt (unsigned int baseAddress,
    unsigned int mask
    );
extern void eI2C_disableInterrupt (unsigned int baseAddress,
    unsigned int mask
    );
extern void eI2C_clearInterruptFlag (unsigned int baseAddress,
    unsigned int mask
    );
extern unsigned char eI2C_getInterruptStatus (unsigned int baseAddress,
    unsigned int mask
    );
extern void eI2C_masterSendSingleByte (unsigned int baseAddress,
    unsigned char txData
    );
extern void eI2C_masterMultiByteSendStart (unsigned int baseAddress,
    unsigned char txData
    );
extern void eI2C_masterMultiByteSendNext (unsigned int baseAddress,
    unsigned char txData
    );
extern void eI2C_masterMultiByteSendFinish (unsigned int baseAddress,
    unsigned char txData
    );
extern void eI2C_masterMultiByteSendStop (unsigned int baseAddress);
extern void eI2C_masterReceiveStart (unsigned int baseAddress);
extern unsigned char eI2C_masterMultiByteReceiveNext (unsigned int baseAddress);
extern unsigned char eI2C_masterMultiByteReceiveFinish (
    unsigned int baseAddress
    );
extern void eI2C_masterMultiByteReceiveStop (unsigned int baseAddress);
extern unsigned char eI2C_masterSingleReceive (unsigned int baseAddress);
extern unsigned long eI2C_getReceiveBufferAddressForDMA (
    unsigned int baseAddress
    );
extern unsigned long eI2C_getTransmitBufferAddressForDMA (
    unsigned int baseAddress
    );
extern unsigned char eI2C_masterIsSTOPSent (unsigned int baseAddress);
extern void eI2C_masterSendStart (unsigned int baseAddress);
extern void eI2C_enableMultiMasterMode(unsigned int baseAddress);
extern void eI2C_disableMultiMasterMode(unsigned int baseAddress);
#endif
