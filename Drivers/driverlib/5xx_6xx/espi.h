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
#ifndef __MSP430WARE_EUSCI_SPI_H__
#define __MSP430WARE_EUSCI_SPI_H__

//*****************************************************************************
//
//The following are the defines to include the required modules for this
//peripheral in msp430xgeneric.h file
//
//*****************************************************************************
#define __MSP430_HAS_EUSCI_Ax__

//*****************************************************************************
//
//The following are values that can be passed to the SPI_masterInit() API
//as the selectClockSource parameter.
//
//*****************************************************************************
#define eSPI_CLOCKSOURCE_ACLK    UCSSEL__ACLK
#define eSPI_CLOCKSOURCE_SMCLK   UCSSEL__SMCLK

//*****************************************************************************
//
//The following are values that can be passed to the eSPI_masterInit() ,
//eSPI_slaveInit() API as the msbFirst parameter.
//
//*****************************************************************************
#define eSPI_MSB_FIRST    UCMSB
#define eSPI_LSB_FIRST    0x00

//*****************************************************************************
//
//The following are values that can be returned by the eSPI_isBusy() API
//
//*****************************************************************************
#define eSPI_BUSY        UCBUSY
#define eSPI_NOT_BUSY    0x00

//*****************************************************************************
//
//The following are values that can be passed to the eSPI_masterInit() ,
//eSPI_slaveInit() API as the clockPhase parameter.
//
//*****************************************************************************
#define eSPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT    0x00
#define eSPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT    UCCKPH

//*****************************************************************************
//
//The following are values that can be passed to the eSPI_masterInit() ,
//as the spiMode parameter.
//
//*****************************************************************************
#define eSPI_3PIN                      UCMODE_0
#define eSPI_4PIN_UCxSTE_ACTIVE_HIGH   UCMODE_1
#define eSPI_4PIN_UCxSTE_ACTIVE_LOW    UCMODE_2

//*****************************************************************************
//
//The following are values that can be passed to the eSPI_masterInit() ,
//eSPI_slaveInit() API as the clockPolarity parameter.
//
//*****************************************************************************
#define eSPI_CLOCKPOLARITY_INACTIVITY_HIGH    UCCKPL
#define eSPI_CLOCKPOLARITY_INACTIVITY_LOW     0x00

//*****************************************************************************
//
//The following are values that can be passed to the eSPI_enableInterrupt() ,
//eSPI_disableInterrupt(), eSPI_getInterruptStatus(),  API as the mask parameter.
//
//*****************************************************************************
#define eSPI_TRANSMIT_INTERRUPT    UCTXIE
#define eSPI_RECEIVE_INTERRUPT     UCRXIE

//*****************************************************************************
//
//The following are values that can be passed to the 
//eSPI_select4PinFunctionality() API as the select4PinFunctionality parameter.
//
//*****************************************************************************
#define eSPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE			UCSTEM
#define eSPI_PREVENT_CONFLICTS_WITH_OTHER_MASTERS	0x00

//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************
extern unsigned short eSPI_masterInit (unsigned int baseAddress,
    unsigned char selectClockSource,
    unsigned long clockSourceFrequency,
    unsigned long desiredSpiClock,
    unsigned int msbFirst,
    unsigned int clockPhase,
    unsigned int clockPolarity,
    unsigned char spiMode
    );
extern void eSPI_select4PinFunctionality (unsigned int baseAddress,
    unsigned char select4PinFunctionality
    );
extern void eSPI_masterChangeClock (unsigned int baseAddress,
    unsigned long clockSourceFrequency,
    unsigned long desiredSpiClock
    );

extern unsigned short eSPI_slaveInit (unsigned int baseAddress,
    unsigned int msbFirst,
    unsigned int clockPhase,
    unsigned int clockPolarity
    );
extern void eSPI_changeClockPhasePolarity (unsigned int baseAddress,
    unsigned char clockPhase,
    unsigned char clockPolarity
    );
extern void eSPI_transmitData ( unsigned int baseAddress,
    unsigned char transmitData
    );

extern unsigned char eSPI_receiveData (unsigned int baseAddress);
extern void eSPI_enableInterrupt (unsigned int baseAddress,
    unsigned char mask
    );
extern void eSPI_disableInterrupt (unsigned int baseAddress,
    unsigned char mask
    );
extern unsigned char eSPI_getInterruptStatus (unsigned int baseAddress,
    unsigned char mask
    );
extern void eSPI_enable (unsigned int baseAddress);
extern void eSPI_disable (unsigned int baseAddress);
extern unsigned long eSPI_getReceiveBufferAddressForDMA
    (unsigned int baseAddress);
extern unsigned long eSPI_getTransmitBufferAddressForDMA
    (unsigned int baseAddress);
extern unsigned char eSPI_isBusy (unsigned int baseAddress);
extern void eSPI_clearInterruptFlag (unsigned int baseAddress,
    unsigned char mask
    );
#endif

