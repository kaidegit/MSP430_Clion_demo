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
#ifndef __MSP430WARE_EUART_H__
#define __MSP430WARE_EUART_H__

//*****************************************************************************
//
//The following are the defines to include the required modules for this
//peripheral in msp430xgeneric.h file
//
//*****************************************************************************
#define __MSP430_HAS_EUSCI_Ax__

//*****************************************************************************
//
//The following are values that can be passed to the eUART_init() API
//as the parity parameter.
//
//*****************************************************************************
#define eUART_NO_PARITY   0x00
#define eUART_ODD_PARITY  0x01
#define eUART_EVEN_PARITY 0x02

//*****************************************************************************
//
//The following are values that can be passed to the eUART_init() API
//as the selectClockSource parameter.
//
//*****************************************************************************
#define eUART_CLOCKSOURCE_ACLK    UCSSEL__ACLK
#define eUART_CLOCKSOURCE_SMCLK   UCSSEL__SMCLK

//*****************************************************************************

//*****************************************************************************
//
//The following are values that can be passed to the eUART_init() API
//as the numberofStopBits parameter.
//
//*****************************************************************************
#define eUART_ONE_STOP_BIT    0x00
#define eUART_TWO_STOP_BITS   UCSPB


//*****************************************************************************
//
//The following are values that can be passed to the eUART_init() API
//as the msborLsbFirst parameter.
//
//*****************************************************************************
#define eUART_MSB_FIRST    UCMSB
#define eUART_LSB_FIRST    0x00

//*****************************************************************************
//
//The following are values that can be passed to the eUART_getInterruptStatus(),
//as the mask parameter.
//
//*****************************************************************************
#define eUART_RECEIVE_INTERRUPT_FLAG            UCRXIFG
#define eUART_TRANSMIT_INTERRUPT_FLAG           UCTXIFG

//*****************************************************************************
//
//The following are values that can be passed to the eUART_enableInterrupt(),
//eUART_disableInterrupt() API as the mask parameter.
//
//*****************************************************************************
#define eUART_RECEIVE_INTERRUPT                  UCRXIE
#define eUART_TRANSMIT_INTERRUPT                 UCTXIE
#define eUART_RECEIVE_ERRONEOUSCHAR_INTERRUPT    UCRXEIE
#define eUART_BREAKCHAR_INTERRUPT                UCBRKIE

//*****************************************************************************
//
//The following are values that can be passed to the eUART_selectDeglitchTime()
//API as the deglitchTime parameter.
//
//*****************************************************************************
#define eUART_DEGLITCH_TIME_2ns		0x00
#define eUART_DEGLITCH_TIME_50ns	UCGLIT0
#define eUART_DEGLITCH_TIME_100ns	UCGLIT1
#define eUART_DEGLITCH_TIME_200ns	(UCGLIT0 + UCGLIT1)

//*****************************************************************************
//
//The following are values that can be passed to the eUART_queryStatusFlags()
//API as the mask parameter.
//
//*****************************************************************************
#define eUART_LISTEN_ENABLE      UCLISTEN
#define eUART_FRAMING_ERROR      UCFE
#define eUART_OVERRUN_ERROR      UCOE
#define eUART_PARITY_ERROR       UCPE
#define eUARTBREAK_DETECT        UCBRK
#define eUART_RECEIVE_ERROR      UCRXERR
#define eUART_ADDRESS_RECEIVED   UCADDR
#define eUART_IDLELINE           UCIDLE
#define eUART_BUSY               UCBUSY

//*****************************************************************************
//
//The following are values that can be passed to the eUART_init()
//API as the mode parameter.
//
//*****************************************************************************
#define eUART_MODE                              UCMODE_0
#define eUART_IDLE_LINE_MULTI_PROCESSOR_MODE    UCMODE_1
#define eUART_ADDRESS_BIT_MULTI_PROCESSOR_MODE  UCMODE_2
#define eUART_AUTOMATIC_BAUDRATE_DETECTION_MODE UCMODE_3

//*****************************************************************************
//
//The following are values that can be passed to the eUART_init()
//API as the overSampling parameter.
//
//*****************************************************************************
#define eUART_OVERSAMPLING_BAUDRATE_GENERATION     0x01
#define eUART_LOW_FREQUENCY_BAUDRATE_GENERATION    0x00


//*****************************************************************************
//
//The following are values are the sync characters possible
//
//*****************************************************************************
#define DEFAULT_SYNC 0x00
#define eUART_AUTOMATICBAUDRATE_SYNC 0x55

//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************
extern
unsigned short eUART_init ( unsigned int baseAddress,
    unsigned char selectClockSource,
    unsigned long clockSourceFrequency,
    unsigned long desiredUartBaudRate,
    unsigned char parity,
    unsigned char msborLsbFirst,
    unsigned char numberofStopBits,
    unsigned char uartMode,
    unsigned short overSampling
    );
extern
unsigned short eUART_initAdvance ( unsigned int baseAddress,
    unsigned char selectClockSource,
    unsigned int clockPrescalar,
    unsigned char firstModReg,
    unsigned char secondModReg,
    unsigned char parity,
    unsigned char msborLsbFirst,
    unsigned char numberofStopBits,
    unsigned char uartMode,
    unsigned short overSampling
    );
extern
void eUART_transmitData ( unsigned int baseAddress,
    unsigned char transmitData
    );
extern
unsigned char eUART_receiveData (unsigned int baseAddress);
extern
void eUART_enableInterrupt (unsigned int baseAddress,
    unsigned char mask
    );
extern
void eUART_disableInterrupt (unsigned int baseAddress,
    unsigned char mask
    );
extern
unsigned char eUART_getInterruptStatus (unsigned int baseAddress,
    unsigned char mask
    );
extern
void eUART_clearInterruptFlag (unsigned int baseAddress,
    unsigned char mask
    );
extern
void eUART_enable (unsigned int baseAddress);
extern
void eUART_disable (unsigned int baseAddress);
extern
unsigned char eUART_queryStatusFlags (unsigned int baseAddress,
    unsigned char mask);
extern
void eUART_setDormant (unsigned int baseAddress);
extern
void eUART_resetDormant (unsigned int baseAddress);
extern
void eUART_transmitAddress (unsigned int baseAddress,
    unsigned char transmitAddress);
extern
void eUART_transmitBreak (unsigned int baseAddress);
extern
unsigned long eUART_getReceiveBufferAddressForDMA (unsigned int baseAddress);
extern
unsigned long eUART_getTransmitBufferAddressForDMA (unsigned int baseAddress);
extern
void eUART_selectDeglitchTime(unsigned int baseAddress,
			unsigned long deglitchTime
			);
#endif
