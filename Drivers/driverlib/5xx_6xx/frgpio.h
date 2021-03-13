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
#ifndef __MSP430WARE_FRGPIO_H__
#define __MSP430WARE_FRGPIO_H__
//*****************************************************************************
//
//The following are the defines to include the required modules for this
//peripheral in msp430xgeneric.h file
//
// The reason for only including Ports 1,2,A is that most of the others are the exact
//  same format with exceptions specific to your device.  It is up to the user
//  to make sure the ports are available by looking in the datasheet for their device.
//  These API's will work, however; ports that are not fully implemented will not work.
//
//*****************************************************************************
#define __MSP430_HAS_PORT1_R__        /* Definition to show that Module is available */
#define __MSP430_HAS_PORT2_R__        /* Definition to show that Module is available */
#define __MSP430_HAS_PORTA_R__        /* Definition to show that Module is available */

//*****************************************************************************
//
//The following are values that can be passed to the all GPIO module API
//as the selectedPort parameter.
//
// Note: Since these GPIOs have the same offsets, we can just combine them into sets.
//*****************************************************************************
#define FRGPIO_PORT_P1     (0x00)
#define FRGPIO_PORT_P2     (0x01)
#define FRGPIO_PORT_P3     (0x00)
#define FRGPIO_PORT_P4     (0x01)
#define FRGPIO_PORT_P5     (0x00)
#define FRGPIO_PORT_P6     (0x01)
#define FRGPIO_PORT_P7     (0x00)
#define FRGPIO_PORT_P8     (0x01)
#define FRGPIO_PORT_P9     (0x00)
#define FRGPIO_PORT_P10    (0x01)
#define FRGPIO_PORT_P11    (0x00)
#define FRGPIO_PORT_PA     (0x02)
#define FRGPIO_PORT_PB     (0x02)
#define FRGPIO_PORT_PC     (0x02)
#define FRGPIO_PORT_PD     (0x02)
#define FRGPIO_PORT_PE     (0x02)
#define FRGPIO_PORT_PF     (0x02)
#define FRGPIO_PORT_PJ     (0x02)

//*****************************************************************************
//
//The following are values that can be passed to the all GPIO module API
//as the selectedPin parameter.
//
//*****************************************************************************
#define FRGPIO_PIN0    (0x0001)
#define FRGPIO_PIN1    (0x0002)
#define FRGPIO_PIN2    (0x0004)
#define FRGPIO_PIN3    (0x0008)
#define FRGPIO_PIN4    (0x0010)
#define FRGPIO_PIN5    (0x0020)
#define FRGPIO_PIN6    (0x0040)
#define FRGPIO_PIN7    (0x0080)
#define FRGPIO_PIN8    (0x0100)
#define FRGPIO_PIN9    (0x0200)
#define FRGPIO_PIN10   (0x0400)
#define FRGPIO_PIN11   (0x0800)
#define FRGPIO_PIN12   (0x1000)
#define FRGPIO_PIN13   (0x2000)
#define FRGPIO_PIN14   (0x4000)
#define FRGPIO_PIN15   (0x8000)

//*****************************************************************************
//
//The following are values that may be returned by the FRGPIO_getInputPinValue()
//API.
//
//*****************************************************************************
#define FRGPIO_INPUT_PIN_HIGH (0x01)
#define FRGPIO_INPUT_PIN_LOW  (0x00)

//*****************************************************************************
//
//The following are values that can be passed to the FRGPIO_interruptEdgeSelect()
//API as the edgeSelect parameter.
//
//*****************************************************************************
#define FRGPIO_HIGH_TO_LOW_TRANSITION (0x01)
#define FRGPIO_LOW_TO_HIGH_TRANSITION (0x00)

//*****************************************************************************
//
//The following are values that can be passed to the
//  FRGPIO_setAsPeripheralModuleFunctionOutputPin() or FRGPIO_setAsPeripheralModuleFunctionInputPin() API
//  as the mode parameter.
//
//*****************************************************************************
#define FRGPIO_PRIMARY_MODULE_FUNCTION 		(0x01)
#define FRGPIO_SECONDARY_MODULE_FUNCTION 	(0x02)
#define FRGPIO_TERNARY_MODULE_FUNCTION 		(0x03)

//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************
extern void FRGPIO_setAsOutputPin (unsigned int baseAddress,
								unsigned char selectedPort,
								unsigned int selectedPin);

extern void FRGPIO_setAsInputPin (unsigned int baseAddress,
								unsigned char selectedPort,
								unsigned int selectedPin);

extern void FRGPIO_setOutputHighOnPin ( unsigned int baseAddress,
										unsigned char selectedPort,
										unsigned int selectedPin);

extern void FRGPIO_setOutputLowOnPin (unsigned int baseAddress,
										unsigned char selectedPort,
										unsigned int selectedPin);

extern void FRGPIO_toggleOutputOnPin (unsigned int baseAddress,
									unsigned char selectedPort,
									unsigned int selectedPin);

extern void FRGPIO_setAsInputPinWithPullDownresistor (unsigned int baseAddress,
													unsigned char selectedPort,
													unsigned int selectedPin);

extern void FRGPIO_setAsInputPinWithPullUpresistor (unsigned int baseAddress,
													unsigned char selectedPort,
													unsigned int selectedPin);

extern void FRGPIO_enableInterrupt (unsigned int baseAddress,
									unsigned char selectedPort,
									unsigned int selectedPins);

extern void FRGPIO_disableInterrupt (unsigned int baseAddress,
									unsigned char selectedPort,
									unsigned int selectedPins);

extern void FRGPIO_clearInterruptFlag (unsigned int baseAddress,
										unsigned char selectedPort,
										unsigned int selectedPins);

extern unsigned int FRGPIO_getInterruptStatus (unsigned int baseAddress,
												unsigned char selectedPort,
												unsigned int selectedPins);

extern void FRGPIO_interruptEdgeSelect (unsigned int baseAddress,
										unsigned char selectedPort,
										unsigned int selectedPins,
										unsigned char edgeSelect);

extern unsigned short FRGPIO_getInputPinValue (unsigned int baseAddress,
												unsigned char selectedPort,
												unsigned int selectedPin);


extern void FRGPIO_setAsPeripheralModuleFunctionOutputPin ( unsigned int baseAddress,
												unsigned char selectedPort,
												unsigned int selectedPin,
												unsigned char mode);

extern void FRGPIO_setAsPeripheralModuleFunctionInputPin ( unsigned int baseAddress,
												unsigned char selectedPort,
												unsigned int selectedPin,
												unsigned char mode);
#endif
