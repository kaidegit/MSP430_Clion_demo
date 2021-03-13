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
//frgpio.c - Driver for the DIgital I/O Module.
//
//*****************************************************************************
#include "inc/hw_types.h"
#include "driverlib/5xx_6xx/debug.h"
#include "driverlib/5xx_6xx/frgpio.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "deprecated/CCS/msp430xgeneric.h"
#endif
//*****************************************************************************
//
//! This function configures the selected Pin as output pin
//!
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
///!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! Modified registers are \b PxSEL and \b PxDIR.
//!
//! \return None
//
//*****************************************************************************
void FRGPIO_setAsOutputPin ( unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

     ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));

    switch (selectedPort){
        case FRGPIO_PORT_P1:
        	HWREGB(baseAddress + OFS_P1SEL0) &= ~selectedPins;
        	HWREGB(baseAddress + OFS_P1SEL1) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P1DIR) |= selectedPins;
            break;
        case FRGPIO_PORT_P2:
            HWREGB(baseAddress + OFS_P2SEL0) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P2SEL1) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P2DIR) |= selectedPins;
            break;
        case FRGPIO_PORT_PA:
        	HWREG(baseAddress + OFS_PASEL0) &= ~selectedPins;
        	HWREG(baseAddress + OFS_PASEL1) &= ~selectedPins;
            HWREG(baseAddress + OFS_PADIR) |= selectedPins;
            break;
    }
}

//*****************************************************************************
//
//! This function configures the selected Pin as input pin
//!
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
///!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! Modified registers are \b PxSEL, \b PxREN and \b PxDIR.
//! \return None
//
//*****************************************************************************
void FRGPIO_setAsInputPin (unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

    ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));

    switch (selectedPort){
        case FRGPIO_PORT_P1:
        	HWREGB(baseAddress + OFS_P1SEL0) &= ~selectedPins;
        	HWREGB(baseAddress + OFS_P1SEL1) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P1DIR) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P1REN) &= ~selectedPins;
            break;
        case FRGPIO_PORT_P2:
            HWREGB(baseAddress + OFS_P2SEL0) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P2SEL1) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P2DIR) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P2REN) &= ~selectedPins;
            break;
        case FRGPIO_PORT_PA:
        	HWREG(baseAddress + OFS_PASEL0) &= ~selectedPins;
        	HWREG(baseAddress + OFS_PASEL1) &= ~selectedPins;
            HWREG(baseAddress + OFS_PADIR) &= ~selectedPins;
            HWREG(baseAddress + OFS_PAREN) &= ~selectedPins;
            break;
    }
}

//*****************************************************************************
//
//! This function configures the peripheral module function in the output direction
//! for the selected pin for either primary, secondary or ternary module function modes
//!
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! \param mode is the specified mode that the pin should be configured for the module function.
//!             Valid values are:
//!             \b FRGPIO_PRIMARY_MODULE_FUNCTION
//!             \b FRGPIO_SECONDARY_MODULE_FUNCTION
//!             \b FRGPIO_TERNARY_MODULE_FUNCTION
///! Modified registers are \b PxSEL and \b PxDIR.
//!
//! \return None
//
//*****************************************************************************
void FRGPIO_setAsPeripheralModuleFunctionOutputPin ( unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins,
    unsigned char mode
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

    ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));

    switch (selectedPort){
        case FRGPIO_PORT_P1:
        {
        	switch (mode){
				case FRGPIO_PRIMARY_MODULE_FUNCTION:
		        	HWREGB(baseAddress + OFS_P1SEL0) |= selectedPins;
		        	HWREGB(baseAddress + OFS_P1SEL1) &= ~selectedPins;
		            HWREGB(baseAddress + OFS_P1DIR) |= selectedPins;
		            break;
				case FRGPIO_SECONDARY_MODULE_FUNCTION:
		        	HWREGB(baseAddress + OFS_P1SEL0) &= ~selectedPins;
		        	HWREGB(baseAddress + OFS_P1SEL1) |= selectedPins;
		            HWREGB(baseAddress + OFS_P1DIR) |= selectedPins;
		            break;
				case FRGPIO_TERNARY_MODULE_FUNCTION:
		        	HWREGB(baseAddress + OFS_P1SEL0) |= selectedPins;
		        	HWREGB(baseAddress + OFS_P1SEL1) |= selectedPins;
		            HWREGB(baseAddress + OFS_P1DIR) |= selectedPins;
        	}
            break;
        }
        case FRGPIO_PORT_P2:
        {
        	switch (mode){
				case FRGPIO_PRIMARY_MODULE_FUNCTION:
		        	HWREGB(baseAddress + OFS_P2SEL0) |= selectedPins;
		        	HWREGB(baseAddress + OFS_P2SEL1) &= ~selectedPins;
		            HWREGB(baseAddress + OFS_P2DIR) |= selectedPins;
		            break;
				case FRGPIO_SECONDARY_MODULE_FUNCTION:
		        	HWREGB(baseAddress + OFS_P2SEL0) &= ~selectedPins;
		        	HWREGB(baseAddress + OFS_P2SEL1) |= selectedPins;
		            HWREGB(baseAddress + OFS_P2DIR) |= selectedPins;
		            break;
				case FRGPIO_TERNARY_MODULE_FUNCTION:
		        	HWREGB(baseAddress + OFS_P2SEL0) |= selectedPins;
		        	HWREGB(baseAddress + OFS_P2SEL1) |= selectedPins;
		            HWREGB(baseAddress + OFS_P2DIR) |= selectedPins;
        	}
            break;
        }
        case FRGPIO_PORT_PA:
        {
        	switch (mode){
				case FRGPIO_PRIMARY_MODULE_FUNCTION:
		        	HWREG(baseAddress + OFS_PASEL0) |= selectedPins;
		        	HWREG(baseAddress + OFS_PASEL1) &= ~selectedPins;
		            HWREG(baseAddress + OFS_PADIR) |= selectedPins;
		            break;
				case FRGPIO_SECONDARY_MODULE_FUNCTION:
		        	HWREG(baseAddress + OFS_PASEL0) &= ~selectedPins;
		        	HWREG(baseAddress + OFS_PASEL1) |= selectedPins;
		            HWREG(baseAddress + OFS_PADIR) |= selectedPins;
		            break;
				case FRGPIO_TERNARY_MODULE_FUNCTION:
		        	HWREG(baseAddress + OFS_PASEL0) |= selectedPins;
		        	HWREG(baseAddress + OFS_PASEL1) |= selectedPins;
		            HWREG(baseAddress + OFS_PADIR) |= selectedPins;
        	}
            break;
        }
    }
}

//*****************************************************************************
//
//! This function configures the peripheral module function in the input direction
//! for the selected pin for either primary, secondary or ternary module function modes.
//!
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! \param mode is the specified mode that the pin should be configured for the module function.
//!             Valid values are:
//!             \b FRGPIO_PRIMARY_MODULE_FUNCTION
//!             \b FRGPIO_SECONDARY_MODULE_FUNCTION
//!             \b FRGPIO_TERNARY_MODULE_FUNCTION
//! Modified registers are \b PxSEL and \b PxDIR.
//!
//! \return None
//
//*****************************************************************************
void FRGPIO_setAsPeripheralModuleFunctionInputPin ( unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins,
    unsigned char mode
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

    ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));


    switch (selectedPort){
        case FRGPIO_PORT_P1:
        {
        	switch (mode){
				case FRGPIO_PRIMARY_MODULE_FUNCTION:
		        	HWREGB(baseAddress + OFS_P1SEL0) |= selectedPins;
		        	HWREGB(baseAddress + OFS_P1SEL1) &= ~selectedPins;
		            HWREGB(baseAddress + OFS_P1DIR) &= ~selectedPins;
		            break;
				case FRGPIO_SECONDARY_MODULE_FUNCTION:
		        	HWREGB(baseAddress + OFS_P1SEL0) &= ~selectedPins;
		        	HWREGB(baseAddress + OFS_P1SEL1) |= selectedPins;
		            HWREGB(baseAddress + OFS_P1DIR) &= ~selectedPins;
		            break;
				case FRGPIO_TERNARY_MODULE_FUNCTION:
		        	HWREGB(baseAddress + OFS_P1SEL0) |= selectedPins;
		        	HWREGB(baseAddress + OFS_P1SEL1) |= selectedPins;
		            HWREGB(baseAddress + OFS_P1DIR) &= ~selectedPins;
        	}
            break;
        }
        case FRGPIO_PORT_P2:
        {
        	switch (mode){
				case FRGPIO_PRIMARY_MODULE_FUNCTION:
		        	HWREGB(baseAddress + OFS_P2SEL0) |= selectedPins;
		        	HWREGB(baseAddress + OFS_P2SEL1) &= ~selectedPins;
		            HWREGB(baseAddress + OFS_P2DIR) &= ~selectedPins;
		            break;
				case FRGPIO_SECONDARY_MODULE_FUNCTION:
		        	HWREGB(baseAddress + OFS_P2SEL0) &= ~selectedPins;
		        	HWREGB(baseAddress + OFS_P2SEL1) |= selectedPins;
		            HWREGB(baseAddress + OFS_P2DIR) &= ~selectedPins;
		            break;
				case FRGPIO_TERNARY_MODULE_FUNCTION:
		        	HWREGB(baseAddress + OFS_P2SEL0) |= selectedPins;
		        	HWREGB(baseAddress + OFS_P2SEL1) |= selectedPins;
		            HWREGB(baseAddress + OFS_P2DIR) &= ~selectedPins;
        	}
            break;
        }
        case FRGPIO_PORT_PA:
        {
        	switch (mode){
				case FRGPIO_PRIMARY_MODULE_FUNCTION:
		        	HWREG(baseAddress + OFS_PASEL0) |= selectedPins;
		        	HWREG(baseAddress + OFS_PASEL1) &= ~selectedPins;
		            HWREG(baseAddress + OFS_PADIR) &= ~selectedPins;
		            break;
				case FRGPIO_SECONDARY_MODULE_FUNCTION:
		        	HWREG(baseAddress + OFS_PASEL0) &= ~selectedPins;
		        	HWREG(baseAddress + OFS_PASEL1) |= selectedPins;
		            HWREG(baseAddress + OFS_PADIR) &= ~selectedPins;
		            break;
				case FRGPIO_TERNARY_MODULE_FUNCTION:
		        	HWREG(baseAddress + OFS_PASEL0) |= selectedPins;
		        	HWREG(baseAddress + OFS_PASEL1) |= selectedPins;
		            HWREG(baseAddress + OFS_PADIR) &= ~selectedPins;
        	}
            break;
        }
    }
}

//*****************************************************************************
//
//! This function sets output HIGH on the selected Pin
//!
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! Modified registers are \b PxOUT.
//!
//! \return None
//
//*****************************************************************************
void FRGPIO_setOutputHighOnPin (  unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

    ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));

    switch (selectedPort){
        case FRGPIO_PORT_P1:
            HWREGB(baseAddress + OFS_P1OUT) |= selectedPins;
            break;
        case FRGPIO_PORT_P2:
            HWREGB(baseAddress + OFS_P2OUT) |= selectedPins;
            break;
        case FRGPIO_PORT_PA:
            HWREG(baseAddress + OFS_PAOUT) |= selectedPins;
            break;
    }
}

//*****************************************************************************
//
//! This function sets output LOW on the selected Pin
//!
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! Modified registers are \b PxOUT.
//!
//! \return None
//
//*****************************************************************************
void FRGPIO_setOutputLowOnPin (  unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

    ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));

    switch (selectedPort){
        case FRGPIO_PORT_P1:
            HWREGB(baseAddress + OFS_P1OUT) &= ~selectedPins;
            break;
        case FRGPIO_PORT_P2:
            HWREGB(baseAddress + OFS_P2OUT) &= ~selectedPins;
            break;
        case FRGPIO_PORT_PA:
            HWREG(baseAddress + OFS_PAOUT) &= ~selectedPins;
            break;
    }
}

//*****************************************************************************
//
//! This function toggles the output on the selected Pin
//!
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! Modified registers are \b PxOUT.
//!
//! \return None
//
//*****************************************************************************
void FRGPIO_toggleOutputOnPin (  unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

    ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));

    switch (selectedPort){
        case FRGPIO_PORT_P1:
            HWREGB(baseAddress + OFS_P1OUT) ^= selectedPins;
            break;
        case FRGPIO_PORT_P2:
            HWREGB(baseAddress + OFS_P2OUT) ^= selectedPins;
            break;
        case FRGPIO_PORT_PA:
            HWREG(baseAddress + OFS_PAOUT) ^= selectedPins;
            break;
    }
}

//*****************************************************************************
//
//! This function sets the selected Pin in input Mode with Pull Down resistor
//!
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! Modified registers are \b PxREN, \b PxOUT and \b PxDIR.
//!
//! \return None
//
//*****************************************************************************
void FRGPIO_setAsInputPinWithPullDownresistor (  unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

    ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));


    switch (selectedPort){
        case FRGPIO_PORT_P1:
            HWREGB(baseAddress + OFS_P1SEL0) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P1SEL1) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P1DIR) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P1REN) |= selectedPins;
            HWREGB(baseAddress + OFS_P1OUT) &= ~selectedPins;
            break;
        case FRGPIO_PORT_P2:
        	HWREGB(baseAddress + OFS_P2SEL0) &= ~selectedPins;
        	HWREGB(baseAddress + OFS_P2SEL1) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P2DIR) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P2REN) |= selectedPins;
            HWREGB(baseAddress + OFS_P2OUT) &= ~selectedPins;
            break;
        case FRGPIO_PORT_PA:
        	HWREG(baseAddress + OFS_PASEL0) &= ~selectedPins;
        	HWREG(baseAddress + OFS_PASEL1) &= ~selectedPins;
            HWREG(baseAddress + OFS_PADIR) &= ~selectedPins;
            HWREG(baseAddress + OFS_PAREN) |= selectedPins;
            HWREG(baseAddress + OFS_PAOUT) &= ~selectedPins;
            break;
    }
}

//*****************************************************************************
//
//! This function sets the selected Pin in input Mode with Pull Up resistor
//!
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! Modified registers are \b PxREN, \b PxOUT and \b PxDIR.
//!
//! \return None
//
//*****************************************************************************
void FRGPIO_setAsInputPinWithPullUpresistor (unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

    ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));
    switch (selectedPort){
        case FRGPIO_PORT_P1:
        	HWREGB(baseAddress + OFS_P1SEL0) &= ~selectedPins;
        	HWREGB(baseAddress + OFS_P1SEL1) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P1DIR) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P1REN) |= selectedPins;
            HWREGB(baseAddress + OFS_P1OUT) |= selectedPins;
            break;
        case FRGPIO_PORT_P2:
        	HWREGB(baseAddress + OFS_P2SEL0) &= ~selectedPins;
        	HWREGB(baseAddress + OFS_P2SEL1) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P2DIR) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P2REN) |= selectedPins;
            HWREGB(baseAddress + OFS_P2OUT) |= selectedPins;
            break;
        case FRGPIO_PORT_PA:
        	HWREG(baseAddress + OFS_PASEL0) &= ~selectedPins;
        	HWREG(baseAddress + OFS_PASEL1) &= ~selectedPins;
            HWREG(baseAddress + OFS_PADIR) &= ~selectedPins;
            HWREG(baseAddress + OFS_PAREN) |= selectedPins;
            HWREG(baseAddress + OFS_PAOUT) |= selectedPins;
            break;
    }
}

//*****************************************************************************
//
//! This function gets the input value on the selected pin
//!
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! Modified registers are \b PxIN.
//!
//! \return Input value on Pin - \b FRGPIO_INPUT_PIN_HIGH,
//!                              \b FRGPIO_INPUT_PIN_LOW
//
//*****************************************************************************
unsigned short FRGPIO_getInputPinValue (unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

    ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));

    unsigned int inputPinValue = 0;

    switch (selectedPort){
        case FRGPIO_PORT_P1:
            inputPinValue = HWREGB(baseAddress + OFS_P1IN) & selectedPins;
            break;
        case FRGPIO_PORT_P2:
            inputPinValue = HWREGB(baseAddress + OFS_P2IN) & selectedPins;
            break;
        case FRGPIO_PORT_PA:
            inputPinValue = HWREG(baseAddress + OFS_PAIN) & selectedPins;
            break;
    }

    if (inputPinValue > 0){
        return ( FRGPIO_INPUT_PIN_HIGH) ;
    }
    return ( FRGPIO_INPUT_PIN_LOW) ;
}

//*****************************************************************************
//
//! This function enables the port interrupt on the selected pin.
//! Note: Not all ports have this capability.  Please refer to the device
//!   specific datasheet.
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! Modified registers are \b PxIE.
//!
//! \return None
//
//*****************************************************************************
void FRGPIO_enableInterrupt (unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

    ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));

    switch (selectedPort){
        case FRGPIO_PORT_P1:
	    HWREGB(baseAddress + OFS_P1IFG) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P1IE) |= selectedPins;
            break;
        case FRGPIO_PORT_P2:
	    HWREGB(baseAddress + OFS_P2IFG) &= ~selectedPins;
            HWREGB(baseAddress + OFS_P2IE) |= selectedPins;
            break;
        case FRGPIO_PORT_PA:
	    HWREG(baseAddress + OFS_PAIFG) &= ~selectedPins;
            HWREG(baseAddress + OFS_PAIE) |= selectedPins;
            break;
    }
}

//*****************************************************************************
//
//! This function disables the port interrupt on the selected pin.
//!     Note that only Port 1,2, A have this capability
//!
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! Modified registers are \b PxIE.
//!
//! \return None
//
//*****************************************************************************
void FRGPIO_disableInterrupt (unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

    ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));

    switch (selectedPort){
        case FRGPIO_PORT_P1:
            HWREGB(baseAddress + OFS_P1IE) &= ~selectedPins;
            break;
        case FRGPIO_PORT_P2:
            HWREGB(baseAddress + OFS_P2IE) &= ~selectedPins;
            break;
        case FRGPIO_PORT_PA:
            HWREG(baseAddress + OFS_PAIE) &= ~selectedPins;
            break;
    }
}

//*****************************************************************************
//
//! This function gets the interrupt status of the selected pin.
//!     Note that only Port 1,2, A have this capability
//!
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! Modified registers are \b PxIFG.
//!
//! \return Masked state of the interupt
//
//*****************************************************************************
unsigned int FRGPIO_getInterruptStatus (unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

    ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));

    unsigned char returnValue;

    switch (selectedPort){
        case FRGPIO_PORT_P1:
            returnValue = (HWREGB(baseAddress + OFS_P1IFG) & selectedPins);
            break;
        case FRGPIO_PORT_P2:
            returnValue = (HWREGB(baseAddress + OFS_P2IFG) & selectedPins);
            break;
        case FRGPIO_PORT_PA:
            returnValue = (HWREG(baseAddress + OFS_PAIFG) & selectedPins);
            break;
    }

    return ( returnValue) ;
}

//*****************************************************************************
//
//! This function clears the interrupt flag on the selected pin.
//!     Note that only Port 1,2, A have this capability
//!
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! Modified registers are \b PxIFG.
//!
//! \return None
//
//*****************************************************************************
void FRGPIO_clearInterruptFlag (unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

    ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));


    switch (selectedPort){
        case FRGPIO_PORT_P1:
            HWREGB(baseAddress + OFS_P1IFG) &= ~selectedPins;
            break;
        case FRGPIO_PORT_P2:
            HWREGB(baseAddress + OFS_P2IFG) &= ~selectedPins;
            break;
        case FRGPIO_PORT_PA:
            HWREG(baseAddress + OFS_PAIFG) &= ~selectedPins;
            break;
    }
}

//*****************************************************************************
//
//! This function selects on what edge the port interrupt flag should be set
//! for a transition
//!
//! \param baseAddress is the base address of the GPIO Port Register
//! \param selectedPort is the selected port.
//!             Valid values are \b FRGPIO_PORT_P1, \b FRGPIO_PORT_P2,
//!             \b FRGPIO_PORT_P3, \b FRGPIO_PORT_P4,\b FRGPIO_PORT_P5,
//!             \b FRGPIO_PORT_P6, \b FRGPIO_PORT_P7,\b FRGPIO_PORT_P8,
//!             \b FRGPIO_PORT_P9, \b FRGPIO_PORT_P10,
//!             \b FRGPIO_PORT_P11, \b FRGPIO_PORT_PA,
//!             \b FRGPIO_PORT_PB, \b FRGPIO_PORT_PC,
//!             \b FRGPIO_PORT_PD, \b FRGPIO_PORT_PE,
//!             \b FRGPIO_PORT_PF, \b FRGPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are \b FRGPIO_PIN0, \b FRGPIO_PIN1, \b FRGPIO_PIN2,
//!             \b FRGPIO_PIN3, \b FRGPIO_PIN4, \b FRGPIO_PIN5, \b FRGPIO_PIN6,
//!             \b FRGPIO_PIN7,\b FRGPIO_PIN8,\b FRGPIO_PIN9,\b FRGPIO_PIN10,
//!             \b FRGPIO_PIN11,\b FRGPIO_PIN12,\b FRGPIO_PIN13,\b FRGPIO_PIN14,
//!             \b FRGPIO_PIN15
//! \param edgeSelect specifies what transition sets the interrupt flag
//!             Valid values are
//!             \b FRGPIO_HIGH_TO_LOW_TRANSITION,
//!             \b FRGPIO_LOW_TO_HIGH_TRANSITION
//! Modified registers are \b PxIES.
//!
//! \return None
//
//*****************************************************************************
void FRGPIO_interruptEdgeSelect (unsigned int baseAddress,
    unsigned char selectedPort,
    unsigned int selectedPins,
    unsigned char edgeSelect
    )
{
    ASSERT((FRGPIO_PORT_P1 == selectedPort) || (FRGPIO_PORT_P2 == selectedPort) ||
        (FRGPIO_PORT_P3 == selectedPort) || (FRGPIO_PORT_P4 == selectedPort) ||
        (FRGPIO_PORT_P5 == selectedPort) || (FRGPIO_PORT_P6 == selectedPort) ||
        (FRGPIO_PORT_P7 == selectedPort) || (FRGPIO_PORT_P8 == selectedPort) ||
        (FRGPIO_PORT_P9 == selectedPort) || (FRGPIO_PORT_P10 == selectedPort) ||
        (FRGPIO_PORT_P11 == selectedPort) || (FRGPIO_PORT_PA == selectedPort) ||
        (FRGPIO_PORT_PB == selectedPort) || (FRGPIO_PORT_PC == selectedPort) ||
        (FRGPIO_PORT_PD == selectedPort) || (FRGPIO_PORT_PE == selectedPort) ||
        (FRGPIO_PORT_PF == selectedPort) || (FRGPIO_PORT_PJ == selectedPort)
        );

    ASSERT(0x00 != (selectedPins & (FRGPIO_PIN0 + FRGPIO_PIN1 + FRGPIO_PIN2 +
                                     FRGPIO_PIN3 + FRGPIO_PIN4 + FRGPIO_PIN5 +
                                     FRGPIO_PIN6 + FRGPIO_PIN7 + FRGPIO_PIN8 +
                                     FRGPIO_PIN9 + FRGPIO_PIN10 + FRGPIO_PIN11 +
                                     FRGPIO_PIN12 + FRGPIO_PIN13 + FRGPIO_PIN14 +
                                     FRGPIO_PIN15
             )));

    ASSERT((edgeSelect == FRGPIO_HIGH_TO_LOW_TRANSITION) ||
        (edgeSelect == FRGPIO_LOW_TO_HIGH_TRANSITION)
        );

    switch (selectedPort){
        case FRGPIO_PORT_P1:
            if (FRGPIO_LOW_TO_HIGH_TRANSITION == edgeSelect){
                HWREGB(baseAddress + OFS_P1IES) &= ~selectedPins;
            } else   {
                HWREGB(baseAddress + OFS_P1IES) |= selectedPins;
            }
            break;

        case FRGPIO_PORT_P2:
            if (FRGPIO_LOW_TO_HIGH_TRANSITION == edgeSelect){
                HWREGB(baseAddress + OFS_P2IES) &= ~selectedPins;
            } else  {
                HWREGB(baseAddress + OFS_P2IES) |= selectedPins;
            }
            break;

        case FRGPIO_PORT_PA:
            if (FRGPIO_LOW_TO_HIGH_TRANSITION == edgeSelect){
                HWREG(baseAddress + OFS_PAIES) &= ~selectedPins;
            } else  {
                HWREG(baseAddress + OFS_PAIES) |= selectedPins;
            }
            break;
    }
}

//*****************************************************************************
//
//Close the Doxygen group.
//! @}
//
//*****************************************************************************
