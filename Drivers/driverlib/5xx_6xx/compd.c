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
//compd.c - Driver for the COMPARATOR_D Module.
//
//*****************************************************************************
#include "driverlib/5xx_6xx/compd.h"
#include "inc/hw_types.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "deprecated/CCS/msp430xgeneric.h"
#endif
#include "driverlib/5xx_6xx/debug.h"

//*****************************************************************************
//
//! Initializes the Comparator Module.
//!
//! \param baseAddress is the base address of the Comparator module.
//! \param positiveTerminalInput selects the input to the positive terminal.
//!        Valid values are
//!        \b COMPD_INPUT0 [Default]
//!        \b COMPD_INPUT1
//!        \b COMPD_INPUT2
//!        \b COMPD_INPUT3
//!        \b COMPD_INPUT4
//!        \b COMPD_INPUT5
//!        \b COMPD_INPUT6
//!        \b COMPD_INPUT7
//!        \b COMPD_INPUT8
//!        \b COMPD_INPUT9
//!        \b COMPD_INPUT10
//!        \b COMPD_INPUT11
//!        \b COMPD_INPUT12
//!        \b COMPD_INPUT13
//!        \b COMPD_INPUT14
//!        \b COMPD_INPUT15
//!        \b COMPD_VREF
//!        Modified bits are \b CDIPSEL and \b CDIPEN of \b CDCTL0 register,
//!        \b CDRSEL of \b CDCTL2 register, and CDPDx of \b CDCTL3 register.
//! \param negativeTerminalInput selects the input to the negative terminal.
//!        Valid values are
//!        \b COMPD_INPUT0 [Default]
//!        \b COMPD_INPUT1
//!        \b COMPD_INPUT2
//!        \b COMPD_INPUT3
//!        \b COMPD_INPUT4
//!        \b COMPD_INPUT5
//!        \b COMPD_INPUT6
//!        \b COMPD_INPUT7
//!        \b COMPD_INPUT8
//!        \b COMPD_INPUT9
//!        \b COMPD_INPUT10
//!        \b COMPD_INPUT11
//!        \b COMPD_INPUT12
//!        \b COMPD_INPUT13
//!        \b COMPD_INPUT14
//!        \b COMPD_INPUT15
//!        \b COMPD_VREF
//!        Modified bits are \b CDIMSEL and \b CDIMEN of \b CDCTL0 register,
//!        \b CDRSEL of \b CDCTL2 register, and CDPDx of \b CDCTL3 register.
//! \param outputFilterEnableAndDelayLevel controls the output filter delay
//!       state, which is either off or enabled with a specified delay level.
//!        Valid values are
//!        \b COMPD_FILTEROUTPUT_OFF [Default]
//!        \b COMPD_FILTEROUTPUT_DLYLVL1
//!        \b COMPD_FILTEROUTPUT_DLYLVL2
//!        \b COMPD_FILTEROUTPUT_DLYLVL3
//!        \b COMPD_FILTEROUTPUT_DLYLVL4
//!        This parameter is device specific and delay levels should be found in
//!        the device's datasheet.
//!        Modified bits are \b CDF and \b CDFDLY of \b CDCTL1 register.
//! \param invertedOutput controls if the output will be inverted or not
//!        Valid values are
//!        \b COMPD_NORMALOUTPUTPOLARITY - indicates the output should be normal.
//!             [Default]
//!        \b COMPD_INVERTEDOUTPUTPOLARITY -  the output should be inverted.
//!        Modified bits are \b CDOUTPOL of \b CDCTL1 register.
//!
//! Upon successful initialization of the Comparator module, this function will
//! have reset all necessary register bits and set the given options in the
//! registers. To actually use the comparator module, the COMPD_enable()
//! function must be explicitly called before use.
//! If a Reference Voltage is set to a terminal, the Voltage should be set
//! using the setReferenceVoltage() function.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the initialization process.
//
//*****************************************************************************
unsigned short COMPD_init (unsigned int baseAddress,
    unsigned char positiveTerminalInput,
    unsigned char negativeTerminalInput,
    unsigned char outputFilterEnableAndDelayLevel,
    unsigned short invertedOutputPolarity)
{
    ASSERT(positiveTerminalInput <= COMPD_VREF);
    ASSERT(negativeTerminalInput <= COMPD_VREF);
    ASSERT(positiveTerminalInput != negativeTerminalInput);
    ASSERT(outputFilterEnableAndDelayLevel <= COMPD_FILTEROUTPUT_DLYLVL4);

    unsigned char retVal = STATUS_SUCCESS;

    //Reset COMPD Control 1 & Interrupt Registers for initialization (OFS_CDCTL3
    //is not reset because it controls the input buffers of the analog signals
    //and may cause parasitic effects if an analog signal is still attached and
    //the buffer is re-enabled
    HWREG(baseAddress + OFS_CDCTL0) &= 0x0000;
    HWREG(baseAddress + OFS_CDINT)  &= 0x0000;

    //Set the Positive Terminal
    if (COMPD_VREF != positiveTerminalInput){
        //Enable Positive Terminal Input Mux and Set it to the appropriate input
        HWREG(baseAddress + OFS_CDCTL0) |= CDIPEN + positiveTerminalInput;

        //Disable the input buffer
        HWREG(baseAddress + OFS_CDCTL3) |= (1 << positiveTerminalInput);
    } else {
        //Reset and Set COMPD Control 2 Register
        HWREG(baseAddress + OFS_CDCTL2) = ~(CDRSEL); //Set Vref to go to (+)terminal
    }

    //Set the Negative Terminal
    if (COMPD_VREF != negativeTerminalInput){
        //Enable Negative Terminal Input Mux and Set it to the appropriate input
        HWREG(baseAddress + OFS_CDCTL0) |= CDIMEN + (negativeTerminalInput << 8);

        //Disable the input buffer
        HWREG(baseAddress + OFS_CDCTL3) |= (1 << negativeTerminalInput);
    } else {
        //Reset and Set COMPD Control 2 Register
        HWREG(baseAddress + OFS_CDCTL2) = CDRSEL; //Set Vref to go to (-) terminal
    }

    //Reset and Set COMPD Control 1 Register
    HWREG(baseAddress + OFS_CDCTL1) =
        + outputFilterEnableAndDelayLevel //Set the filter enable bit and delay
        + invertedOutputPolarity; //Set the polarity of the output

    return ( retVal) ;
}
//*****************************************************************************
//
//! Generates a Reference Voltage to the terminal selected during initialization.
//!
//! \param baseAddress is the base address of the Comparator module.
//! \param supplyVoltageReferenceBase decides the source and max amount of Voltage that
//!       can be used as a reference.
//!        Valid values are
//!        \b COMPD_REFERENCE_AMPLIFIER_DISABLED
//!        \b COMPD_VREFBASE1_5V
//!        \b COMPD_VREFBASE2_0V
//!        \b COMPD_VREFBASE2_5V
//!        Modified bits are \b CDREFL of \b CDCTL2 register.
//! \param upperLimitSupplyVoltageFractionOf32 is the numerator of the equation
//!       to generate the reference voltage for the upper limit reference voltage.
//!        Modified bits are \b CDREF1 of \b CDCTL2 register.
//! \param lowerLimitSupplyVoltageFractionOf32 is the numerator of the equation
//!       to generate the reference voltage for the lower limit reference voltage.
//!        Modified bits are \b CDREF0 of \b CDCTL2 register.
//!
//! Use this function to generate a voltage to serve as a reference to the
//! terminal selected at initialization. The voltage is determined by the
//! equation: Vbase * (Numerator / 32). If the upper and lower limit voltage
//! numerators are equal, then a static reference is defined, whereas they are
//! different then a hysteresis effect is generated.
//!
//! \return NONE
//
//*****************************************************************************
void COMPD_setReferenceVoltage (unsigned int baseAddress,
    unsigned int supplyVoltageReferenceBase,
    unsigned int lowerLimitSupplyVoltageFractionOf32,
    unsigned int upperLimitSupplyVoltageFractionOf32)
{
    ASSERT(supplyVoltageReferenceBase <= COMPD_VREFBASE2_5V)
    ASSERT(upperLimitSupplyVoltageFractionOf32 <= 32);
    ASSERT(lowerLimitSupplyVoltageFractionOf32 <= 32);
    ASSERT(upperLimitSupplyVoltageFractionOf32
        >= lowerLimitSupplyVoltageFractionOf32);

    HWREG(baseAddress + OFS_CDCTL1) &= ~(CDMRVS); //Set to VREF0

    //Reset COMPD Control 2 Bits (Except for CDRSEL which is set in Comp_Init() )
    HWREG(baseAddress + OFS_CDCTL2) &= CDRSEL;

    //Set Voltage Source (Vcc | Vref, resistor ladder or not)
    if (COMPD_VREFBASE_VCC == supplyVoltageReferenceBase){
        HWREG(baseAddress + OFS_CDCTL2) |= CDRS_1; //Vcc with resistor ladder
    } else if (lowerLimitSupplyVoltageFractionOf32 == 32){
        //If the lower limit is 32, then the upper limit has to be 32 due to the
        //assertion that upper must be >= to the lower limit. If the numerator is
        //equal to 32, then the equation would be 32/32 == 1, therefore no resistor
        //ladder is needed
        HWREG(baseAddress + OFS_CDCTL2) |= CDRS_3; //Vref, no resistor ladder
    } else {
        HWREG(baseAddress + OFS_CDCTL2) |= CDRS_2; //Vref with resistor ladder
    }

    //Set COMPD Control 2 Register
    HWREG(baseAddress + OFS_CDCTL2) |=
        supplyVoltageReferenceBase //Set Supply Voltage Base
        + (upperLimitSupplyVoltageFractionOf32 - 1) //Set Supply Voltage Num.
        + ((lowerLimitSupplyVoltageFractionOf32 - 1) << 8);
}

//*****************************************************************************
//
//! Enables selected Comparator interrupt sources.
//!
//! \param baseAddress is the base address of the Comparator module.
//! \param mask is the bit mask of the interrupt sources to be enabled.
//!        Mask value is the logical OR of any of the following
//!        \b COMPD_INTERRUPT_ENABLE - Output interrupt enable
//!        \b COMPD_INTERRUPT_ENABLE_INVERTED_POLARITY - Output interrupt enable inverted polarity
//!
//! Enables the indicated Comparator interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor. The default trigger for the non-inverted
//! interrupt is a rising edge of the output, this can be changed with the
//! interruptSetEdgeDirection() function.
//!
//! \return NONE
//
//*****************************************************************************
void COMPD_enableInterrupt (unsigned int baseAddress,
    unsigned int interruptMask)
{
    HWREG(baseAddress + OFS_CDINT) &= ~(interruptMask);
    //Set the edge direction as default direction
    HWREG(baseAddress + OFS_CDCTL1) &= ~(CDIES);
    //Set the Interrupt enable bit
    HWREG(baseAddress + OFS_CDINT) |= interruptMask;
}

//*****************************************************************************
//
//! Disables selected Comparator interrupt sources.
//!
//! \param baseAddress is the base address of the Comparator module.
//! \param mask is the bit mask of the interrupt sources to be disabled.
//!        Mask value is the logical OR of any of the following
//!        \b COMPD_INTERRUPT_ENABLE - Output interrupt enable
//!        \b COMPD_INTERRUPT_ENABLE_INVERTED_POLARITY - Output interrupt enable inverted polarity
//!
//! Disables the indicated Comparator interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! \return NONE
//
//*****************************************************************************
void COMPD_disableInterrupt (unsigned int baseAddress,
    unsigned int interruptMask)
{
    HWREG(baseAddress + OFS_CDINT) &= ~(interruptMask);
}

//*****************************************************************************
//
//! Clears Comparator interrupt flags.
//!
//! \param baseAddress is the base address of the Comparator module.
//! \param mask is a bit mask of the interrupt sources to be cleared.
//!        Mask value is the logical OR of any of the following
//!        \b COMPD_INTERRUPT_FLAG - Output interrupt flag
//!        \b COMPD_INTERRUPT_FLAG_INVERTED_POLARITY - Output interrupt flag inverted polarity
//!
//! The Comparator interrupt source is cleared, so that it no longer asserts.
//! The highest interrupt flag is automatically cleared when an interrupt vector
//! generator is used.
//!
//! \return NONE
//
//*****************************************************************************
void COMPD_clearInterrupt (unsigned int baseAddress,
    unsigned int interruptFlagMask)
{
    HWREG(baseAddress + OFS_CDINT) &= ~(interruptFlagMask);
}

//*****************************************************************************
//
//! Gets the current Comparator interrupt status.
//!
//! \param baseAddress is the base address of the Comparator module.
//! \param mask is the masked interrupt flag status to be returned.
//!        Mask value is the logical OR of any of the following
//!        \b COMPD_INTERRUPT_FLAG - Output interrupt flag
//!        \b COMPD_INTERRUPT_FLAG_INVERTED_POLARITY - Output interrupt flag inverted polarity
//!
//! This returns the interrupt status for the Comparator module based on which
//! flag is passed.
//!
//! \return The current interrupt flag status for the corresponding mask.
//
//*****************************************************************************
unsigned char COMPD_getInterruptStatus (unsigned int baseAddress,
    unsigned int interruptFlagMask)
{
    return ( HWREG(baseAddress + OFS_CDINT) & interruptFlagMask );
}

//*****************************************************************************
//
//! Explicitly sets the edge direction that would trigger an interrupt.
//!
//! \param baseAddress is the base address of the Comparator module.
//! \param edgeDirection determines which direction the edge would have to go to
//!       generate an interrupt based on the non-inverted interrupt flag.
//!        Valid values are
//!        \b COMPD_FALLINGEDGE - sets the bit to generate an interrupt when the
//!             output of the comparator falls from HIGH to LOW if the normal
//!             interrupt bit is set(and LOW to HIGH if the inverted interrupt
//!             enable bit is set). [Default]
//!        \b COMPD_RISINGEDGE - sets the bit to generate an interrupt when the
//!             output of the comparator rises from LOW to HIGH if the normal
//!             interrupt bit is set(and HIGH to LOW if the inverted interrupt
//!             enable bit is set).
//!        Modified bits are \b CDIES of \b CDCTL1 register.
//!
//! This function will set which direction the output will have to go, whether
//! rising or falling, to generate an interrupt based on a non-inverted
//! interrupt.
//!
//! \return NONE
//
//*****************************************************************************
void COMPD_interruptSetEdgeDirection (unsigned int baseAddress,
    unsigned short edgeDirection)
{
    ASSERT(edgeDirection <= COMPD_RISINGEDGE);

    //Set the edge direction that will trigger an interrupt
    if (COMPD_RISINGEDGE == edgeDirection){
        HWREG(baseAddress + OFS_CDCTL1) |= CDIES;
    } else if (COMPD_FALLINGEDGE == edgeDirection){
        HWREG(baseAddress + OFS_CDCTL1) &= ~(CDIES);
    }
}

//*****************************************************************************
//
//! Toggles the edge direction that would trigger an interrupt.
//!
//! \param baseAddress is the base address of the Comparator module.
//!
//! This function will toggle which direction the output will have to go,
//! whether rising or falling, to generate an interrupt based on a non-inverted
//! interrupt. If the direction was rising, it is now falling, if it was
//! falling, it is now rising.
//!
//! Modified bits are \b CDIES of \b CDCTL1 register.
//!
//! \return NONE
//
//*****************************************************************************
void COMPD_interruptToggleEdgeDirection (unsigned int baseAddress)
{
    HWREG(baseAddress + OFS_CDCTL1) ^= CDIES;
}

//*****************************************************************************
//
//! Turns on the Comparator module.
//!
//! \param baseAddress is the base address of the Comparator module.
//!
//! This function sets the bit that enables the operation of the
//! Comparator module.
//!
//! \return NONE
//
//*****************************************************************************
void COMPD_enable (unsigned int baseAddress)
{
    HWREG(baseAddress + OFS_CDCTL1) |= CDON;
}

//*****************************************************************************
//
//! Turns off the Comparator module.
//!
//! \param baseAddress is the base address of the Comparator module.
//!
//! This function clears the CDON bit disabling the operation of the Comparator
//! module, saving from excess power consumption.
//!
//! Modified bits are \b CDON of \b CDCTL1 register.
//! \return NONE
//
//*****************************************************************************
void COMPD_disable (unsigned int baseAddress)
{
    HWREG(baseAddress + OFS_CDCTL1) &= ~(CDON);
}

//*****************************************************************************
//
//! Shorts the two input pins chosen during initialization.
//!
//! \param baseAddress is the base address of the Comparator module.
//!
//! This function sets the bit that shorts the devices attached to the input
//! pins chosen from the initialization of the comparator.
//!
//! Modified bits are \b CDSHORT of \b CDCTL1 register.
//! \return NONE
//
//*****************************************************************************
void COMPD_shortInputs (unsigned int baseAddress)
{
    HWREG(baseAddress + OFS_CDCTL1) |= CDSHORT;
}

//*****************************************************************************
//
//! Disables the short of the two input pins chosen during initialization.
//!
//! \param baseAddress is the base address of the Comparator module.
//!
//! This function clears the bit that shorts the devices attached to the input
//! pins chosen from the initialization of the comparator.
//!
//! Modified bits are \b CDSHORT of \b CDCTL1 register.
//! \return NONE
//
//*****************************************************************************
void COMPD_unshortInputs (unsigned int baseAddress)
{
    HWREG(baseAddress + OFS_CDCTL1) &= ~(CDSHORT);
}

//*****************************************************************************
//
//! Disables the input buffer of the selected input port to effectively allow
//! for analog signals.
//!
//! \param baseAddress is the base address of the Comparator module.
//! \param inputPort is the port in which the input buffer will be disabled.
//!        Valid values are
//!        \b COMPD_INPUT0 [Default]
//!        \b COMPD_INPUT1
//!        \b COMPD_INPUT2
//!        \b COMPD_INPUT3
//!        \b COMPD_INPUT4
//!        \b COMPD_INPUT5
//!        \b COMPD_INPUT6
//!        \b COMPD_INPUT7
//!        \b COMPD_INPUT8
//!        \b COMPD_INPUT9
//!        \b COMPD_INPUT10
//!        \b COMPD_INPUT11
//!        \b COMPD_INPUT12
//!        \b COMPD_INPUT13
//!        \b COMPD_INPUT14
//!        \b COMPD_INPUT15
//!        \b COMPD_VREF
//!        Modified bits are \b CDPDx of \b CDCTL3 register.
//!
//! This function sets the bit to disable the buffer for the specified input
//! port to allow for analog signals from any of the comparator input pins. This
//! bit is automatically set when the input is initialized to be used with the
//! comparator module. This function should be used whenever an analog input is
//! connected to one of these pins to prevent parasitic voltage from causing
//! unexpected results.
//!
//! \return NONE
//
//*****************************************************************************
void COMPD_disableInputBuffer (unsigned int baseAddress,
    unsigned char inputPort)
{
    HWREG(baseAddress + OFS_CDCTL3) |= (1 << inputPort);
}

//*****************************************************************************
//
//! Enables the input buffer of the selected input port to allow for digital
//! signals.
//!
//! \param baseAddress is the base address of the Comparator module.
//! \param inputPort is the port in which the input buffer will be enabled.
//!        Valid values are
//!        \b COMPD_INPUT0 [Default]
//!        \b COMPD_INPUT1
//!        \b COMPD_INPUT2
//!        \b COMPD_INPUT3
//!        \b COMPD_INPUT4
//!        \b COMPD_INPUT5
//!        \b COMPD_INPUT6
//!        \b COMPD_INPUT7
//!        \b COMPD_INPUT8
//!        \b COMPD_INPUT9
//!        \b COMPD_INPUT10
//!        \b COMPD_INPUT11
//!        \b COMPD_INPUT12
//!        \b COMPD_INPUT13
//!        \b COMPD_INPUT14
//!        \b COMPD_INPUT15
//!        \b COMPD_VREF
//!        Modified bits are \b CDPDx of \b CDCTL3 register.
//!
//! This function clears the bit to enable the buffer for the specified input
//! port to allow for digital signals from any of the comparator input pins.
//! This should not be reset if there is an analog signal connected to the
//! specified input pin to prevent from unexpected results.
//!
//! \return NONE
//
//*****************************************************************************
void COMPD_enableInputBuffer (unsigned int baseAddress, unsigned char inputPort)
{
    HWREG(baseAddress + OFS_CDCTL3) &= ~(1 << inputPort);
}

//*****************************************************************************
//
//! Toggles the bit that swaps which terminals the inputs go to, while also
//! inverting the output of the comparator.
//!
//! \param baseAddress is the base address of the Comparator module.
//!
//! This function toggles the bit that controls which input goes to which
//! terminal. After initialization, this bit is set to 0, after toggling it once
//! the inputs are routed to the opposite terminal and the output is inverted.
//!
//! Modified bits are \b CDEX of \b CDCTL1 register.
//! \return NONE
//
//*****************************************************************************
void COMPD_IOSwap (unsigned int baseAddress)
{
    HWREG(baseAddress + OFS_CDCTL1) ^= CDEX; //Toggle CDEX bit
}

//*****************************************************************************
//
//! Returns the output value of the Comparator module.
//!
//! \param baseAddress is the base address of the Comparator module.
//!
//! Returns the output value of the Comparator module.
//!
//! \return COMPD_HIGH or COMPD_LOW as the output value of the Comparator module.
//
//*****************************************************************************
unsigned short COMPD_outputValue (unsigned int baseAddress)
{
    if ( HWREG(baseAddress + OFS_CDCTL1) & CDOUT){
        return ( COMPD_HIGH) ;
    } else {
        return ( COMPD_LOW) ;
    }
}

//*****************************************************************************
//
//Close the Doxygen group.
//! @}
//
//*****************************************************************************
