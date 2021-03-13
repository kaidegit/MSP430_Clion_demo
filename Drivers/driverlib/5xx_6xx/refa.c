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
//ref.c - Driver for the REF Module.
//
//*****************************************************************************
#include "inc/hw_types.h"
#include "driverlib/5xx_6xx/debug.h"
#include "driverlib/5xx_6xx/refa.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "deprecated/CCS/msp430xgeneric.h"
#endif


//*****************************************************************************
//
//! Sets the reference voltage for the voltage generator.
//!
//! \param baseAddress is the base address of the REF module.
//! \param referenceVoltageSelect is the desired voltage to generate for a
//!       reference voltage.
//!        Valid values are
//!        \b REFA_VREF1_2V [Default]
//!        \b REFA_VREF2_0V
//!        \b REFA_VREF2_5V
//!        Modified bits are \b REFVSEL of \b REFCTL0 register.
//!
//! This function sets the reference voltage generated by the voltage generator
//! to be used by other peripherals. This reference voltage will only be valid
//! while the REF module is in control.
//! Please note, if the REFA_isRefGenBusy() returns REFA_ BUSY, this function
//! will have no effect.
//!
//! \return NONE
//
//*****************************************************************************
void REFA_setReferenceVoltage (unsigned int baseAddress,
    unsigned char referenceVoltageSelect)
{
    ASSERT(referenceVoltageSelect <= REFA_VREF2_5V);

    HWREGB(baseAddress + OFS_REFCTL0_L) &= ~(REFVSEL_3);
    HWREGB(baseAddress + OFS_REFCTL0_L) |= referenceVoltageSelect;
}

//*****************************************************************************
//
//! Disables the internal temperature sensor to save power consumption.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to turn off the internal temperature sensor to save
//! on power consumption. The temperature sensor is enabled by default. Please
//! note, that giving ADC12 module control over the REF module, the state of the
//! temperature sensor is dependent on the controls of the ADC12 module.
//! Please note, if the REFA_isRefGenBusy() returns REFA_ BUSY, this function
//! will have no effect.
//!
//! Modified bits are \b REFTCOFF of \b REFCTL0 register.
//! \return NONE
//
//*****************************************************************************
void REFA_disableTempSensor (unsigned int baseAddress)
{
    HWREGB(baseAddress + OFS_REFCTL0_L) |= REFTCOFF;
}

//*****************************************************************************
//
//! Enables the internal temperature sensor.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to turn on the internal temperature sensor to use by
//! other peripherals. The temperature sensor is enabled by default.
//! Please note, if the REFA_isRefGenBusy() returns REFA_ BUSY, this function
//! will have no effect.
//!
//! Modified bits are \b REFTCOFF of \b REFCTL0 register.
//! \return NONE
//
//*****************************************************************************
void REFA_enableTempSensor (unsigned int baseAddress)
{
    HWREGB(baseAddress + OFS_REFCTL0_L) &= ~(REFTCOFF);
}

//*****************************************************************************
//
//! Outputs the reference voltage to an output pin.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to output the reference voltage being generated to an
//! output pin. Please note, the output pin is device specific. Please note,
//! that giving ADC12 module control over the REF module, the state of the
//! reference voltage as an output to a pin is dependent on the controls of the
//! ADC12 module.
//! Please note, if the REFA_isRefGenBusy() returns REFA_BUSY, this function
//! will have no effect.
//!
//! NOTE: Function not applicable for MSP430FR5xx Family
//!
//! Modified bits are \b REFOUT of \b REFCTL0 register.
//! \return NONE
//
//*****************************************************************************
void REFA_enableReferenceVoltageOutput (unsigned int baseAddress)
{
    HWREGB(baseAddress + OFS_REFCTL0_L) |= REFOUT;
}

//*****************************************************************************
//
//! Disables the reference voltage as an output to a pin.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to disables the reference voltage being generated to
//! be given to an output pin.
//! Please note, if the REFA_isRefGenBusy() returns REFA_ BUSY, this function
//! will have no effect.
//!
//! Modified bits are \b REFOUT of \b REFCTL0 register.
//! \return NONE
//
//*****************************************************************************
void REFA_disableReferenceVoltageOutput (unsigned int baseAddress)
{
    HWREGB(baseAddress + OFS_REFCTL0_L) &= ~(REFOUT);
}

//*****************************************************************************
//
//! Enables the reference voltage to be used by peripherals.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to enable the generated reference voltage to be used
//! other peripherals or by an output pin, if enabled. Please note, that giving
//! ADC12 module control over the REF module, the state of the reference voltage
//! is dependent on the controls of the ADC12 module.
//! Please note, if the REFA_isRefGenBusy() returns REFA_ BUSY, this function
//! will have no effect.
//!
//! Modified bits are \b REFON of \b REFCTL0 register.
//! \return NONE
//
//*****************************************************************************
void REFA_enableReferenceVoltage (unsigned int baseAddress)
{
    HWREGB(baseAddress + OFS_REFCTL0_L) |= REFON;
}

//*****************************************************************************
//
//! Disables the reference voltage.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to disable the generated reference voltage.
//! Please note, if the REFA_isRefGenBusy() returns REFA_ BUSY, this function
//! will have no effect.
//!
//! Modified bits are \b REFON of \b REFCTL0 register.
//! \return NONE
//
//*****************************************************************************
void REFA_disableReferenceVoltage (unsigned int baseAddress)
{
    HWREGB(baseAddress + OFS_REFCTL0_L) &= ~(REFON);
}

//*****************************************************************************
//
//! Returns the bandgap mode of the REF module.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to return the bandgap mode of the REF module,
//! requested by the peripherals using the bandgap. If a peripheral requests
//! static mode, then the bandgap mode will be static for all modules, whereas
//! if all of the peripherals using the bandgap request sample mode, then that
//! will be the mode returned. Sample mode allows the bandgap to be active only
//! when necessary to save on power consumption, static mode requires the
//! bandgap to be active until no peripherals are using it anymore.
//!
//! \return The bandgap mode of the REF module:
//!        REFA_STATICMODE if the bandgap is operating in static mode
//!        REFA_SAMPLEMODE if the bandgap is operating in sample mode
//
//*****************************************************************************
unsigned short REFA_getBandgapMode (unsigned int baseAddress)
{
    if (HWREGB((baseAddress) + OFS_REFCTL0_H) & BGMODE){
        return ( REFA_SAMPLEMODE) ;
    } else   {
        return ( REFA_STATICMODE) ;
    }
}

//*****************************************************************************
//
//! Returns the active status of the bandgap in the REF module.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to return the active status of the bandgap in the REF
//! module. If the bandgap is in use by a peripheral, then the status will be
//! seen as active.
//!
//! \return The bandgap active status of the REF module:
//!        REFA_INACTIVE if the bandgap is not being used at the time of query
//!        REFA_ACTIVE if the bandgap is being used at the time of query
//
//*****************************************************************************
unsigned short  REFA_isBandgapActive (unsigned int baseAddress)
{
    if (HWREGB((baseAddress) + OFS_REFCTL0_H) & REFBGACT){
        return ( REFA_ACTIVE) ;
    } else   {
        return ( REFA_INACTIVE) ;
    }
}

//*****************************************************************************
//
//! Returns the busy status of the reference generator in the REF module.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to return the busy status of the reference generator
//! in the REF module. If the ref. generator is in use by a peripheral, then the
//! status will be seen as busy.
//!
//! \return The reference generator busy status of the REF module:
//!        REFA_NOTBUSY if the reference generator is not being used
//!        REFA_BUSY if the reference generator is being used, disallowing any
//!                  changes to be made to the REF module controls
//
//*****************************************************************************
unsigned short REFA_isRefGenBusy (unsigned int baseAddress)
{
    if (HWREGB((baseAddress) + OFS_REFCTL0_H) & REFGENBUSY){
        return ( REFA_BUSY) ;
    } else   {
        return ( REFA_NOTBUSY) ;
    }
}

//*****************************************************************************
//
//! Returns the active status of the reference generator in the REF module.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to return the active status of the reference generator
//! in the REF module. If the ref. generator is on and ready to use, then the
//! status will be seen as active.
//!
//! \return The reference generator active status of the REF module:
//!        REFA_INACTIVE if the ref. generator is off and not operating
//!        REFA_ACTIVE if the ref. generator is on and ready to be used
//
//*****************************************************************************
unsigned short REFA_isRefGenActive (unsigned int baseAddress)
{
    if (HWREGB((baseAddress) + OFS_REFCTL0_H) & REFGENACT){
        return ( REFA_ACTIVE) ;
    } else   {
        return ( REFA_INACTIVE) ;
    }
}
//*****************************************************************************
//
//! Returns the busy status of the reference generator in the REF module.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to return the buys status of the buffered bandgap voltage
//! in the REF module. If the ref. generator is on and ready to use, then the
//! status will be seen as active.
//!
//! \return The reference generator active status of the REF module:
//!        REFA_NOTREADY if buffered bandgap voltage is NOT ready to be used
//!        REFA_READY if buffered bandgap voltage ready to be used
//
//*****************************************************************************
unsigned short REFA_getBufferedBandgapVoltageStatus(unsigned int baseAddress)
{
    if (HWREGB((baseAddress) + OFS_REFCTL0_H) & REFBGRDY){
        return ( REFA_READY) ;
    } else   {
        return ( REFA_NOTREADY) ;
    }
}

//*****************************************************************************
//
//! Returns the busy status of the variable reference voltage in the REF module.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! This function is used to return the buys status of the variable reference voltage
//! in the REF module. If the ref. generator is on and ready to use, then the
//! status will be seen as active.
//!
//! \return The reference generator active status of the REF module:
//!        REFA_NOTREADY if variable reference voltage is NOT ready to be used
//!        REFA_READY if variable reference voltage ready to be used
//
//*****************************************************************************
unsigned short REFA_getVariableReferenceVoltageStatus(unsigned int baseAddress)
{
    if (HWREGB((baseAddress) + OFS_REFCTL0_H) & REFGENRDY){
        return ( REFA_READY) ;
    } else   {
        return ( REFA_NOTREADY) ;
    }
}

//*****************************************************************************
//
//! Enables the one-time trigger of the reference voltage.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! Triggers the one-time generation of the variable reference voltage.  Once
//! the reference voltage request is set, this bit is cleared by hardware
//!
//! Modified bits are \b REFGENOT of \b REFCTL0 register.
//!
//! \return NONE
//
//*****************************************************************************
void REFA_setReferenceVoltageOneTimeTrigger(unsigned int baseAddress)
{
	HWREGB(baseAddress + OFS_REFCTL0_L) |= REFGENOT;
}

//*****************************************************************************
//
//! Enables the one-time trigger of the buffered bandgap voltage.
//!
//! \param baseAddress is the base address of the REF module.
//!
//! Triggers the one-time generation of the buffered bandgap voltage.  Once
//! the buffered bandgap voltage request is set, this bit is cleared by hardware
//!
//! Modified bits are \b REFBGOT of \b REFCTL0 register.
//!
//! \return NONE
//
//*****************************************************************************
void REFA_setBufferedBandgapVoltageOneTimeTrigger(unsigned int baseAddress)
{
	HWREGB(baseAddress + OFS_REFCTL0_L) |= REFBGOT;
}

//*****************************************************************************
//
//Close the Doxygen group.
//! @}
//
//*****************************************************************************
