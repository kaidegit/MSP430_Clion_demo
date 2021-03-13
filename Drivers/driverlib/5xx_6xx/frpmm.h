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
#ifndef __MSP430WARE_PMM_FR5xx_H__
#define __MSP430WARE_PMM_FR5xx_H__

#define  __MSP430_HAS_PMM_FR5xx__
#define  __MSP430_HAS_PMM_FRAM__


//*****************************************************************************
//
//The following are values that can be passed to the
//PMMInterruptStatus() API as the mask parameter.
//
//*****************************************************************************

#define FRPMM_PMMBORIFG   		PMMBORIFG	/* PMM Software BOR interrupt flag */
#define FRPMM_PMMRSTIFG   		PMMRSTIFG	/* PMM RESET pin interrupt flag */
#define FRPMM_PMMPORIFG   		PMMPORIFG	/* PMM Software POR interrupt flag */
#define FRPMM_SVSHIFG     		SVSHIFG		/* SVS high side interrupt flag */
#define FRPMM_SVSLIFG     		SVSLIFG		/* SVS low side interrupt flag, NOT available for FR58xx/59xx */
#define FRPMM_PMMLPM5IFG  		PMMLPM5IFG  /* LPM5 indication Flag */

#define FRPMM_ALL				(0xB7)

#define FRPMM_LPRST				PMMLPRST	/* Low-Power Reset Enable, ONLY available for FR58xx/59xx */

#define FRPMM_SVSHE				SVSHE
#define FRPMM_SVSLE				SVSLE
#define FRPMM_PMMREGOFF			PMMREGOFF
#define FRPMM_PMMSWPOR			PMMSWPOR
#define FRPMM_PMMSWBOR			PMMSWBOR
#define FRPMM_LOCKLPM5			LOCKLPM5

#define FRPMM_PW                PMMPW
#define FRPMM_PW_H              PMMPW_H

//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************
extern void FRPMM_enableLowPowerReset 			(unsigned int baseAddress); 	//ONLY available for FR58xx/59xx
extern void FRPMM_disableLowPowerReset 			(unsigned int baseAddress);		//ONLY available for FR58xx/59xx
extern void FRPMM_enableSVSH         			(unsigned int baseAddress);
extern void FRPMM_disableSVSH        			(unsigned int baseAddress);
extern void FRPMM_enableSVSL         			(unsigned int baseAddress);		//ONLY available for FR57xx
extern void FRPMM_disableSVSL        			(unsigned int baseAddress);		//ONLY available for FR57xx
extern void FRPMM_regOff             			(unsigned int baseAddress);
extern void FRPMM_regOn              			(unsigned int baseAddress);
extern void FRPMM_trigPOR            			(unsigned int baseAddress);
extern void FRPMM_trigBOR            			(unsigned int baseAddress);
extern void FRPMM_clearInterrupt       			(unsigned int baseAddress, unsigned int mask);
extern unsigned int FRPMM_getInterruptStatus    (unsigned int baseAddress, unsigned int mask);
extern void FRPMM_lockLPM5           			(unsigned int baseAddress);
extern void FRPMM_unlockLPM5         			(unsigned int baseAddress);

#endif
