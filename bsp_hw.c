/**
*
*  Copyright (c) 2013,2021 Yuri Tiomkin
*  All Rights Reserved
*
*
*  Permission to use, copy, modify, and distribute this software in source
*  and binary forms and its documentation for any purpose and without fee
*  is hereby granted, provided that the above copyright notice appear
*  in all copies and that both that copyright notice and this permission
*  notice appear in supporting documentation.
*
*
*  THIS SOFTWARE IS PROVIDED BY YURI TIOMKIN "AS IS" AND ANY EXPRESSED OR
*  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
*  IN NO EVENT SHALL YURI TIOMKIN OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
*  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
*  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
*  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
*  THE POSSIBILITY OF SUCH DAMAGE.
*
*/

/**
 * \file
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */


#include "samd21.h"
#include "hal_gpio.h"


int clock_init(void);
int usb_hw_init(void);
int32_t usb_d_init(void);

// Constants for Clock Generators

#define GENERIC_CLOCK_GENERATOR_MAIN      (0u)
#define GENERIC_CLOCK_GENERATOR_XOSC32K   (1u)
#define GENERIC_CLOCK_GENERATOR_OSCULP32K (2u) /* Initialized at reset for WDT */
#define GENERIC_CLOCK_GENERATOR_OSC8M     (3u)

// Constants for Clock multiplexers

#define GENERIC_CLOCK_MULTIPLEXER_DFLL48M (0u) 

HAL_GPIO_PIN(USB_DM,   A, 24);
HAL_GPIO_PIN(USB_DP,   A, 25);
 
#define NVM_USB_PAD_TRANSN_POS     45
#define NVM_USB_PAD_TRANSN_SIZE     5
#define NVM_USB_PAD_TRANSP_POS     50
#define NVM_USB_PAD_TRANSP_SIZE     5
#define NVM_USB_PAD_TRIM_POS       55
#define NVM_USB_PAD_TRIM_SIZE       3
 

//----------------------------------------------------------------------------
int clock_init(void)
{
    uint32_t tempDFLL48CalibrationCoarse;

    NVMCTRL->CTRLB.bit.RWS = 1;     /* 1 wait state required @ 3.3V & 48MHz */

    // Enable XOSC32K clock (External on-board 32.768kHz oscillator),
    // will be used as DFLL48M reference.

    // 1. Configure SYSCTRL->XOSC32K settings

    SYSCTRL_XOSC32K_Type sysctrl_xosc32k =
    {
        .bit.WRTLOCK  = 0,    // XOSC32K configuration is not locked
        .bit.STARTUP  = 2,    //  7 - max     //0x2,  // 3 cycle start-up time
        .bit.ONDEMAND = 0,    // Osc. is always running when enabled
        .bit.RUNSTDBY = 0,    // Osc. is disabled in standby sleep mode
        .bit.AAMPEN   = 0,    // Disable automatic amplitude control
        .bit.EN32K    = 1,    // 32kHz output is disabled
        .bit.XTALEN   = 1     // Crystal connected to XIN32/XOUT32
    };
    SYSCTRL->XOSC32K.reg = sysctrl_xosc32k.reg;

    SYSCTRL->XOSC32K.bit.ENABLE = 1;        // Enable the Oscillator - Separate step
                                            // per data sheet recommendation (sec 17.6.3)
    while(!SYSCTRL->PCLKSR.bit.XOSC32KRDY); // Wait for XOSC32K to stabilize

    // 2. Put XOSC32K as source of Generic Clock Generator 1

    GCLK_GENDIV_Type gclk1_gendiv =
    {
        .bit.DIV = 1,                              // Set output division factor = 1
        .bit.ID = GENERIC_CLOCK_GENERATOR_XOSC32K  // Apply division factor to Generator 1
    };
    GCLK->GENDIV.reg = gclk1_gendiv.reg;

    // Configure Generic Clock Generator 1 with XOSC32K as source
    GCLK_GENCTRL_Type gclk1_genctrl =
    {
        .bit.RUNSTDBY = 0,    // Generic Clock Generator is stopped in stdby
        .bit.DIVSEL   = 0,    // Use GENDIV.DIV value to divide the generator
        .bit.OE       = 0,    // Disable generator output to GCLK_IO[1]
        .bit.OOV      = 0,    // GCLK_IO[1] output value when generator is off
        .bit.IDC      = 1,    // Generator duty cycle is 50/50
        .bit.GENEN    = 1,    // Enable the generator
        .bit.SRC      = 0x05, // Generator source: XOSC32K output
        .bit.ID = GENERIC_CLOCK_GENERATOR_XOSC32K // Generator ID: 1
    };
    GCLK->GENCTRL.reg = gclk1_genctrl.reg;

    while(GCLK->STATUS.bit.SYNCBUSY); // GENCTRL is Write-Synchronized...
                                      // so wait for write to complete

    // 3) Put Generic Clock Generator 1 as source for
    //    Generic Clock Multiplexer 0 (DFLL48M reference)

    GCLK_CLKCTRL_Type gclk_clkctrl =
    {
        .bit.WRTLOCK = 0,     // Generic Clock is not locked from subsequent writes
        .bit.CLKEN   = 1,     // Enable the Generic Clock
        .bit.GEN     = 1,     // Num gen = 1,  Generic Clock Generator 1 is the source
        .bit.ID      = 0x00   // Generic Clock Multiplexer 0 (DFLL48M Reference)
    };
    GCLK->CLKCTRL.reg = gclk_clkctrl.reg;

    // 4) Enable DFLL48M clock

    // DFLL Configuration in Closed Loop mode, cf product data sheet chapter
    // 17.6.7.1 - Closed-Loop Operation

    // Enable the DFLL48M in open loop mode. Without this step,
    // attempts to go into closed loop mode at 48 MHz will
    // result in Processor Reset (you'll be at the in the Reset_Handler in startup_samd21.c).
    // PCLKSR.DFLLRDY must be one before writing to the DFLL Control register
    // Note that the DFLLRDY bit represents status of register synchronization - NOT clock stability
    // (see Data Sheet 17.6.14 Synchronization for detail)
    while(!SYSCTRL->PCLKSR.bit.DFLLRDY);

    SYSCTRL->DFLLCTRL.reg = (uint16_t)(SYSCTRL_DFLLCTRL_ENABLE);
    while(!SYSCTRL->PCLKSR.bit.DFLLRDY);

    // Set up the Multiplier, Coarse and Fine steps
    SYSCTRL_DFLLMUL_Type sysctrl_dfllmul =
    {
        .bit.CSTEP = 31,     // Coarse step - use half of the max value (63)
        .bit.FSTEP = 511,    // Fine step - use half of the max value (1023)
        .bit.MUL = 1465      // Multiplier = MAIN_CLK_FREQ (48MHz) / EXT_32K_CLK_FREQ (32768 Hz)
    };
    SYSCTRL->DFLLMUL.reg = sysctrl_dfllmul.reg;
    while(!SYSCTRL->PCLKSR.bit.DFLLRDY);

    // To reduce lock time, load factory calibrated values into DFLLVAL (cf. Data Sheet 17.6.7.1)
    // Location of value is defined in Data Sheet Table 10-5. NVM Software Calibration Area Mapping

    // Get factory calibrated value for "DFLL48M COARSE CAL" from NVM Software Calibration Area

    tempDFLL48CalibrationCoarse = *(uint32_t*)FUSES_DFLL48M_COARSE_CAL_ADDR;
    tempDFLL48CalibrationCoarse &= FUSES_DFLL48M_COARSE_CAL_Msk;
    tempDFLL48CalibrationCoarse = tempDFLL48CalibrationCoarse>>FUSES_DFLL48M_COARSE_CAL_Pos;

    SYSCTRL->DFLLVAL.bit.COARSE = tempDFLL48CalibrationCoarse; // Write the coarse calibration value

    // Switch DFLL48M to Closed Loop mode and enable WAITLOCK
    while(!SYSCTRL->PCLKSR.bit.DFLLRDY);

    SYSCTRL->DFLLCTRL.reg |= (uint16_t) (SYSCTRL_DFLLCTRL_MODE |
                                         SYSCTRL_DFLLCTRL_WAITLOCK);

    //--- 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.

    // Now that DFLL48M is running, switch CLKGEN0 source
    // to it to run the core at 48 MHz.

    GCLK_GENCTRL_Type gclk_genctrl0 =
    {
        .bit.RUNSTDBY = 0,   // Generic Clock Generator is stopped in stdby
        .bit.DIVSEL   = 0,   // Use GENDIV.DIV value to divide the generator
        .bit.OE       = 1,   // Enable generator output to GCLK_IO[0]
        .bit.OOV      = 0,   // GCLK_IO[0] output value when generator is off
        .bit.IDC      = 1,   // Generator duty cycle is 50/50
        .bit.GENEN    = 1,   // Enable the generator
        .bit.SRC    = 0x07,  // Generator source: DFLL48M output
        .bit.ID = GENERIC_CLOCK_GENERATOR_MAIN  // Generator ID: 0
    };
    GCLK->GENCTRL.reg = gclk_genctrl0.reg;

    // GENCTRL is Write-Synchronized...so wait for write to complete
    while(GCLK->STATUS.bit.SYNCBUSY);
 

    return 0;
}

//----------------------------------------------------------------------------
int usb_hw_init(void)
{
    uint32_t pad_transn, pad_transp, pad_trim;

//  Enable USB bus clock
    
    PM->APBBMASK.reg |= PM_APBBMASK_USB;
    PM->AHBMASK.reg  |= PM_AHBMASK_USB;

// Set I/O pins for USB
    
    HAL_GPIO_USB_DM_pmuxen(HAL_GPIO_PMUX_G);
    HAL_GPIO_USB_DP_pmuxen(HAL_GPIO_PMUX_G);

//--- Clock Generator 0 as source for Generic Clock Mux 6 (USB reference)
    
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                        GCLK_CLKCTRL_GEN(/*USB_GCLK_GEN*/ 0) |
                        GCLK_CLKCTRL_ID(USB_GCLK_ID);
    while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    {
    }
    
    // Reset
    
    USB->DEVICE.CTRLA.bit.SWRST = 1;
    while (USB->DEVICE.SYNCBUSY.bit.SWRST)
    {
    }

    /* Load Pad Calibration */

    pad_transn =  (*((uint32_t *)(NVMCTRL_OTP4)
                     + (NVM_USB_PAD_TRANSN_POS / 32))
                   >> (NVM_USB_PAD_TRANSN_POS % 32))
                   & ((1 << NVM_USB_PAD_TRANSN_SIZE) - 1);

    if(pad_transn == 0x1F)
    {
        pad_transn = 5;
    }
    USB->DEVICE.PADCAL.bit.TRANSN = pad_transn;
    
    pad_transp = (*((uint32_t *)(NVMCTRL_OTP4)
                    + (NVM_USB_PAD_TRANSP_POS / 32))
                  >> (NVM_USB_PAD_TRANSP_POS % 32))
                  & ((1 << NVM_USB_PAD_TRANSP_SIZE) - 1);

    if(pad_transp == 0x1F)
    {
        pad_transp = 29;
    }
    USB->DEVICE.PADCAL.bit.TRANSP = pad_transp;

    pad_trim = (*((uint32_t *)(NVMCTRL_OTP4)
                  + (NVM_USB_PAD_TRIM_POS / 32))
                >> (NVM_USB_PAD_TRIM_POS % 32))
                & ((1 << NVM_USB_PAD_TRIM_SIZE) - 1);

    if(pad_trim == 0x7)
    {
        pad_trim = 3;
    }
    USB->DEVICE.PADCAL.bit.TRIM = pad_trim;
    
//-----------------------------------------

    usb_d_init();
    
    return 0;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------


