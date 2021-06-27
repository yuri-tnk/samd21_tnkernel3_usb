/*
 * Copyright (c) 2015, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//-----------------------------------------------------------------------------
#include "samd21.h"
#include "bsp_dmac.h"

//-----------------------------------------------------------------------------
#define DUMMY __attribute__ ((weak, alias ("irq_handler_dummy")))

//-----------------------------------------------------------------------------
void Reset_Handler(void);
DUMMY void NMI_Handler(void);
DUMMY void HardFault_Handler(void);
DUMMY void SVC_Handler(void);
DUMMY void PendSV_Handler(void);
DUMMY void SysTick_Handler(void);

DUMMY void PM_Handler(void);
DUMMY void SYSCTRL_Handler(void);
DUMMY void WDT_Handler(void);
DUMMY void RTC_Handler(void);
DUMMY void EIC_Handler(void);
DUMMY void NVMCTRL_Handler(void);
DUMMY void DMAC_Handler(void);
DUMMY void USB_Handler(void);
DUMMY void EVSYS_Handler(void);
DUMMY void SERCOM0_Handler(void);
DUMMY void SERCOM1_Handler(void);
DUMMY void SERCOM2_Handler(void);
DUMMY void SERCOM3_Handler(void);
DUMMY void SERCOM4_Handler(void);
DUMMY void SERCOM5_Handler(void);
DUMMY void TCC0_Handler(void);
DUMMY void TCC1_Handler(void);
DUMMY void TCC2_Handler(void);
DUMMY void TC3_Handler(void);
DUMMY void TC4_Handler(void);
DUMMY void TC5_Handler(void);
DUMMY void ADC_Handler(void);
DUMMY void AC_Handler(void);
DUMMY void DAC_Handler(void);
DUMMY void PTC_Handler(void);
DUMMY void I2S_Handler(void);


extern int main(void);

extern void _stack_top(void);
extern unsigned int _etext;
extern unsigned int _data;
extern unsigned int _edata;
extern unsigned int _bss;
extern unsigned int _ebss;

//-----------------------------------------------------------------------------
__attribute__ ((used, section(".vectors")))
void (* const vectors[])(void) =
{
    &_stack_top,                   // 0 - Initial Stack Pointer Value

    // Cortex-M0+ handlers
    Reset_Handler,                 //  1 - Reset
    NMI_Handler,                   //  2 - NMI
    HardFault_Handler,             //  3 - Hard Fault
    0,                             //  4 - Reserved
    0,                             //  5 - Reserved
    0,                             //  6 - Reserved
    0,                             //  7 - Reserved
    0,                             //  8 - Reserved
    0,                             //  9 - Reserved
    0,                             // 10 - Reserved
    SVC_Handler,                   // 11 - SVCall
    0,                             // 12 - Reserved
    0,                             // 13 - Reserved
    PendSV_Handler,                // 14 - PendSV
    SysTick_Handler,               // 15 - SysTick

    // Peripheral handlers

    PM_Handler,           /**<  0 SAMD21G18A Power Manager (PM) */
    SYSCTRL_Handler,      /**<  1 SAMD21G18A System Control (SYSCTRL) */
    WDT_Handler,          /**<  2 SAMD21G18A Watchdog Timer (WDT) */
    RTC_Handler,          /**<  3 SAMD21G18A Real-Time Counter (RTC) */
    EIC_Handler,          /**<  4 SAMD21G18A External Interrupt Controller (EIC) */
    NVMCTRL_Handler,      /**<  5 SAMD21G18A Non-Volatile Memory Controller (NVMCTRL) */
    DMAC_Handler,         /**<  6 SAMD21G18A Direct Memory Access Controller (DMAC) */
    USB_Handler,          /**<  7 SAMD21G18A Universal Serial Bus (USB) */
    EVSYS_Handler,        /**<  8 SAMD21G18A Event System Interface (EVSYS) */
    SERCOM0_Handler,      /**<  9 SAMD21G18A Serial Communication Interface 0 (SERCOM0) */
    SERCOM1_Handler,      /**< 10 SAMD21G18A Serial Communication Interface 1 (SERCOM1) */
    SERCOM2_Handler,      /**< 11 SAMD21G18A Serial Communication Interface 2 (SERCOM2) */
    SERCOM3_Handler,      /**< 12 SAMD21G18A Serial Communication Interface 3 (SERCOM3) */
    SERCOM4_Handler,      /**< 13 SAMD21G18A Serial Communication Interface 4 (SERCOM4) */
    SERCOM5_Handler,      /**< 14 SAMD21G18A Serial Communication Interface 5 (SERCOM5) */
    TCC0_Handler,         /**< 15 SAMD21G18A Timer Counter Control 0 (TCC0) */
    TCC1_Handler,         /**< 16 SAMD21G18A Timer Counter Control 1 (TCC1) */
    TCC2_Handler,         /**< 17 SAMD21G18A Timer Counter Control 2 (TCC2) */
    TC3_Handler,          /**< 18 SAMD21G18A Basic Timer Counter 3 (TC3) */
    TC4_Handler,          /**< 19 SAMD21G18A Basic Timer Counter 4 (TC4) */
    TC5_Handler,          /**< 20 SAMD21G18A Basic Timer Counter 5 (TC5) */
    ADC_Handler,          /**< 23 SAMD21G18A Analog Digital Converter (ADC) */
    AC_Handler,           /**< 24 SAMD21G18A Analog Comparators (AC) */
    DAC_Handler,          /**< 25 SAMD21G18A Digital Analog Converter (DAC) */
    PTC_Handler,          /**< 26 SAMD21G18A Peripheral Touch Controller (PTC) */
    I2S_Handler,          /**< 27 SAMD21G18A Inter-IC Sound Interface (I2S) */

};

//-----------------------------------------------------------------------------
void Reset_Handler(void)
{
    unsigned int *src, *dst;

        /* Change default QOS values to have the best performance and correct USB behaviour */
    SBMATRIX->SFR[SBMATRIX_SLAVE_HMCRAMC0].reg = 2;

#if defined(ID_USB)
    USB->DEVICE.QOSCTRL.bit.CQOS = 2;
    USB->DEVICE.QOSCTRL.bit.DQOS = 2;
#endif
    DMAC_REGS->DMAC_QOSCTRL =  DMAC_QOSCTRL_DQOS_MEDIUM |
                               DMAC_QOSCTRL_FQOS_MEDIUM |
                               DMAC_QOSCTRL_WRBQOS_MEDIUM;  

//        DMAC->QOSCTRL.bit.DQOS = 2;
//        DMAC->QOSCTRL.bit.FQOS = 2;
//        DMAC->QOSCTRL.bit.WRBQOS = 2;

   /* Overwriting the default value of the NVMCTRL.CTRLB.MANW bit (errata reference 13134) */
   NVMCTRL->CTRLB.bit.MANW = 1;


    src = &_etext;
    dst = &_data;
    while(dst < &_edata)
    { 
        *dst++ = *src++;
    }

    dst = &_bss;
    while(dst < &_ebss)
    {
        *dst++ = 0;
    }

    SCB->VTOR = (uint32_t)vectors;

    main();

    while(1)
    {}
}

//-----------------------------------------------------------------------------
void irq_handler_dummy(void)
{
    while(1)
    {}
}

//-----------------------------------------------------------------------------
void _exit(int status)
{
    (void)status;
    while(1)
    {}
}
/* ------------------------------------------- */
