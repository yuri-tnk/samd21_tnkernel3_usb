/**
*
*  Copyright (c) 2021 Yuri Tiomkin
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

#include "samd21.h"
#include "prj_conf.h"
#include "tn_config.h"
#include "tn.h"
#include "prj_def.h"

void usb_task(void);
void USB_Handler(void);
void _usb_d_dev_handler(void);  // YVT

//----------------------------------------------------------------------------
void tn_cpu_int_enable(void)
{
    // Enable a peripheral Int (Sys_Tick etc.)
    // The core Int is still disabled here

}

#if 0
//----------------------------------------------------------------------------
int tn_inside_int(void)
{
    int rc = 0;
    unsigned int ipsr;
    ipsr = __get_IPSR();
    if((ipsr & VECTACTIVE_M0) != 0UL)
    {
        rc = 1;
    }
    return rc;
}
#endif

//----------------------------------------------------------------------------
void SysTick_Handler(void)
{
    tn_tick_int_processing();

    //-- !!!    For the Cortex CPU, this function is a last func in the
    //--     any user's interrupt handler

    tn_int_exit();
}

//----------------------------------------------------------------------------
void DMAC_Handler(void)
{
    bsp_dma_interrupt_handler();

    //-- !!!    For the Cortex CPU, this function is a last func in the
    //--     any user's interrupt handler

    tn_int_exit();
}

//----------------------------------------------------------------------------
void USB_Handler(void)
{
    _usb_d_dev_handler();  // YVT

    //-- !!!    For the Cortex CPU, this function is a last func in the
    //--     any user's interrupt handler

    tn_int_exit();
}

#if 0
//----------------------------------------------------------------------------
static void led_blink(void)
{
#define DELAY_MS  100
    static int cnt = 0;

    cnt++;
    if(cnt == DELAY_MS) // LED - on
    {
        HAL_GPIO_TX_LED_clr();
    }        
    else
    { 
        if(cnt == DELAY_MS * 2) // LED - off
        {
            cnt = 0;
            HAL_GPIO_TX_LED_set();
        }
    }  
#undef DELAY_MS
}
#endif

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
