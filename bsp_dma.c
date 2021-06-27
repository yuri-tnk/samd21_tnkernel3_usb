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

#include <stdbool.h>
#include "samd21.h"
#include "bsp_dmac.h"
#include "prj_conf.h"
#include "tn_config.h"
#include "tn.h"
#include "prj_def.h"

/*
   Project uses 3 DMA channels:

     ch 0 - UART 0 Rx - Cyclic (no interrupt)

     ch 1 - UART 0 Tx - Interrupt at the transfer complete

     ch 2 - Dummy; uses just to force DMA controller to refresh write-back memory
            We need it to get a fresh value of .DMAC_BTCNT of UART 0 Rx DMA
*/

#define  DMA_CH_UART0_RX    0
#define  DMA_CH_UART0_TX    1
#define  DMA_CH_DUMMY       2

#define DMAC_CHANNELS_NUMBER        3U  // UART0 Rx , UART0 Tx, Dummy


// DMAC channels object configuration structure (do not uses by DMA hardware)

typedef struct _DMAC_CH_OBJECT
{
    int                    inUse;
    DMAC_CHANNEL_CALLBACK  callback;
    uintptr_t              context;
    int                    busyStatus;

} DMAC_CH_OBJECT;

// Initial write back memory section for DMAC
COMPILER_ALIGNED(16) static  dmac_descriptor_registers_t _write_back_section[DMAC_CHANNELS_NUMBER];

// Descriptor section for DMAC
COMPILER_ALIGNED(16) static  dmac_descriptor_registers_t  descriptor_section[DMAC_CHANNELS_NUMBER];

// DMAC Channels object information structure - do not uses by DMA hardware
static DMAC_CH_OBJECT dmacChannelObj[DMAC_CHANNELS_NUMBER];

#define DMA_DUMMY_BYTES    4  // Aligned to 4
static uint32_t g_dummy_src_arr[DMA_DUMMY_BYTES / 4];
static uint32_t g_dummy_dst_arr[DMA_DUMMY_BYTES / 4];

//----------------------------------------------------------------------------
int bsp_dma_init(void)   // After HW reset
{
    DMAC_CH_OBJECT * dmacChObj = (DMAC_CH_OBJECT *)&dmacChannelObj[0];
    int channel;

    // Initialize DMAC Channel objects
    for(channel = 0U; channel < DMAC_CHANNELS_NUMBER; channel++)
    {
        dmacChObj->inUse      = 0U;
        dmacChObj->callback   = NULL;
        dmacChObj->context    = 0U;
        dmacChObj->busyStatus = false;

        // Point to next channel object

        dmacChObj++;
    }

    PM->AHBMASK.bit.DMAC_ = 1;
    PM->APBBMASK.bit.DMAC_ = 1;

    // Update the Base address and Write Back address register

    DMAC_REGS->DMAC_BASEADDR = (uint32_t) descriptor_section;
    DMAC_REGS->DMAC_WRBADDR  = (uint32_t)_write_back_section;

    /* Enable the DMAC module & Priority Level x Enable */

    DMAC_REGS->DMAC_CTRL = (uint16_t)(DMAC_CTRL_DMAENABLE_Msk |
                                      DMAC_CTRL_LVLEN0_Msk |
                                      DMAC_CTRL_LVLEN1_Msk |
                                      DMAC_CTRL_LVLEN2_Msk |
                                      DMAC_CTRL_LVLEN3_Msk);
    return 0;
}

//----------------------------------------------------------------------------
int bsp_dma_uart0_rx_init(const void * dstAddr,
                          int num_items,
                          int item_size)
{
    TN_INTSAVE_DATA

    uint8_t channelId = 0U;

    tn_disable_interrupt();

    // Save channel ID
    channelId = (uint8_t)DMAC_REGS->DMAC_CHID;

    // DMA channel 0  - UART0 Rx

    DMAC_REGS->DMAC_CHID = DMA_CH_UART0_RX; // Now setup DMA ch 0

    // Disable the DMA channel 0
    DMAC_REGS->DMAC_CHCTRLA &= (uint8_t)(~DMAC_CHCTRLA_ENABLE_Msk);

    while((DMAC_REGS->DMAC_CHCTRLA & DMAC_CHCTRLA_ENABLE_Msk) != 0U)
    {
        // Wait till Channel 0 is enabled
    }

    // Perform a reset for the allocated channel
    DMAC_REGS->DMAC_CHCTRLA = DMAC_CHCTRLA_SWRST_Msk;
    while(DMAC_REGS->DMAC_CHCTRLA & DMAC_CHCTRLA_SWRST_Msk);


    DMAC_REGS->DMAC_CHCTRLB =
        DMAC_CHCTRLB_LVL(1) |      // Channel Priority Level 1
        DMAC_CHCTRLB_TRIGSRC(SERCOM0_DMAC_ID_RX) | // SERCOM0 RX Trigger
        DMAC_CHCTRLB_TRIGACT_BEAT; // One trigger required for each beat transfer

    descriptor_section[DMA_CH_UART0_RX].DMAC_BTCTRL  = 
            (uint16_t)(DMAC_BTCTRL_STEPSIZE_X1 |
            DMAC_BTCTRL_DSTINC_Msk |
            DMAC_BTCTRL_BEATSIZE_BYTE |
            DMAC_BTCTRL_BLOCKACT_NOACT |
            DMAC_BTCTRL_VALID_Msk |
            DMAC_BTCTRL_STEPSEL_DST);  // Step size settings apply to the destination address Position

    descriptor_section[DMA_CH_UART0_RX].DMAC_BTCNT    = num_items;
    descriptor_section[DMA_CH_UART0_RX].DMAC_DESCADDR = (uint32_t)&descriptor_section[0];  // Circular
    descriptor_section[DMA_CH_UART0_RX].DMAC_SRCADDR  = (uint32_t)&SERCOM0->USART.DATA.reg;
    descriptor_section[DMA_CH_UART0_RX].DMAC_DSTADDR  = (uint32_t)((uint8_t *)dstAddr + (num_items * item_size));

    dmacChannelObj[0].inUse = 1U;

#if 0
    DMAC_REGS->DMAC_QOSCTRL = DMAC_QOSCTRL_WRBQOS_LOW |
                              DMAC_QOSCTRL_FQOS_LOW |
                              DMAC_QOSCTRL_DQOS_LOW;
#endif


    DMAC_REGS->DMAC_CHINTENSET = (uint8_t)(DMAC_CHINTENSET_TERR_Msk);
   
    // Enable the channel 0

    DMAC_REGS->DMAC_CHCTRLA |= (uint8_t)DMAC_CHCTRLA_ENABLE_Msk;
    //while(DMAC_REGS->CHSTATUS.bit.BUSY);

    // Restore channel ID

    DMAC_REGS->DMAC_CHID = channelId;

    tn_enable_interrupt();

    return 0;
}

//----------------------------------------------------------------------------
int bsp_dma_dummy_init(void)
{
    TN_INTSAVE_DATA

    uint32_t channelId = 0U;
    int num_bytes = DMA_DUMMY_BYTES;

    g_dummy_src_arr[0] = 1234U;
    g_dummy_dst_arr[0] = 0U;

    tn_disable_interrupt();

    channelId = DMAC_REGS->DMAC_CHID;    // Save channel ID

    DMAC_REGS->DMAC_CHID = DMA_CH_DUMMY; // Now setup DMA ch 2

    // Disable the DMA channel 2

    DMAC_REGS->DMAC_CHCTRLA &= (uint8_t)(~DMAC_CHCTRLA_ENABLE_Msk);

    while((DMAC_REGS->DMAC_CHCTRLA & DMAC_CHCTRLA_ENABLE_Msk) != 0U)
    {
        // Wait till Channel 2 is enabled
    }

    dmacChannelObj[DMA_CH_DUMMY].busyStatus = false;

    // Bits 25:24 CMD[1:0]     Software Command      0x0  NOACT
    // Bits 23:22 TRIGACT[1:0] Trigger Action        0x0  BLOCK   One trigger required for each block transfer
    // Bits 13:8  TRIGSRC[5:0] Trigger Source       0x00  DISABLE Only software/event triggers
    // Bits 6:5   LVL[1:0] Channel Arbitration Level      We set it to 2
    // Bit 4      EVOE Channel Event Output Enable     0  Channel event generation is disabled
    // Bit 3      EVIE Channel Event Input Enable      0  Channel event action will not be executed on any incoming event
    // Bits 2:0   EVACT[2:0] Event Input Action      0x0  NOACT   No action

    DMAC_REGS->DMAC_CHCTRLB = DMAC_CHCTRLB_LVL(2UL) ;     // Channel Arbitration Level is set to 2


    descriptor_section[DMA_CH_DUMMY].DMAC_BTCTRL  = 
                (uint16_t)(DMAC_BTCTRL_SRCINC_Msk |
                DMAC_BTCTRL_DSTINC_Msk |
                DMAC_BTCTRL_BEATSIZE_WORD |
                DMAC_BTCTRL_VALID_Msk);

    descriptor_section[DMA_CH_DUMMY].DMAC_BTCNT    = num_bytes / 4;
    descriptor_section[DMA_CH_DUMMY].DMAC_DESCADDR = 0U; // Do it once
    descriptor_section[DMA_CH_DUMMY].DMAC_DSTADDR  = (uint32_t)&g_dummy_dst_arr[0] + num_bytes;
    descriptor_section[DMA_CH_DUMMY].DMAC_SRCADDR  = (uint32_t)&g_dummy_src_arr[0] + num_bytes;

    dmacChannelObj[DMA_CH_DUMMY].inUse = 1U;

    DMAC_REGS->DMAC_CHID = (uint8_t)channelId; // Restore channel ID

    tn_enable_interrupt();

    return 0;
}

//----------------------------------------------------------------------------
int bsp_dma_dummy_start(void)
{
    TN_INTSAVE_DATA

    uint32_t channelId = 0U;

    tn_disable_interrupt();

    channelId = DMAC_REGS->DMAC_CHID;          // Save channel ID

    DMAC_REGS->DMAC_CHID = DMA_CH_DUMMY;       // Now setup DMA ch 2

    DMAC_REGS->DMAC_CHCTRLA    |= DMAC_CHCTRLA_ENABLE_Msk; // enable channel
    DMAC_REGS->DMAC_SWTRIGCTRL |= (1 << DMA_CH_DUMMY);     // trigger channel

    DMAC_REGS->DMAC_CHID = (uint8_t)channelId; // Restore channel ID

    tn_enable_interrupt();

    return 0;
}

//----------------------------------------------------------------------------
int bsp_dma_uart0_tx_init(const void * srcAddr,
                          int num_items,
                          int item_size,
                          const DMAC_CHANNEL_CALLBACK eventHandler)
{
    TN_INTSAVE_DATA

    uint32_t channelId = 0U;

    tn_disable_interrupt();

    // Save channel ID
    channelId = DMAC_REGS->DMAC_CHID;

    DMAC_REGS->DMAC_CHID = DMA_CH_UART0_TX; // Now setup DMA ch 1

    // Disable the DMA curr channel (ch 1)
    DMAC_REGS->DMAC_CHCTRLA &= (uint8_t)(~DMAC_CHCTRLA_ENABLE_Msk);

    while((DMAC_REGS->DMAC_CHCTRLA & DMAC_CHCTRLA_ENABLE_Msk) != 0U)
    {
        // Wait till Channel 1 is enabled
    }

    dmacChannelObj[DMA_CH_UART0_TX].busyStatus = false;

    DMAC_REGS->DMAC_CHCTRLB = DMAC_CHCTRLB_TRIGACT(2UL) | // One trigger required for each beat transfer
                              DMAC_CHCTRLB_TRIGSRC(2UL) | // SERCOM0 TX Trigger
                              DMAC_CHCTRLB_LVL(0UL);      // Channel Priority Level 0

    descriptor_section[DMA_CH_UART0_TX].DMAC_BTCTRL  = 
            (uint16_t)(DMAC_BTCTRL_STEPSIZE_X1 |
            DMAC_BTCTRL_SRCINC_Msk |
            DMAC_BTCTRL_BEATSIZE_BYTE |
            DMAC_BTCTRL_BLOCKACT_NOACT |
            DMAC_BTCTRL_VALID_Msk |
            DMAC_BTCTRL_STEPSEL_SRC);  // Step size settings apply to the source address Position

    descriptor_section[DMA_CH_UART0_TX].DMAC_BTCNT    = num_items;
    descriptor_section[DMA_CH_UART0_TX].DMAC_DESCADDR = 0U; // Do it once
    descriptor_section[DMA_CH_UART0_TX].DMAC_DSTADDR  = (uint32_t)&SERCOM0->USART.DATA.reg;
    descriptor_section[DMA_CH_UART0_TX].DMAC_SRCADDR  = (uint32_t)((uint8_t *)srcAddr + (num_items * item_size));

    dmacChannelObj[DMA_CH_UART0_TX].inUse = 1U;

    DMAC_REGS->DMAC_CHINTENSET = (uint8_t)(DMAC_CHINTENSET_TERR_Msk |
                                           DMAC_CHINTENSET_TCMPL_Msk); // Enable Transfer Complete Interrupt

    dmacChannelObj[DMA_CH_UART0_TX].callback = eventHandler;

    // Enable 'TransferComplete' (or Error) DMA interrupt

    NVIC_SetPriority(DMAC_IRQn, 0);
    NVIC_EnableIRQ(DMAC_IRQn);

    // Enable the channel 1  (DMA_CH_UART0_TX)

    DMAC_REGS->DMAC_CHCTRLA |= (uint8_t)DMAC_CHCTRLA_ENABLE_Msk;

    // Restore channel ID
    DMAC_REGS->DMAC_CHID = (uint8_t)channelId;

    tn_enable_interrupt();

    return 0;
}

//----------------------------------------------------------------------------
void bsp_dma_uart0_tx_ch_disable(void)
{
    TN_INTSAVE_DATA

    uint32_t channelId = 0U;

    tn_disable_interrupt();

    channelId = DMAC_REGS->DMAC_CHID;          // Save channel ID

    DMAC_REGS->DMAC_CHID = DMA_CH_UART0_TX;    // Set the DMA Channel ID

    // Disable the DMA channel
    DMAC_REGS->DMAC_CHCTRLA &= (uint8_t)(~DMAC_CHCTRLA_ENABLE_Msk);

    while((DMAC_REGS->DMAC_CHCTRLA & DMAC_CHCTRLA_ENABLE_Msk) != 0U)
    {
    }

    dmacChannelObj[DMA_CH_UART0_TX].busyStatus = false;

    DMAC_REGS->DMAC_CHID = (uint8_t)channelId; // Restore channel ID

    tn_enable_interrupt();
}

//----------------------------------------------------------------------------
int bsp_dma_uart0_get_btcnt(void)
{
    return (int)_write_back_section[DMA_CH_UART0_RX].DMAC_BTCNT;
}

//----------------------------------------------------------------------------
int bsp_dma_uart0_get_rx_count(void)
{
    int transferredCount  = descriptor_section[DMA_CH_UART0_RX].DMAC_BTCNT;
    transferredCount     -= _write_back_section[DMA_CH_UART0_RX].DMAC_BTCNT;

    return transferredCount;
}

//----------------------------------------------------------------------------
void bsp_dma_interrupt_handler(void)
{
    TN_INTSAVE_DATA_INT

    DMAC_CH_OBJECT  *dmacChObj = NULL;
    uint8_t channel = 0U;
    uint32_t channelId = 0U;

    volatile uint32_t chanIntFlagStatus = 0U;

    DMAC_TRANSFER_EVENT event = DMAC_TRANSFER_EVENT_ERROR;

    tn_idisable_interrupt();

    // Get active channel number
    channel = (uint8_t)((uint32_t)DMAC_REGS->DMAC_INTPEND & DMAC_INTPEND_ID_Msk);

    dmacChObj = (DMAC_CH_OBJECT *)&dmacChannelObj[channel];

    // Save channel ID
    channelId = DMAC_REGS->DMAC_CHID;

    // Update the DMAC channel ID
    DMAC_REGS->DMAC_CHID = channel;

    // Get the DMAC channel interrupt status
    chanIntFlagStatus = DMAC_REGS->DMAC_CHINTFLAG;

    // Verify if DMAC Channel Transfer complete flag is set
    if((chanIntFlagStatus & DMAC_CHINTENCLR_TCMPL_Msk) == DMAC_CHINTENCLR_TCMPL_Msk)
    {
        // Clear the transfer complete flag
        DMAC_REGS->DMAC_CHINTFLAG = (uint8_t)DMAC_CHINTENCLR_TCMPL_Msk;

        event = DMAC_TRANSFER_EVENT_COMPLETE;

        dmacChObj->busyStatus = false;
    }

    // Verify if DMAC Channel Error flag is set
    if((chanIntFlagStatus & DMAC_CHINTENCLR_TERR_Msk) == DMAC_CHINTENCLR_TERR_Msk)
    {
        // Clear transfer error flag
        DMAC_REGS->DMAC_CHINTFLAG = (uint8_t)DMAC_CHINTENCLR_TERR_Msk;

        event = DMAC_TRANSFER_EVENT_ERROR;

        dmacChObj->busyStatus = false;
    }

    // Execute the callback function
    if(dmacChObj->callback != NULL)
    {
        dmacChObj->callback(event, dmacChObj->context);
    }

    // Restore channel ID
    DMAC_REGS->DMAC_CHID = (uint8_t)channelId;

    tn_ienable_interrupt();
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
