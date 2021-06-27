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

#include <string.h>
#include "samd21.h"
#include "hal_gpio.h"
#include "prj_conf.h"
#include "tn_config.h"
#include "tn.h"

#include "prj_def.h"


#define UART_STATE_CLOSE         0
#define UART_STATE_IN_PROGRESS   1
#define UART_STATE_OPEN          2


enum
{
    USB_CDC_1_STOP_BIT    = 0,
    USB_CDC_1_5_STOP_BITS = 1,
    USB_CDC_2_STOP_BITS   = 2,
};

enum
{
    USB_CDC_NO_PARITY     = 0,
    USB_CDC_ODD_PARITY    = 1,
    USB_CDC_EVEN_PARITY   = 2,
    USB_CDC_MARK_PARITY   = 3,
    USB_CDC_SPACE_PARITY  = 4,
};

enum
{
    USB_CDC_5_DATA_BITS   = 5,
    USB_CDC_6_DATA_BITS   = 6,
    USB_CDC_7_DATA_BITS   = 7,
    USB_CDC_8_DATA_BITS   = 8,
    USB_CDC_16_DATA_BITS  = 16,
};

static int bsp_uart0_hw_init(UARTBAUDRATE baud_rate);
static void dma_eventHandler(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle);

HAL_GPIO_PIN(UART_TX, A, 4);
HAL_GPIO_PIN(UART_RX, A, 5);

//----------------------------------------------------------------------------
int bsp_uart0_open(uint8_t * rx_buf,
                   unsigned int rx_buf_items,
                   uint8_t * tx_buf,
                   unsigned int tx_buf_items,
                   UARTBAUDRATE baud_rate)
{
    TN_INTSAVE_DATA
    int rc = TERR_ILUSE;
    int state = UART_STATE_IN_PROGRESS;

    UARTINFO * ui = get_uart0_info();

    if(ui == NULL || rx_buf == NULL || rx_buf_items < 8 ||
            tx_buf == NULL || tx_buf_items < 8)
    {
        rc = TERR_WPARAM;
    }
    else
    {
        tn_disable_interrupt();
        if(ui->state == UART_STATE_CLOSE)
        {
            ui->state = UART_STATE_IN_PROGRESS; //!< as a flag for 'open in progress'
            state = UART_STATE_CLOSE;
        }
        tn_enable_interrupt();

        if(state == UART_STATE_CLOSE)
        {
            ui->rx_tail        = 0UL;
            ui->rx_buf         = rx_buf;
            ui->rx_buf_items   = rx_buf_items;
            ui->tx_buf         = tx_buf;
            ui->tx_buf_items   = tx_buf_items;

            ui->rx_timeout     = TN_WAIT_INFINITE;  // default value
            ui->rx_timeout_cnt = ui->rx_timeout;

            // Create semaphores

            ui->tx_str_sem.id_sem = 0U;
            rc = tn_sem_create(&ui->tx_str_sem,
                               1,
                               1);
            if(rc == TERR_NO_ERR) // && rc1 == TERR_NO_ERR)
            {
                ui->tx_rdy_sem.id_sem = 0U;
                rc = tn_sem_create(&ui->tx_rdy_sem,
                                   0,
                                   1);
                if(rc == TERR_NO_ERR)
                {
                    // Enable clocks, set I/O for UART, set UART registers

                    rc = bsp_uart0_hw_init(UART_BAUD_115200);
                    if(rc == TERR_NO_ERR)
                    {
                        // Set RX DMA
                        (void)bsp_dma_uart0_rx_init((const void *)ui->rx_buf,
                                                    ui->rx_buf_items,//  int num_items,
                                                    1); //  int item_size);

                        tn_disable_interrupt();
                        ui->state = UART_STATE_OPEN;
                        tn_enable_interrupt();

                        rc = TERR_NO_ERR;
                    }
                }
            }
        }
    }

    return rc;
}


//----------------------------------------------------------------------------
// Runs inside DMA interrupt
//----------------------------------------------------------------------------
static void dma_eventHandler(DMAC_TRANSFER_EVENT event, uintptr_t contextHandle)
{
    UARTINFO * ui = get_uart0_info();

    (void)tn_sem_signal(&ui->tx_rdy_sem);
}

//----------------------------------------------------------------------------
int bsp_uart0_transmit(uint8_t * data,
                       int data_size)
{
    int rc;
    UARTINFO * ui = get_uart0_info();

    if(ui == NULL || data == NULL || data_size == 0U)
    {
        rc = TERR_WPARAM;
    }
    else
    {
        if(ui->state == UART_STATE_OPEN)
        {
            // Set DMA 1 and start transfer

            rc = bsp_dma_uart0_tx_init((const void *) data, // srcAddr,
                                       data_size,          // int num_items,
                                       1,                  // int item_size,
                                       dma_eventHandler);
            if(rc != 0)
            {
                // ToDo
            }

            // Wait end of Tx

            rc = tn_sem_acquire(&ui->tx_rdy_sem, 2000); // 2 s (at 115200 baud, this is ~23kbytes)

            // Disable Tx DMA - any case

            bsp_dma_uart0_tx_ch_disable();

            NVIC_DisableIRQ(DMAC_IRQn);

            if(rc != TERR_NO_ERR) // Error - timeout etc.
            {
                // Error Handler
                for(;;){}
            }
        }
        else
        {
            rc = TERR_ILUSE;
        }
    }

    return rc;
}

//----------------------------------------------------------------------------
int uart0_tx_str(char * str)
{
    int len = (int)strlen(str);
    return uart0_tx_buf((uint8_t *) str, len);
}

//----------------------------------------------------------------------------
int uart0_tx_buf(uint8_t * str, int len)
{
    int rc = TERR_ILUSE;
    unsigned char * ptr;
    int nbytes;
    int nb;
    int i;

    UARTINFO * ui = get_uart0_info();

    if(str != NULL && ui->state == UART_STATE_OPEN)
    {
        if(len > 0)
        {
            ptr = str;
            nbytes = len;

            tn_sem_acquire(&ui->tx_str_sem, TN_WAIT_INFINITE);
            while(nbytes > 0)
            {
                if(nbytes > ui->tx_buf_items)
                {
                    nb = ui->tx_buf_items;
                }
                else
                {
                    nb = nbytes;
                }

                for(i = 0; i < nb; i++)
                {
                    ui->tx_buf[i] = (uint16_t)ptr[i];
                }

                rc = bsp_uart0_transmit(ui->tx_buf,
                                        nb);
                nbytes -= nb;
                ptr    += nb;
            }
            tn_sem_signal(&ui->tx_str_sem);
            rc = 0; // OK
        }
    }

    return rc;
}

//----------------------------------------------------------------------------
void uart0_tx_char(unsigned char ch)
{
    uint8_t buf[4];
    UARTINFO * ui = get_uart0_info();

    buf[0] = (uint16_t)ch;

    if(ui->state == UART_STATE_OPEN)
    {
        tn_sem_acquire(&ui->tx_str_sem, TN_WAIT_INFINITE);

        (void)bsp_uart0_transmit(buf,
                                 1U);

        tn_sem_signal(&ui->tx_str_sem);
    }
}

//----------------------------------------------------------------------------
int uart0_read(unsigned char * buf,
               unsigned int max_len)
{
    unsigned int nbytes;
    volatile unsigned int btcnt;
    int ind   = 0;
    BOOL fExit = FALSE;

    UARTINFO * ui = get_uart0_info();

    if(ui == NULL || buf == NULL || max_len == 0U)
    {
        ind = TERR_WPARAM;
    }
    else
    {
        if(ui->state == UART_STATE_OPEN)
        {
            ui->rx_timeout_cnt = ui->rx_timeout;

            if(max_len < ui->rx_buf_items)
            {
                nbytes = max_len;
            }
            else
            {
                nbytes = ui->rx_buf_items;
            }

            while(fExit == FALSE)
            {
                //!< ----- rx char ----------------------------

                // at start, btcnt (DMAC_BTCTRL ch 0) = ui->rx_buf_items
                // each Rxed char - 'btcnt' decremented

                btcnt = bsp_dma_uart0_get_btcnt();

                if(btcnt + ui->rx_tail == ui->rx_buf_items)  // no new char
                {
                    if(ind > 0) //!< We received some character(s) before
                    {
                        fExit = TRUE;
                    }
                    else // no chars in 'buf'
                    {
                        if(ui->rx_timeout == TN_NO_WAIT)
                        {
                            ind = TERR_TIMEOUT;
                            fExit = TRUE;
                        }
                        else if(ui->state != UART_STATE_OPEN)
                        {
                            ind = TERR_ILUSE;
                            fExit = TRUE;
                        }
                        else
                        {
                            (void)tn_task_sleep(1); //!< Sleep (here 1 mS)

                            if(ui->rx_timeout != TN_WAIT_INFINITE)
                            {
                                ui->rx_timeout_cnt--;
                                if(ui->rx_timeout_cnt == 0U)
                                {
                                    ind = TERR_TIMEOUT;
                                    fExit = TRUE;
                                }
                            }
                        }
                    }
                }
                else // got char(s)
                {
                    buf[ind++] = (uint8_t)ui->rx_buf[ui->rx_tail++];  //!< rd data

                    if(ind >= (int)nbytes)
                    {
                        fExit = TRUE;
                    }

                    if(ui->rx_tail >= ui->rx_buf_items)
                    {
                        ui->rx_tail = 0;
                    }
                }
            } //!< while(!fExit)
        }
        else
        {
            ind = TERR_ILUSE;
        }
    }

    return ind;
}


//----------------------------------------------------------------------------
static int bsp_uart0_hw_init(UARTBAUDRATE baud_rate)
{
    int chsize, form, pmode, sbmode, baud, fp;

    int DataBits   = USB_CDC_8_DATA_BITS;
    int ParityType = USB_CDC_NO_PARITY;
    int CharFormat = USB_CDC_1_STOP_BIT;
    int BaudRate   = (int)baud_rate;

    //--- Set UART pins

    HAL_GPIO_UART_TX_out();
    HAL_GPIO_UART_TX_pmuxen(HAL_GPIO_PMUX_D);  //-- Set PMUX TX to HAL_GPIO_PMUX_D (3)
    HAL_GPIO_UART_RX_in();
    HAL_GPIO_UART_RX_pullup();                 //-- Add pull-up resistor to RX input
    HAL_GPIO_UART_RX_pmuxen(HAL_GPIO_PMUX_D);  //-- Set PMUX RX to HAL_GPIO_PMUX_D (3)

#if 0

    for(;;)
    {
        for(fp = 0; fp < 10000; fp++);
        HAL_GPIO_UART_TX_set();
        for(fp = 0; fp < 10000; fp++);
        HAL_GPIO_UART_TX_clr();
    }
#endif

    //--- Turn on power manager for sercom

    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;

    //--- configure GCLK0 to feed sercom0

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM0_GCLK_ID_CORE) |
                        GCLK_CLKCTRL_CLKEN |
                        GCLK_CLKCTRL_GEN(0);
    while (GCLK->STATUS.bit.SYNCBUSY);

//--------------- Orig

    // Reset UART

    SERCOM0->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
    while (SERCOM0->USART.CTRLA.bit.SWRST);

    switch(DataBits)
    {
        case USB_CDC_5_DATA_BITS:
            chsize = 5;
            break;
        case USB_CDC_6_DATA_BITS:
            chsize = 6;
            break;
        case USB_CDC_7_DATA_BITS:
            chsize = 7;
            break;
        case USB_CDC_8_DATA_BITS:
            chsize = 0;
            break;
        default:
            chsize = 0;
            break;
    }

    if(ParityType == USB_CDC_NO_PARITY)
    {
        form = 0;
    }
    else
    {
        form = 1;
    }

    if(ParityType == USB_CDC_EVEN_PARITY)
    {
        pmode = 0;
    }
    else
    {
        pmode = SERCOM_USART_CTRLB_PMODE;
    }

    if(CharFormat == USB_CDC_1_STOP_BIT)
    {
        sbmode = 0;
    }
    else
    {
        sbmode = SERCOM_USART_CTRLB_SBMODE;
    }

    baud = F_CPU / (16 * BaudRate);
    fp = (F_CPU / BaudRate - 16 * baud) / 2;

    //  set to LSB, asynchronous mode without parity, PAD0 Tx, PAD1 Rx,
    //  16x over-sampling, internal clk

    SERCOM0->USART.CTRLA.reg = SERCOM_USART_CTRLA_DORD |
                               SERCOM_USART_CTRLA_RUNSTDBY |
                               SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
                               SERCOM_USART_CTRLA_FORM(form) |
                               SERCOM_USART_CTRLA_SAMPR(1) |
                               SERCOM_USART_CTRLA_RXPO(1) |
                               SERCOM_USART_CTRLA_TXPO(0);

    // enable receiver and transmitter, one stop bit

    SERCOM0->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN |
                               SERCOM_USART_CTRLB_TXEN |
                               SERCOM_USART_CTRLB_CHSIZE(chsize) |
                               pmode |
                               sbmode;
    while(SERCOM0->USART.SYNCBUSY.bit.CTRLB); // synchronization busy


    SERCOM0->USART.BAUD.reg = SERCOM_USART_BAUD_FRACFP_BAUD(baud) |
                              SERCOM_USART_BAUD_FRACFP_FP(fp);

    SERCOM0->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE; // Enable UART0

    while(SERCOM0->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_ENABLE); // sync busy

    //-- To provide DMA beat

    SERCOM0->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC |  // Receive Complete Interrupt Enable
                                  SERCOM_USART_INTENSET_TXC;   // Transmit Complete Interrupt Enable
    //-- disable UART IRQ in NVIC

    NVIC_DisableIRQ(SERCOM0_IRQn);

    return 0;
}

//-----------------------------------------------------------------------------
// For debug only
//-----------------------------------------------------------------------------
static void uart_putc(char c)
{
    while (!(SERCOM0->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE));
    SERCOM0->USART.DATA.reg = c;
}

//-----------------------------------------------------------------------------
void uart_puts(char *s)
{
    while (*s)
    {
        uart_putc(*s++);
    }
}
//----------------------------------------------------------------------------
unsigned char uart_read_byte(void)
{

    while(!SERCOM0->USART.INTFLAG.bit.RXC);  // Wait for Receive Complete flag

    if(SERCOM0->USART.STATUS.bit.PERR ||      // Check for errors
       SERCOM0->USART.STATUS.bit.FERR ||
          SERCOM0->USART.STATUS.bit.BUFOVF)
    {
        //  uart_drv_error_flag = true;         // Set the error flag
    }

    return((uint8_t)SERCOM0->USART.DATA.reg);  // Return the read data
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------




