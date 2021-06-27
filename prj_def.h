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


#ifndef  PRJ_DEF_H_
#define  PRJ_DEF_H_

//================ DMA

typedef enum
{
    DMAC_CHANNEL_0 = 0,
    DMAC_CHANNEL_1 = 1,

} DMAC_CHANNEL;

typedef enum
{
    DMAC_TRANSFER_EVENT_COMPLETE, // Data was transferred successfully.
    DMAC_TRANSFER_EVENT_ERROR     // Error while processing the request

} DMAC_TRANSFER_EVENT;

typedef uint32_t DMAC_CHANNEL_CONFIG;
typedef void (*DMAC_CHANNEL_CALLBACK) (DMAC_TRANSFER_EVENT event, uintptr_t contextHandle);

int bsp_dma_init(void);
int bsp_dma_uart0_rx_init(const void * dstAddr,
                          int num_items,
                          int item_size);
int bsp_dma_uart0_tx_init(const void * srcAddr,
                          int num_items,
                          int item_size,
                          const DMAC_CHANNEL_CALLBACK eventHandler);
void bsp_dma_uart0_tx_ch_disable(void);
int bsp_dma_uart0_get_rx_count(void);
void bsp_dma_interrupt_handler(void);
int bsp_dma_uart0_get_btcnt(void);

int bsp_dma_dummy_init(void);
int bsp_dma_dummy_start(void);

//================  UART

typedef struct _UARTINFO
{
    TN_SEM tx_str_sem;
    TN_SEM tx_rdy_sem;
    unsigned int rx_buf_items;
    uint8_t * rx_buf;
    unsigned int tx_buf_items;
    uint8_t * tx_buf;
    unsigned int rx_timeout_cnt;
    unsigned int rx_timeout;
    unsigned int rx_tail;
    int state;

} UARTINFO;

typedef enum _UARTBAUDRATE
{
    UART_BAUD_9600   = 9600,
    UART_BAUD_19200  = 19200,
    UART_BAUD_38400  = 38400,
    UART_BAUD_115200 = 115200,
} UARTBAUDRATE;

UARTINFO * get_uart0_info(void);

int bsp_uart0_open(uint8_t * rx_buf,
                   unsigned int rx_buf_items,
                   uint8_t * tx_buf,
                   unsigned int tx_buf_items,
                   UARTBAUDRATE baud_rate);
int bsp_uart0_transmit(uint8_t * data,
                       int data_size);
int uart0_tx_buf(uint8_t * str, int len);
int uart0_tx_str(char * str);
void uart0_tx_char(unsigned char ch);
int uart0_read(unsigned char * buf,
               unsigned int max_len);

void uart_puts(char *s);
unsigned char uart_read_byte(void);

int tn_snprintf( char *outStr, int maxLen, const char *fmt, ... );

void do_itoa(int val, char * buf, int buf_len);

#define uart_tx_str_func     uart0_tx_str
#define uart_tx_char_func    uart0_tx_char
#define uart_tx_buf_func     uart0_tx_buf

#endif  /* #define PRJ_DEF_H_ */
