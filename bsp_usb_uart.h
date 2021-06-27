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

#ifndef BSP_USB_UART_H_
#define BSP_USB_UART_H_

#ifdef __cplusplus
extern "C"  {
#endif
  
#define  DTR_ON     1   // USB attached and ready to recieve
#define  DTR_OFF    0   // USB may (see DTE) not ready to recieve
  

#define  USB_RING_BUFFER_SIZE  128   // 256 (Must be 2^n size)

#define  USB_TX_BUF_SIZE        64
#define  USB_RX_BUF_SIZE        64
#define  USB_CONTROL_BUF_SIZE   64

typedef struct _USBCDCUARTINFO
{
    uint8_t usb_rx_buf[USB_RX_BUF_SIZE];
    uint8_t usb_tx_buf[USB_TX_BUF_SIZE];
    uint8_t usb_ctrl_buf[USB_CONTROL_BUF_SIZE];

    struct ringbuffer rbuf_info;
    uint8_t ring_buf[USB_RING_BUFFER_SIZE];

    TN_SEM  tx_rdy_sem;
    TN_SEM  tx_str_sem;
    
    unsigned int rx_timeout_cnt;
    unsigned int rx_timeout;

    int init_state;

}USBCDCUARTINFO; 


USBCDCUARTINFO * get_usb_cdc_uart_info(void);


int uart_usb_read(uint8_t * buf, int max_len);
int uart_usb_tx_buf(uint8_t * buf, int len);
int uart_usb_tx_str(char * str);
int uart_usb_tx_char(int ch);

int usb_serial_init(void);
void usb_uart_active_proc(void);
int is_usb_uart_active(void);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* #define BSP_USB_UART_H_ */