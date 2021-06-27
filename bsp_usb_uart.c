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
#include "cdcdf_acm.h"
#include "cdcdf_acm_desc.h"
#include "utils_ringbuffer.h"

#include "prj_conf.h"
#include "tn_config.h"
#include "tn.h"

#include "bsp_usb_uart.h"

int usb_hw_init(void);

// USB CDC descriptors

static uint8_t single_desc_bytes[] = 
{
    CDCD_ACM_HS_DESCES_LS_FS
};

/*const ???*/ static struct usbd_descriptors g_single_desc[] =
{
   {single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}
};



static USBCDCUARTINFO g_usb_cdc_uart_info;

//---------------------

static int usb_device_cb_state_changed(usb_cdc_control_signal_t state);
static int usb_device_cb_rx(const uint8_t ep,
                            const enum usb_xfer_code rc,
                            const uint32_t count);
static int usb_device_cb_tx(const uint8_t ep,
                            const enum usb_xfer_code rc,
                            const uint32_t count);

//----------------------------------------------------------------------------
USBCDCUARTINFO * get_usb_cdc_uart_info(void)
{
    return &g_usb_cdc_uart_info;
}
 
//----------------------------------------------------------------------------
static int usb_device_cb_state_changed(usb_cdc_control_signal_t state)
{
    USBCDCUARTINFO * ucui = get_usb_cdc_uart_info();

    if(state.rs232.DTR == DTR_ON)   // Only becomes true if host connected and
    {                               // serial port at host was open

        cdcdf_acm_register_callback(CDCDF_ACM_CB_READ,
                                    (FUNC_PTR)usb_device_cb_rx);

        cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE,
                                    (FUNC_PTR)usb_device_cb_tx);

        // Dummy IN write
        cdcdf_acm_write((uint8_t *)ucui->usb_tx_buf, 0);
        // Arrange next OUT reading
        cdcdf_acm_read((uint8_t *)ucui->usb_rx_buf, USB_RX_BUF_SIZE);
    }

    return ERR_NONE;
}

//----------------------------------------------------------------------------
// OUT ( device rx) we need re-start at the each OUT interrupt callback
//
// Runs inside interrupt
//----------------------------------------------------------------------------
static int usb_device_cb_rx(const uint8_t ep,
                            const enum usb_xfer_code rc,
                            const uint32_t count)
{
    TN_INTSAVE_DATA_INT
    int i;
    USBCDCUARTINFO * ucui =  get_usb_cdc_uart_info();

    for(i = 0; i < count; i++)
    {
        tn_idisable_interrupt();  // Inside interrupt

        ringbuffer_put(&ucui->rbuf_info, ucui->usb_rx_buf[i]);
        
        tn_ienable_interrupt();   // Inside interrupt
    }

    // Arrange the next OUT reading

    cdcdf_acm_read((uint8_t *)ucui->usb_rx_buf, USB_RX_BUF_SIZE);

    return ERR_NONE;
}

//----------------------------------------------------------------------------
static int usb_device_cb_tx(const uint8_t ep,
                            const enum usb_xfer_code rc,
                            const uint32_t count)
{
    USBCDCUARTINFO * ucui = get_usb_cdc_uart_info();

    (void)tn_sem_signal(&ucui->tx_rdy_sem); // Notify uart usb tx func about
                                            // block Tx ending

    return ERR_NONE;
}

//----------------------------------------------------------------------------
int usb_serial_init(void)
{
    int rc;
    USBCDCUARTINFO * ucui = get_usb_cdc_uart_info();

    rc = ringbuffer_init(&ucui->rbuf_info,
                         ucui->ring_buf,
                         USB_RING_BUFFER_SIZE);
    if(rc == ERR_NONE)
    {
        rc = tn_sem_create(&ucui->tx_rdy_sem,
                                           1,  // Start val
                                           1); // Max val
    }
    if(rc == ERR_NONE)
    {
        rc = tn_sem_create(&ucui->tx_str_sem,
                                           1,
                                           1);
    }

    usb_hw_init();

    if(rc == ERR_NONE)
    {
        rc = usbdc_init(ucui->usb_ctrl_buf);  // Initialize the USB stack
    }
    if(rc == ERR_NONE)
    {
        rc = cdcdf_acm_init();                // Initialize USB CDC ACM Function Driver
    }
    if(rc == ERR_NONE)
    {
        rc = usbdc_start(g_single_desc);
    }
    if(rc == ERR_NONE)
    {
        usbdc_attach();
    }

    
    ucui->init_state = 0;  // Not yet active

    return rc;
}

//----------------------------------------------------------------------------
// Inside some periodic task
//----------------------------------------------------------------------------
void usb_uart_active_proc(void)
{
    USBCDCUARTINFO * ucui = get_usb_cdc_uart_info();

    if(ucui->init_state == 0)
    {
        if(cdcdf_acm_is_enabled() != 0)
        {
// typedef void (*FUNC_PTR)(void);
            cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C,
                                        (FUNC_PTR)usb_device_cb_state_changed);
            ucui->init_state = 1;
        }
    }
}

//----------------------------------------------------------------------------
int is_usb_uart_active(void)
{
    int rc = false;
    USBCDCUARTINFO * ucui = get_usb_cdc_uart_info();

    if(ucui->init_state == 1)
    {
        rc = true;
    }
    return rc;
}

//----------------------------------------------------------------------------
int uart_usb_read(uint8_t * buf, int max_len)
{
    TN_INTSAVE_DATA
    int ind = 0;
    int nb;
    int i;
    int rc;
    int rb_bytes;
    
    USBCDCUARTINFO * ucui = get_usb_cdc_uart_info();

    if(buf == NULL || max_len <= 0)
    {
        ind = TERR_WPARAM;
    }
    else
    {  
        if(ucui->init_state != 1)   // UART is not active
        {
            ind = TERR_ILUSE;
        }
        else
        {
            ucui->rx_timeout_cnt = ucui->rx_timeout;

            tn_disable_interrupt();
            
            rb_bytes = ringbuffer_num(&ucui->rbuf_info);
            
            tn_enable_interrupt();

            // MIN(ringbuffer_num(), max_len) - bytes to read
            if(max_len < rb_bytes)
            {
                nb = max_len;
            }
            else
            {
                nb = rb_bytes;
            }
            
            if(nb > 0) // Have some bytes to read
            {
                for(i = 0; i < nb; i++)
                {  
                    tn_disable_interrupt();
                    
                    rc = ringbuffer_get(&ucui->rbuf_info, &buf[i]);
                    
                    tn_enable_interrupt();

                    if(rc != ERR_NONE)
                    {
                        break;
                    }  
                }
                ind = i; 
            }
            else // No bytes to read now
            {
                if(ucui->rx_timeout == TN_NO_WAIT)
                {
                    ind = TERR_TIMEOUT;
                }
                else if(ucui->init_state != 1) // UART was close in between
                {
                    ind = TERR_ILUSE;
                }
                else
                {
                    (void)tn_task_sleep(1); //!< Sleep (here 1 mS)

                    if(ucui->rx_timeout != TN_WAIT_INFINITE)
                    {
                        ucui->rx_timeout_cnt--;
                        if(ucui->rx_timeout_cnt == 0U)
                        {
                            ind = TERR_TIMEOUT;
                        }
                    }
                }
            }
        }
    }     
    
    return ind;
}

//----------------------------------------------------------------------------
int uart_usb_tx_buf(uint8_t * buf, int len)
{
    int rc = -1;
    USBCDCUARTINFO * ucui = get_usb_cdc_uart_info();
    uint8_t * ptr;
    int nbytes;
    int nb;    

    if(ucui->init_state == 1)  // UART is active
    {
        if(len > 0)
        {    
            ptr    = buf;
            nbytes = len;
        
            tn_sem_acquire(&ucui->tx_str_sem, TN_WAIT_INFINITE);
            
            while(nbytes > 0)
            {
                if(nbytes > USB_TX_BUF_SIZE)
                {
                    nb = USB_TX_BUF_SIZE;
                }
                else
                {
                    nb = nbytes;
                }                    
                 
                rc = tn_sem_acquire(&ucui->tx_rdy_sem, TN_NO_WAIT);  
                if(rc == TERR_NO_ERR)
                { 
                    tn_sem_signal(&ucui->tx_rdy_sem);  
                    
                    cdcdf_acm_write(ptr, nb);
                }
                else
                {
                    break;
                }
                
                rc = tn_sem_acquire(&ucui->tx_rdy_sem, 100);  // 100 ms should be OK
                if(rc == TERR_NO_ERR)
                { 
                    nbytes -= nb;
                    ptr    += nb;
                }
                else
                {
                    break;
                }
            }                
            tn_sem_signal(&ucui->tx_str_sem);
        }                   
    }     
    
    return rc;
}

//----------------------------------------------------------------------------
int uart_usb_tx_str(char * str)
{
    int rc = -1;
    int size;
    
    if(str != NULL)
    {    
        size = (int)strlen(str);
        rc = uart_usb_tx_buf((uint8_t *)str, size);
    }     
    
    return rc;
}

//----------------------------------------------------------------------------
int uart_usb_tx_char(int ch)
{
    int rc;
    uint8_t buf[4];
    buf[0] = (uint8_t)ch;
    
    rc = uart_usb_tx_buf(buf, 1);
    return rc;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//---------------------------------------------------------------------------- 
