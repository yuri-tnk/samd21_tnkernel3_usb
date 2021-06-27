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

#include <string.h>
#include "samd21.h"
#include "cdcdf_acm.h"
#include "cdcdf_acm_desc.h"
#include "utils_ringbuffer.h"

#include "prj_conf.h"
#include "tn_config.h"
#include "tn.h"
#include "hal_gpio.h"
#include "prj_def.h"
#include "shell.h"
#include "bsp_usb_uart.h"


/*
    The project is based on the SAMD21 MINI board

  (CPU - Mirochip(Atmel) ATSAMD21G18 Cortex-M0P,  power - from USB,
   32KHz external crystal osc)

  The UART0 uses DMA for Rx & Tx
  The USB is used as the Serial Port CDC

                  IO Port   CPU Pin     Board

UART_TX            PA04       9          A3         SERCOM0/PAD[0]
UART_RX            PA05      10          A4         SERCOM0/PAD[1]

TX_LED             PA27      39         TX_LED
RX_LED             PB03      48         RX_LED

*/


//----------- Tasks ----------------------------------------------------------

//-- The OS ticks task - must

#define  TASK_OS_TICK_PRIORITY        0  /* always highest possible priority */
#define  OS_TICK_TASK_STACK_SIZE    128

static TN_TCB tn_os_tick_task;
COMPILER_ALIGNED(8) static unsigned int tn_os_tick_task_stack[OS_TICK_TASK_STACK_SIZE] TN_ALIGN_ATTR_END;
void tn_os_tick_task_func(void * param);

/*
*/

#define  TASK_UART0_RX_PRIORITY      3
#define  TASK_MAIN_PRIORITY          6
#define  TASK_IO_PRIORITY            8

#define  TASK_UART0_RX_STK_SIZE    192
#define  TASK_MAIN_STK_SIZE        256
#define  TASK_IO_STK_SIZE          192

static TN_TCB  task_uart0_rx;
COMPILER_ALIGNED(8) static unsigned int task_uart0_rx_stack[TASK_UART0_RX_STK_SIZE] TN_ALIGN_ATTR_END;
void task_uart0_rx_func(void * par);

static TN_TCB  task_main;
COMPILER_ALIGNED(8) static unsigned int task_main_stack[TASK_MAIN_STK_SIZE] TN_ALIGN_ATTR_END;
void task_main_func(void * par);

static TN_TCB  task_io;
COMPILER_ALIGNED(8) static unsigned int task_io_stack[TASK_IO_STK_SIZE] TN_ALIGN_ATTR_END;
void task_io_func(void * par);

void idle_hook_func(void * par);

//-------- Semaphores -----------------------

//-------- Queues ---------------------------

#define QUEUE_ACTIONS_SIZE           8
#define QUEUE_ACTIONS_MEM_BUF_SIZE  (DQUEUE_ENTRY_SIZE * QUEUE_ACTIONS_SIZE)
TN_DQUEUE queueShellEvents;
COMPILER_ALIGNED(8) static unsigned char queueShellEventsMem[QUEUE_ACTIONS_MEM_BUF_SIZE];

//------- I/O pins --------------------------

HAL_GPIO_PIN(RX_LED, B, 3);
HAL_GPIO_PIN(TX_LED, A, 27);

//-------------------------------------------

int clock_init(void);
void usb_hw_init(void);

static UARTINFO g_uart0_info;
static BSP_SHELL_DS  g_shell_ds;
static BSP_SHELL_DS * g_p_shell_ds = &g_shell_ds;


#define UART0_RX_BUF_CAPACITY  32
COMPILER_ALIGNED(8) uint8_t g_uart0_rx_buf[UART0_RX_BUF_CAPACITY];

#define UART0_TX_BUF_CAPACITY 64
COMPILER_ALIGNED(8) uint8_t g_uart0_tx_buf[UART0_TX_BUF_CAPACITY];

//------------- USB

COMPILER_ALIGNED(4) uint8_t g_usb_recv_buffer[USB_BUFFER_SIZE];
COMPILER_ALIGNED(4) uint8_t g_usb_send_buffer[USB_BUFFER_SIZE];

//-------------------------------------------------
#define L_RX_BUF_SIZE 32
typedef struct _SASCIIDRV
{
    int idx;
    uint8_t rx_buf[L_RX_BUF_SIZE];

} SASCIIDRV;

SASCIIDRV g_ascii_drv;
int simple_ascii_rx_drv(SASCIIDRV * drv, int ch);

uint32_t random_xor32(void);
int fill_line_buf(char * sbuf, int sbuf_size, int idx);

#define ULINE_BUF_SIZE   128
char g_line_buf[ULINE_BUF_SIZE];

//----------------------------------------------------------------------------
uint8_t * get_usb_recv_buffer(void)
{
    return  &g_usb_recv_buffer[0];
}

//----------------------------------------------------------------------------
uint8_t * get_usb_send_buffer(void)
{
    return  &g_usb_send_buffer[0];
}

//----------------------------------------------------------------------------
UARTINFO * get_uart0_info(void)
{
    return &g_uart0_info;
}

//----------------------------------------------------------------------------
int main()
{
    int rc;

    clock_init();
    bsp_dma_init();

    // Randomize()
    for(rc = 0; rc < 17; rc++)
    {
       (void)random_xor32();
    }

//================
    HAL_GPIO_TX_LED_out();
    HAL_GPIO_TX_LED_set();   // LED - off

    HAL_GPIO_RX_LED_out();
    HAL_GPIO_RX_LED_set();   // LED - off

    sh_init(g_p_shell_ds);
    memset(&g_uart0_info, 0, sizeof(UARTINFO));

    memset(&g_ascii_drv, 0, sizeof(SASCIIDRV));

    (void)SysTick_Config(48000); // 1 ms interrupt

    rc = tn_start_system(NULL,
                        0UL,
                        NULL); // NULL is OK for the project

   // if we are here, something goes wrong

    if(rc != TERR_NO_ERR)
    {
      // Add your own error handler here, (if you are really need it)
        for(;;)
        {
        }
    }

    return 0;
}

//----------------------------------------------------------------------------
int tn_app_init(void)
{
    int rc;

    tn_idle_task_hook_func = NULL;

    bsp_dma_dummy_init();

    rc = bsp_uart0_open(g_uart0_rx_buf,
                        UART0_RX_BUF_CAPACITY,
                        g_uart0_tx_buf,
                        UART0_TX_BUF_CAPACITY,
                        UART_BAUD_115200); // UARTBAUDRATE baud_rate);
    if(rc != TERR_NO_ERR)
    {
        goto err_exit;
    }

    rc = usb_serial_init();
    if(rc != TERR_NO_ERR)
    {
        goto err_exit;
    }

   //--- OS ticks task - must

    tn_os_tick_task.id_task = 0UL;
    rc = tn_task_create(&tn_os_tick_task,          //-- task TCB
                        tn_os_tick_task_func,      //-- task function
                        0,                         //-- task priority
                        &(tn_os_tick_task_stack    //-- task stack first addr in memory
                        [OS_TICK_TASK_STACK_SIZE - 1]),
                        OS_TICK_TASK_STACK_SIZE,   //-- task stack size (in int,not bytes)
                        NULL,                      //-- task function parameter
                        TN_TASK_OS_TICK);          //-- Creation option
    if(rc != TERR_NO_ERR)
    {
        goto err_exit;
    }
    //--- Task Rx UART0

    task_uart0_rx.id_task = 0UL;
    rc = tn_task_create(&task_uart0_rx,               //-- task TCB
                        task_uart0_rx_func,           //-- task function
                        TASK_UART0_RX_PRIORITY,       //-- task priority
                        & (task_uart0_rx_stack        //-- task stack first addr in memory
                           [TASK_UART0_RX_STK_SIZE - 1]),
                        TASK_UART0_RX_STK_SIZE,       //-- task stack size (in int,not bytes)
                        NULL,                         //-- task function parameter
                        TN_TASK_START_ON_CREATION);   //-- Creation option
    if(rc != TERR_NO_ERR)
    {
        goto err_exit;
    }
   //--- Task Main

    task_main.id_task = 0U;
    rc = tn_task_create(&task_main,                 //-- task TCB
                        task_main_func,             //-- task function
                        TASK_MAIN_PRIORITY,         //-- task priority
                        &(task_main_stack           //-- task stack first addr in memory
                        [TASK_MAIN_STK_SIZE-1]),
                        TASK_MAIN_STK_SIZE,         //-- task stack size (in int,not bytes)
                        NULL,                       //-- task function parameter
                        TN_TASK_START_ON_CREATION); //-- Creation option
    if(rc != TERR_NO_ERR)
    {
        goto err_exit;
    }
   //--- Task IO

    task_io.id_task = 0U;
    rc = tn_task_create(&task_io,                       //-- task TCB
                        task_io_func,                   //-- task function
                        TASK_IO_PRIORITY,               //-- task priority
                        &(task_io_stack                 //-- task stack first addr in memory
                        [TASK_IO_STK_SIZE-1]),
                        TASK_IO_STK_SIZE,               //-- task stack size (in int,not bytes)
                        NULL,                           //-- task function parameter
                        TN_TASK_START_ON_CREATION);     //-- Creation option
    if(rc != TERR_NO_ERR)
    {
        goto err_exit;
    }

   //--- Queues

   queueShellEvents.id_dqueue = 0UL;
   rc = tn_dqueue_create(&queueShellEvents,
                         QUEUE_ACTIONS_SIZE,
                         &queueShellEventsMem[0],
                         QUEUE_ACTIONS_MEM_BUF_SIZE);
   if(rc == TERR_NO_ERR)
   {
      goto err_exit;
   }


err_exit:

    return rc;
}

//----------------------------------------------------------------------------
void tn_os_tick_task_func(void * param) // Must
{
    int rc;
    int cnt = 0;
    for(;;)
    {
        rc = tn_sem_acquire(&tn_sys_tick_sem, TN_WAIT_INFINITE);
        if(rc >= 0) // OK
        {
            (void)tn_os_timers_tick_proc();
#if defined USER_TIMERS
            (void)tn_user_timers_tick_proc();
#endif
            (void)bsp_dma_dummy_start();

            cnt++;
            if(cnt%20 == 0) // Each 20 ms
            {
               usb_uart_active_proc();
            }
        }
    }
}

//----------------------------------------------------------------------------
void task_uart0_rx_func(void * par)
{
#define DRV_UART_RX_BUF_SIZE 64
#define RESP_BUF_SIZE (DRV_UART_RX_BUF_SIZE + 16)

    unsigned char rx_buf[DRV_UART_RX_BUF_SIZE];
    char resp_buf[RESP_BUF_SIZE];
    int i;
    int nbytes;
    int rc;

    UARTINFO * ui = get_uart0_info();
    USBCDCUARTINFO * ucui = get_usb_cdc_uart_info();

    // For the blocking reading , rx_timeout is default(TN_WAIT_INFINITE)

    ui->rx_timeout = TN_NO_WAIT;     //-- Set uart0 reading as non-blocking

    ucui->rx_timeout = TN_NO_WAIT;   //-- Set UART USB reading as non-blocking

    for(;;)
    {
        nbytes = uart0_read(&rx_buf[0],
                            DRV_UART_RX_BUF_SIZE);
        if(nbytes > 0)
        {
            for(i = 0; i < nbytes; i++)
            {
                sh_input(&g_p_shell_ds->shell_info, rx_buf[i]);
            }
        }
        else
        {
            nbytes = uart_usb_read(&rx_buf[0],
                                   DRV_UART_RX_BUF_SIZE);
            if(nbytes > 0)
            {
                for(i = 0; i < nbytes; i++)
                {
                    rc = simple_ascii_rx_drv(&g_ascii_drv, rx_buf[i]);
                    if(rc == 0) // Got str, terminated by '\r'
                    {
                        tn_snprintf(resp_buf, RESP_BUF_SIZE, "Got: %s\r\n",
                                    (char *)g_ascii_drv.rx_buf);
                        (void)uart_usb_tx_str(resp_buf);
                    }
                }
            }
            else
            {
                (void)tn_task_sleep(1);
            }
        }
    }
}


//----------------------------------------------------------------------------
void task_main_func(void * par)
{
    int rc;
    unsigned long rx_data;

    for(;;)
    {
        rc = tn_dqueue_receive(&queueShellEvents,
                               (TN_DQUEUE_ELEMENT *)((void*)&rx_data),
                               TN_WAIT_INFINITE);
        if(rc >= 0) // OK
        {
            if(rx_data == (unsigned long)EVT_EXEC_SHELL)
            {
                (void)sh_do_cmd_exec(&g_p_shell_ds->shell_info);
            }
            else
            {
                (void)uart0_tx_str((char*)("Unknown msg.\r\n"));
            }
        }
    }
}

//----------------------------------------------------------------------------
void task_io_func(void * par)
{
#define SBUF_SIZE 32
    unsigned int cnt = 0;
    //char sbuf[SBUF_SIZE];
    int blink_cnt = 0;
   // unsigned int tmp;

    for(;;)
    {
        (void)tn_task_sleep(3); // 3 ms

        cnt++;
        blink_cnt++;

        if(blink_cnt == 4*33) // LED - on
        {
            HAL_GPIO_TX_LED_clr();
        }
        else
        {
            if(blink_cnt == 8*33) // LED - off
            {
                blink_cnt = 0;
                HAL_GPIO_TX_LED_set();
            }
        }

        fill_line_buf(g_line_buf, ULINE_BUF_SIZE, cnt);
        (void)uart_usb_tx_str(g_line_buf);


        //if(cnt % 10U == 0U)  // Each 1 sec
       // {
        //    (void)strcpy(sbuf, "Cnt: ");
        //    tmp = cnt/10U;  // To make MISRA 2012 happy
        //    do_itoa((int)tmp, sbuf + strlen(sbuf), SBUF_SIZE);
        //    (void)strcat(sbuf, "\r\n");

        //    (void)uart0_tx_str(sbuf);
//            (void)uart_usb_tx_str(sbuf);
       // }
    }
}

//----------------------------------------------------------------------------
int simple_ascii_rx_drv(SASCIIDRV * drv, int ch)
{
    int rc = -1;
    switch(ch)
    {
        case '\r':

            drv->rx_buf[drv->idx] = 0;
            drv->idx = 0;
            rc = 0;
            break;

        case '\n':   // Skip
        case '\t':

            break;

        default:

            drv->rx_buf[drv->idx++] = (uint8_t)ch;
            if(drv->idx >= L_RX_BUF_SIZE)
            {
                memset(drv->rx_buf, 0, L_RX_BUF_SIZE);
                drv->idx = 0;
            }
            break;
    }
    return rc;
}


//----------------------------------------------------------------------------
uint32_t random_xor32(void)
{
    static uint32_t seed = 7;  // 100% random seed value

    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;

    return seed;
}

//----------------------------------------------------------------------------
int fill_line_buf(char * sbuf, int sbuf_size, int idx)
{
    int arr[7];
    int tmp;
    int rc = -1;

    if(sbuf != NULL && sbuf_size > 80 && idx >= 0)
    {
        tmp = ((int)random_xor32())%15;
        arr[0] = idx;
        arr[1] = 10  - tmp;
        arr[2] = 560 + tmp;
        if(tmp & 1)
        {
            arr[3] = 15  - tmp;
        }
        else
        {
            arr[3] = -(15 - tmp);
        }

        tmp = ((int)random_xor32())%50;
        arr[4] = 9120 + tmp;
        arr[5] = 633  + tmp;
        tmp = ((int)random_xor32())%200;
        arr[6] = 13800  + tmp;

        tn_snprintf(sbuf, sbuf_size, "%d %d %d %d %d %d %d\r\n",
                   arr[0],
                   arr[1],
                   arr[2],
                   arr[3],
                   arr[4],
                   arr[5],
                   arr[6]);
        rc = 0;
    }
    return rc;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
