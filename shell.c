/**
*
*  Copyright (c) 2004, 2021 Yuri Tiomkin
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
#include <stdint.h>
#include <stdbool.h>

#include <samd21.h>

#include "prj_conf.h"
#include "tn_config.h"
#include "tn.h"
#include "shell.h"
#include "prj_def.h"

int sh_get_command_line_args(unsigned char * cmd_line, unsigned char ** argv);
void sh_cmd_end(SHELLINFO * sh_info, int use_CRLF);
int sh_send(SHELLINFO * sh_info, char * str_to_send);
int sh_stcasecmp(char * s1, char * s2);

#ifdef __cplusplus
extern "C"  {
#endif

#if defined( __GNUC__ ) // __CROSSWORKS_ARM)
int strcasecmp(const char *s1, const char *s2);
#endif

#ifdef __cplusplus
}  /* extern "C" */
#endif

#if defined SHELL_USE_HISTORY

static void shell_hist_init(SHELLINFO * sh_info);
static int shell_hs_add_line(SHELLINFO * sh_info, char * line, int line_len);
static int shell_hs_get_line(SHELLINFO * sh_info,
                             SH_HIST_ITEM ** hist_item_addr,
                             int dir);
static void shell_hs_free_memory(SHELLINFO * sh_info, SH_HIST_ITEM * data_ptr);
static SH_HIST_ITEM * shell_hs_get_memory(SHELLINFO * sh_info, int line_len);
static int dque_fifo_read(SH_DQUE * dque, void ** data_ptr);
static int dque_fifo_write(SH_DQUE * dque, void * data_ptr);
static int shell_is_line_already_in_history(SHELLINFO * sh_info,
                                            char * line,
                                            int line_len);  
static void shell_send_hs_line(SHELLINFO * sh_info, int dir);

static MEMINFO g_shell_hs_mi;
static unsigned char g_shell_hs_mi_buf[SH_HIST_BUF_SIZE];

#endif  /* #if defined SHELL_USE_HISTORY */

//----------------------------------------------------------------------------
void sh_init(BSP_SHELL_DS * p_sh)
{
    //-- Shell
    unsigned long tmp;

    (void)memset(p_sh, 0, sizeof(BSP_SHELL_DS));

#if defined SHELL_USE_HISTORY

    shell_hist_init(&p_sh->shell_info);

    tn_alloc_init(&g_shell_hs_mi,
                  g_shell_hs_mi_buf,
                  SH_HIST_BUF_SIZE);

#endif  /* #if defined SHELL_USE_HISTORY */

    p_sh->shell_info.cl_size      = SH_CL_SIZE;
    p_sh->shell_info.cl_buf       = &p_sh->shell_cl_buf[0];
    p_sh->shell_info.argv         = (unsigned char **)((void*)&p_sh->shell_argv[0]);
    p_sh->shell_info.exec_buf     = &p_sh->shell_exec_buf[0]; //!< command line exec buffer

    p_sh->shell_info.num_cmd      = g_shell_cmd_arr_size;

    tmp =  (unsigned long) &g_shell_cmd_arr[0];
    p_sh->shell_info.sh_cmd_arr = (SHELLCMD *)tmp;


    p_sh->shell_info.sh_send_str  = uart_tx_str_func;
    p_sh->shell_info.sh_send_char = uart_tx_char_func;
#if defined SHELL_USE_HISTORY
    p_sh->shell_info.sh_send_buf  = uart_tx_buf_func;
#endif  /* #if defined SHELL_USE_HISTORY */

    (void)memset(p_sh->shell_info.cl_buf, 0, (size_t)p_sh->shell_info.cl_size);

    //-- UART rx drv
    p_sh->sh_uart_drv.buf          = &p_sh->sh_uart_drv_payload_buf[0];
    p_sh->sh_uart_drv.max_buf_size = SHELL_UART_RX_PAYLOAD_BUF_SIZE;
    p_sh->sh_uart_drv.pos          = 0;
}

//----------------------------------------------------------------------------
void sh_input(SHELLINFO * sh_info, unsigned char ch)
{
    static int stage = 0;

    if(sh_info->cmd_run == FALSE)
    {
        if(ch != (unsigned char)CR_SYM && sh_info->cl_pos < sh_info->cl_size) // CR or Buffer is full.
        {
            switch(ch)
            {
                case BS_SYM:
                case DEL_SYM:

                    if(sh_info->cl_pos > 0)
                    {
                        sh_info->cl_pos--;
                        (void)sh_echo(sh_info, BS_SYM);
                        (void)sh_putchar(sh_info, (unsigned char)' ');
                        (void)sh_putchar(sh_info, BS_SYM);

#if defined SHELL_USE_HISTORY
                        sh_info->last_len = sh_info->cl_pos;
#endif
                    }
                    break;

                case 27:    // Esc

                    if(stage == 0)
                    {
                        stage = 1;
                    }
                    break;

                case  91:   // '['

                    if(stage == 1)
                    {
                        stage = 2;
                    }

                    break;

                default:

                    if(stage == 0)
                    {
                        if(sh_info->cl_pos + 1 < sh_info->cl_size)
                        {
                        // Only printable characters.
                            if(ch >= (unsigned char)SPACE_SYM && ch <= (unsigned char)DEL_SYM)
                            {
                                sh_info->cl_buf[sh_info->cl_pos++] = (unsigned char)ch;
                                (void)sh_echo(sh_info, ch);
#if defined SHELL_USE_HISTORY
                                sh_info->last_len = sh_info->cl_pos;
#endif
                            }
                        }
                    }
                    else
                    {
                        if(stage == 2)
                        {
#if defined SHELL_USE_HISTORY
                            if(ch == 'A')
                            {
                                shell_send_hs_line(sh_info, SHELL_HIST_DIR_FW);
                            }
                            else if(ch == 'B')
                            {
                                shell_send_hs_line(sh_info, SHELL_HIST_DIR_BK);
                            }
                            else
                            {
                            } 
#endif
                        }

                        stage = 0;
                    }
                    break;
            }
        }
        else //-- cmd received - to process
        {
            int cl;
            sh_info->cl_buf[sh_info->cl_pos] = (unsigned char)'\0';
            strcpy((char*)sh_info->exec_buf, (char*)sh_info->cl_buf);

#if 0
            (void)memset(sh_info->cl_buf, 0, (size_t)sh_info->cl_size);
#endif
            cl = sh_info->cl_pos;
            sh_info->cl_pos = 0;
            sh_info->cl_buf[sh_info->cl_pos] = (unsigned char)'\0';

            sh_info->sh_send_str((char*)"\r\n");

            sh_info->cmd_run = TRUE;
            if(cl > 0)
            {
#if defined SHELL_USE_HISTORY
                shell_hs_add_line(sh_info, (char *)sh_info->exec_buf, (int)strlen((char*)sh_info->exec_buf));
#endif
            }

            (void)sh_start_cmd_exec(sh_info); // Just signal on
        }
    }
    else // cmd_run == TRUE
    {
        if(ch == (unsigned char)CTRLC_SYM)
        {
            (void)sh_stop_cmd_exec(sh_info); // Just signal on, inside ->set cmd_run = FALSE when actually finished
        }
    }
}

//----------------------------------------------------------------------------
int sh_start_cmd_exec(SHELLINFO * sh_info)
{
    void * data = (unsigned char*)((unsigned int)EVT_EXEC_SHELL);

#if defined TNKERNEL_PORT_MSP430X
    (void)tn_dqueue_send(queueShellEvents, (TN_DQUEUE_ELEMENT)data, TN_NO_WAIT);
#else
    (void)tn_dqueue_send(&queueShellEvents, (TN_DQUEUE_ELEMENT)data, TN_NO_WAIT);
#endif

    return 0;
}
//----------------------------------------------------------------------------
int sh_do_cmd_exec(SHELLINFO * sh_info)
{
    int i;
    int argc;
    SHELLCMD * cmd_ptr;
    int rc = TERR_NO_ERR;

    // Clear argv buffer
    (void)memset(sh_info->argv, 0, (size_t)(sizeof(unsigned char *) * (size_t)SH_MAX_ARG));

    argc = sh_get_command_line_args(sh_info->exec_buf, sh_info->argv);
    if(argc <= 0)
    {
        sh_cmd_end(sh_info, FALSE);
        rc = -1;
    }
    else
    {
        for(i = 0; i < sh_info->num_cmd; i++)
        {
            cmd_ptr = &sh_info->sh_cmd_arr[i];
            if(strcasecmp((char*)((void*)sh_info->argv[0]), cmd_ptr->cmd_name) == 0)
            {
                //-- recognized cmd

                cmd_ptr->exec_cmd(sh_info);
                sh_cmd_end(sh_info, TRUE);

                break;
            }
        }
        if(i == sh_info->num_cmd)  // If here - cmd not found
        {
            (void)sh_send(sh_info, (char*)"\r\nWrong command!\r\n");
            sh_cmd_end(sh_info, TRUE);
            rc = -1;
        }
    }

    return rc;
}

//----------------------------------------------------------------------------
int sh_send(SHELLINFO * sh_info, char * str_to_send)
{
    int rc = -1;

    if(sh_info->sh_send_str != NULL)
    {
        sh_info->sh_send_str(str_to_send);
        rc = TERR_NO_ERR;
    }
    return rc;
}

//----------------------------------------------------------------------------
int sh_prompt(SHELLINFO * sh_info, int send_CR_LF)
{
    int rc = TERR_NO_ERR;

    if(sh_info->sh_send_str == NULL)
    {
        rc = -1;
    }
    else
    {
        if(send_CR_LF != 0)
        {
            sh_info->sh_send_str((char*)"\r\n");
        }
        sh_info->sh_send_str((char*)"SHELL>");
    }

    return rc; //-- OK
}

//----------------------------------------------------------------------------
int sh_stop_cmd_exec(SHELLINFO * sh_info)
{
    sh_info->stop_exec = TRUE;

    return 0;
}

//----------------------------------------------------------------------------
int sh_echo(SHELLINFO * sh_info, unsigned char ch)
{
    int rc = TERR_NO_ERR;

    if(sh_info->sh_send_char == NULL)
    {
        rc = -1;
    }
    else
    {
        sh_info->sh_send_char(ch);
    }
    return rc;
}

//----------------------------------------------------------------------------
int sh_putchar(SHELLINFO * sh_info, unsigned char ch)
{
    int rc = TERR_NO_ERR;

    if(sh_info->sh_send_char == NULL)
    {
        rc = -1;
    }
    else
    {
        sh_info->sh_send_char(ch);
    }
    return rc;
}

//----------------------------------------------------------------------------
void sh_cmd_end(SHELLINFO * sh_info, int use_CRLF)
{
    TN_INTSAVE_DATA

    (void)sh_prompt(sh_info, use_CRLF);

    tn_disable_interrupt();
    sh_info->cmd_run = FALSE;
    tn_enable_interrupt();
}
//----------------------------------------------------------------------------
//  Args with whitespaces does not supported !!!
//----------------------------------------------------------------------------
int sh_get_command_line_args(unsigned char * cmd_line, unsigned char ** argv)
{
    int ch;
    int rc = 0;
    int fExit = FALSE;
    int argc = 0;
    int now_arg = FALSE;
    unsigned char * ptr = cmd_line;
    unsigned char * start_ptr = cmd_line;

    for(ch = 0; ch < SH_MAX_ARG; ch++)
    {
        argv[ch] = NULL;
    }

    for(;;)
    {
        ch = (int) * ptr;
        switch(ch)
        {
            case '\0':
            case ' ':
            case '\t':

                if(now_arg == TRUE)
                {
                    *ptr = (unsigned char)'\0';
                    argv[argc] = start_ptr;
                    argc++;
                    if(argc >= SH_MAX_ARG)
                    {
                        rc = argc;
                        fExit = TRUE;
                    }
                    else
                    {
                        now_arg = FALSE;
                    }
                }

                if(fExit == FALSE)
                {
                    if(ch == (int)'\0')
                    {
                        rc = argc;
                        fExit = TRUE;
                    }
                }

                break;

            default:

                if(now_arg == FALSE)
                {
                    now_arg = TRUE;
                    start_ptr = ptr;
                }

                break;
        }
        if(fExit == TRUE)
        {
            break;
        }
        ptr++;
    }

    return rc;
}
//----------------------------------------------------------------------------
#if 0
int sh_strcasecmp(char * s1, char * s2)
{
    char ch1, ch2;

    for(;;)
    {
        ch1 = *s1++;
        if(ch1 >= 'A' && ch1 <= 'Z')
        {
            ch1 += 0x20;
        }

        ch2 = *s2++;
        if(ch2 >= 'A' && ch2 <= 'Z')
        {
            ch2 += 0x20;
        }

        if(ch1 < ch2)
        {
            return -1;
        }
        if(ch1 > ch2)
        {
            return 1;
        }
        if(ch1 == 0)
        {
            return 0;
        }
    }
}
#endif

//----------------------------------------------------------------------------
// Shell history
//----------------------------------------------------------------------------

#if defined SHELL_USE_HISTORY

//----------------------------------------------------------------------------
void shell_hist_init(SHELLINFO * sh_info)
{
    memset(&sh_info->dque, 0, sizeof(sh_info->dque));
}

//----------------------------------------------------------------------------
SH_HIST_ITEM * shell_hs_get_memory(SHELLINFO * sh_info, int line_len)
{
    SH_HIST_ITEM * ret_val = NULL;
    int block_size = line_len + sizeof(int);

    if(sh_info != NULL)
    {
        ret_val = (SH_HIST_ITEM *)tn_alloc(&g_shell_hs_mi,
                                           (unsigned long)block_size);
    }

    return ret_val;
}

//----------------------------------------------------------------------------
void shell_hs_free_memory(SHELLINFO * sh_info, SH_HIST_ITEM * data_ptr)
{
    SH_HIST_ITEM * hist_item_addr;
    int i;

    if(sh_info != NULL && data_ptr != NULL)
    {
        // Free entry (set to NULL) in the shell history fifo
        for(i = 0; i < SHELL_HIST_MAX_LINES; i++)
        {
            hist_item_addr = (SH_HIST_ITEM *)sh_info->dque.data_fifo[i];
            if(hist_item_addr == data_ptr)
            { 
                sh_info->dque.data_fifo[i] = NULL;
                break;  
            }
        }

        tn_dealloc(&g_shell_hs_mi, (void *) data_ptr);
    }
}

//----------------------------------------------------------------------------
int shell_hs_add_line(SHELLINFO * sh_info, char * line, int line_len)
{
    SH_HIST_ITEM * ptr = NULL;
    SH_HIST_ITEM * data_ptr = NULL;
    int rc;
    if(sh_info == NULL || line == NULL || line_len <= 0)
    {
        rc = TERR_WPARAM;  
    }
    else
    {
        rc = shell_is_line_already_in_history(sh_info, line, line_len);  
        if(rc != 0)     // Line isn't already inside history
        {
            ptr = shell_hs_get_memory(sh_info, line_len);
            if(ptr != NULL)
            {
                ptr->len = line_len;
                memcpy(&ptr->data[0], line, line_len);

                rc = dque_fifo_write(&sh_info->dque, (void *)ptr);
            }

            if((rc != TERR_NO_ERR) ||  // No free entries in queue
                        (ptr == NULL)) // No free memory
            {
                for(;;)
                {
                    rc = dque_fifo_read(&sh_info->dque, (void **) &data_ptr);
                    if(rc == TERR_NO_ERR)
                    {
                        shell_hs_free_memory(sh_info, data_ptr);
//             printf("Aval mem: %lu\n", tn_alloc_get_free_size(&g_shell_hs_mi));

                        if(ptr == NULL)
                        {
                            ptr = shell_hs_get_memory(sh_info, line_len);
//             printf("ptr: %p\n", ptr);

                        }
                    }

                    if(ptr != NULL ||      // Ok
                        rc != TERR_NO_ERR) // Err
                    {
                        break;
                    }
                }

                if(ptr != NULL && rc == TERR_NO_ERR)
                {
                    ptr->len  = line_len;
                    memcpy(&ptr->data[0], line, line_len);

                    rc = dque_fifo_write(&sh_info->dque, (void *)ptr);
                }
                else
                {
                    rc = TERR_OUT_OF_MEM;
                }
            }
        }
    }
    return rc;
}

//----------------------------------------------------------------------------
int shell_hs_get_line(SHELLINFO * sh_info,
                      SH_HIST_ITEM ** hist_item_addr,
                      int dir)
{
    int rc = -1;
    int empty_lines = 0;
    for(;;)
    {
        if(sh_info->dque.data_fifo[sh_info->dque.idx] == NULL) // Empty
        {
            empty_lines++;
        }
        else
        {
            *hist_item_addr = (SH_HIST_ITEM *)sh_info->dque.data_fifo[sh_info->dque.idx];
            rc = 0;
        }

        if(dir == SHELL_HIST_DIR_BK) //BK - from freshest to latest
        {
            sh_info->dque.idx++;
            if(sh_info->dque.idx >= SHELL_HIST_MAX_LINES)
            {
                sh_info->dque.idx = 0;
            }
        }
        else
        {
            sh_info->dque.idx--;
            if(sh_info->dque.idx < 0)
            {
                sh_info->dque.idx = SHELL_HIST_MAX_LINES - 1;
            }
        }

        if(rc == 0 || empty_lines >= SHELL_HIST_MAX_LINES)
        {
            break;
        }
    }

    return rc;
}

//----------------------------------------------------------------------------
int shell_is_line_already_in_history(SHELLINFO * sh_info,
                                     char * line,
                                     int line_len)  
{
    int i;
    int rc = -1;

    SH_HIST_ITEM * hist_item_addr;

    for(i = 0; i < SHELL_HIST_MAX_LINES; i++)
    {
        hist_item_addr = (SH_HIST_ITEM *)sh_info->dque.data_fifo[i];
        if(hist_item_addr != NULL)
        { 
            if(line_len == hist_item_addr->len)
            {
                if(memcmp(hist_item_addr->data, line, line_len) == 0)
                {
                    rc = 0;
                    break;   
                }
            } 
        }
    }

    return rc;
}

//----------------------------------------------------------------------------
void shell_send_hs_line(SHELLINFO * sh_info, int dir)
{
    int rc;
    int i;
    int j;
    int idx = 0;
    SH_HIST_ITEM * hs_entry;

    if(sh_info != NULL)
    {
        rc = shell_hs_get_line(sh_info,
                               &hs_entry,
                               dir);
        if(rc == 0)
        {
            if(sh_info->cl_pos > 0)
            {
                // use 'sh_info->cl_buf' as temp buf for 'BS,WS,BS' sequence per
                // each sym to erase on terminal
                for(i = 0; i < sh_info->last_len; i++)
                { 
                    for(j = 0; j < 3; j++)
                    { 
                        if(j == 1)
                        {                            
                            sh_info->cl_buf[idx] = 0x20; // White Space
                        }
                        else // j== 2 || j == 0
                        {
                            sh_info->cl_buf[idx] = BS_SYM;  
                        }
                        
                        idx++;
                        if(idx >= SH_CL_SIZE)
                        { 
                            sh_info->sh_send_buf(sh_info->cl_buf, idx);
                            idx = 0;
                        }
                    }
                }
                
                if(idx > 0) 
                { 
                    sh_info->sh_send_buf(sh_info->cl_buf, idx);
                }
            }

            memcpy(sh_info->cl_buf, &hs_entry->data[0], hs_entry->len);

            sh_info->cl_buf[hs_entry->len] = '\0';

            sh_info->cl_pos   = hs_entry->len;
            sh_info->last_len = hs_entry->len;

            sh_info->sh_send_str((char*)sh_info->cl_buf);
        }
    }
}

//----------------------------------------------------------------------------
int  dque_fifo_write(SH_DQUE * dque, void * data_ptr)
{
    int rc = TERR_NO_ERR;

    if((dque->tail_cnt == 0 && dque->head_cnt == SHELL_HIST_MAX_LINES - 1)
            || dque->head_cnt == dque->tail_cnt-1)
    {
        rc = TERR_OVERFLOW;  //--  full
    }
    else //-- wr data
    {
        dque->data_fifo[dque->head_cnt] = data_ptr;

        dque->head_cnt++;
        if(dque->head_cnt >= SHELL_HIST_MAX_LINES)
        {
            dque->head_cnt = 0;
        }
    }
    return rc;
}

//----------------------------------------------------------------------------
int dque_fifo_read(SH_DQUE * dque, void ** data_ptr)
{
    int rc = TERR_NO_ERR;

    if(dque->tail_cnt == dque->head_cnt)
    {
        rc = TERR_UNDERFLOW; //-- empty
    }
    else //-- rd data
    {
        *data_ptr = dque->data_fifo[dque->tail_cnt];
        dque->tail_cnt++;
        if(dque->tail_cnt >= SHELL_HIST_MAX_LINES)
        {
            dque->tail_cnt = 0;
        }
    }
    return rc;
}

#endif  /* #if defined SHELL_USE_HISTORY */

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
