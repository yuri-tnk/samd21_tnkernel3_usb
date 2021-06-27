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

#ifndef  PRJ_CONF_H
#define  PRJ_CONF_H

//#if (defined __ICCARM__)
//#include <intrinsics.h>
//#endif 

#define COMPILER_PRAGMA(arg)          _Pragma(#arg) 

/**
 * \brief Set aligned boundary.
 */
#if (defined __GNUC__) || (defined __CC_ARM)
#define COMPILER_ALIGNED(a)        __attribute__((__aligned__(a)))
#elif (defined __ICCARM__)
#define COMPILER_ALIGNED(a)        COMPILER_PRAGMA(data_alignment = a)
#endif

//-- g_usb_app_recv_buffer, g_app_send_buffer
   
#define USB_BUFFER_SIZE     64

uint8_t * get_usb_recv_buffer(void);
uint8_t * get_usb_send_buffer(void);

//-- Use shell history

#define SHELL_USE_HISTORY   1
  
   
#endif  /*  #define  PRJ_CONF_H */


