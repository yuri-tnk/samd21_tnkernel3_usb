# ============================================================================
#
# Makefile for 'rtos_usb_g' project (GCC compiler)
#
# Author: Yuri Tiomkin
#
# ============================================================================


FORMAT   = ihex
IMAGE    = rtos_usb_g
CPU		 = cortex-m0plus

ROOTPATH = /home/yurit

PRJ_DIR         = /media/yurit/Samsung_T5/my_github/samd21_tnkernel3_usb
TOOLSROOT       = $(ROOTPATH)/Tools/gcc-arm-none-eabi-10-2020-q4-major

TOOLS           = $(TOOLSROOT)/bin/arm-none-eabi-
OBJ_DIR         = $(PRJ_DIR)/obj
LSTPATH         = $(PRJ_DIR)/lst

# includes dirs

INCLUDES = -I $(PRJ_DIR) \
           -I $(PRJ_DIR)/include \
           -I $(PRJ_DIR)/os \
           -I $(PRJ_DIR)/usb_asf4 \
           -I $(PRJ_DIR)/usb_asf4/device \
           -I $(PRJ_DIR)/usb_asf4/hal/include \
           -I $(PRJ_DIR)/usb_asf4/hal/utils/include \
           -I $(PRJ_DIR)/usb_asf4/hri \
           -I $(PRJ_DIR)/usb_asf4/class/cdc \
           -I $(PRJ_DIR)/usb_asf4/class/cdc/device

# source dirs

OS_DIR    = $(PRJ_DIR)/os
OS_DIR1   = $(PRJ_DIR)/os/CortexM0_M7
ASF4_DIR  = $(PRJ_DIR)/usb_asf4
ASF4_DIR1 = $(PRJ_DIR)/usb_asf4/device
ASF4_DIR2 = $(PRJ_DIR)/usb_asf4/hal/src
ASF4_DIR3 = $(PRJ_DIR)/usb_asf4/hpl/usb
ASF4_DIR4 = $(PRJ_DIR)/usb_asf4/hal/utils/src
ASF4_DIR5 = $(PRJ_DIR)/usb_asf4/class/cdc/device

LDSCRIPT  = $(PRJ_DIR)/samd21g18.ld

VPATH = $(PRJ_DIR) $(OS_DIR) $(OS_DIR1) $(ASF4_DIR)\
        $(ASF4_DIR1) $(ASF4_DIR2) $(ASF4_DIR3) $(ASF4_DIR4) $(ASF4_DIR5)

#_____________________________________________________________________________
#
# List of objects
#

OBJECTS = \
  $(OBJ_DIR)/tn_port_CortexM0_M7_asm.o\
  $(OBJ_DIR)/tn_port_CortexM0_M7.o\
  $(OBJ_DIR)/tn.o\
  $(OBJ_DIR)/tn_alloc.o\
  $(OBJ_DIR)/tn_dqueue.o\
  $(OBJ_DIR)/tn_event.o\
  $(OBJ_DIR)/tn_mailbox.o\
  $(OBJ_DIR)/tn_mem.o\
  $(OBJ_DIR)/tn_mutex.o\
  $(OBJ_DIR)/tn_sem.o\
  $(OBJ_DIR)/tn_sys.o\
  $(OBJ_DIR)/tn_task.o\
  $(OBJ_DIR)/tn_timer.o\
  $(OBJ_DIR)/tn_utils.o\
  $(OBJ_DIR)/cdcdf_acm.o\
  $(OBJ_DIR)/usbdc.o\
  $(OBJ_DIR)/hal_atomic.o\
  $(OBJ_DIR)/hal_usb_device.o\
  $(OBJ_DIR)/utils_assert.o\
  $(OBJ_DIR)/utils_event.o\
  $(OBJ_DIR)/utils_list.o\
  $(OBJ_DIR)/utils_ringbuffer.o\
  $(OBJ_DIR)/hpl_usb.o\
  $(OBJ_DIR)/usb_protocol.o\
  $(OBJ_DIR)/bsp_dma.o\
  $(OBJ_DIR)/bsp_hw.o\
  $(OBJ_DIR)/bsp_uart0.o\
  $(OBJ_DIR)/bsp_usb_uart.o\
  $(OBJ_DIR)/main.o\
  $(OBJ_DIR)/prj_shell_func.o\
  $(OBJ_DIR)/shell.o\
  $(OBJ_DIR)/startup_samd21g18_gcc.o\
  $(OBJ_DIR)/tn_sprintf.o\
  $(OBJ_DIR)/tn_user.o

DEP_FILE =  $(OBJ_DIR)/$(notdir $(basename $@).d)

# create 'obj' directory, if not exists

SHELL := /bin/bash
REQUIRED_DIRS = $(OBJ_DIR) $(LSTPATH)
_MKDIRS := $(shell for d in $(REQUIRED_DIRS); \
             do                               \
               [[ -d $$d ]] || mkdir -p $$d;  \
             done)
#_____________________________________________________________________________
#
#  Tools, tools options(flags)
#

CC      = $(TOOLS)gcc
AS      = $(TOOLS)as
CP      = $(TOOLS)cpp
CPP     = $(TOOLS)g++
LD      = $(TOOLS)ld
OBJCOPY = $(TOOLS)objcopy
OBJDUMP = $(TOOLS)objdump
SIZE    = $(TOOLS)size
NM      = $(TOOLS)nm
REMOVE  = rm
COPY    = copy


#-- Assembler flags

ASMOPTS = -mcpu=$(CPU) -mthumb --gdwarf-2 -mthumb-interwork\
         -D __SAMD21G18A__ -D DONT_USE_CMSIS_INIT -D F_CPU=48000000
CPPOPTS =
ASMLST = -alhms=$(LSTPATH)/$(notdir $(basename $@).lst)
# ASMLST1 =  -fverbose-asm -Wa,-adhln=$(LSTPATH)/$(notdir $(basename $@).lst)
ASMLST1 = -Wa,-adhln=$(LSTPATH)/$(notdir $(basename $@).lst)

#-- Compiler  flags

CCOPTS  = $(INCLUDES) -c -mcpu=cortex-m0plus -g -mthumb -mfloat-abi=soft\
         -Wall -Os -fdata-sections -ffunction-sections \
         -D __SAMD21G18A__ -D DONT_USE_CMSIS_INIT -D F_CPU=48000000
#-flto

#-- Linker flags

LDFLAGS = -T$(LDSCRIPT) -mcpu=cortex-m0plus -mthumb \
           -Wl,-Map=$(OBJ_DIR)/$(IMAGE).map -nostartfiles -Wl,--gc-sections\
           -D __SAMD21G18A__ -D DONT_USE_CMSIS_INIT -D F_CPU=48000000

#-Wl,--gc-sections
#-flto

#-- Build & Linking ---

$(PRJPATH)/$(IMAGE): $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) --output $(OBJ_DIR)/$(notdir $@)
	$(OBJCOPY) -O$(FORMAT)    $(OBJ_DIR)/$(IMAGE)  $(OBJ_DIR)/$(IMAGE).hex
	$(SIZE) -A $(OBJ_DIR)/$(IMAGE)
	$(OBJDUMP) -d -S $(OBJ_DIR)/tn_alloc.o  > $(LSTPATH)/tn_alloc.lst
#$(OBJDUMP) -d -S $(OBJ_DIR)/tn_port_CortexM0_M7_asm.o > $(LSTPATH)/tn_port_CortexM0_M7_asm.lst

#--- Compiling

$(OBJ_DIR)/%.o: %.S
	$(CC) $< $(CCOPTS) -MD -MF $(DEP_FILE) -o $@

$(OBJ_DIR)/%.o: %.c
	$(CC) $< $(CCOPTS) -MD -MF $(DEP_FILE) -o $@

# ============================================================================

.PHONY:   clean
clean:

#$(REMOVE) -f $(LSTPATH)/*.lst
	   $(REMOVE) -f $(OBJ_DIR)/*.d
	   $(REMOVE) -f $(OBJ_DIR)/*.o

include  $(wildcard  $(OBJ_DIR)/$(notdir $(basename *.*).d))