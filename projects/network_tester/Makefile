####################################################################
# Makefile                                                         #
####################################################################

.SUFFIXES:				# ignore builtin rules
.PHONY: all debug release clean

####################################################################
# Definitions                                                      #
####################################################################
# MAKEFLAGS+=-j
DEVICE = EFM32GG232F1024
PROJECTNAME = network_tester
# TARGET_FREQ = 32000000   #in Hz

ROOT_DIR = ../..
MODULE_DIR = $(ROOT_DIR)/modules

OBJ_DIR = build
EXE_DIR = exe
LST_DIR = lst

####################################################################
# Definitions of toolchain.                                        #
# You might need to do changes to match your system setup          #
####################################################################

# Change path to the tools according to your system configuration
# DO NOT add trailing whitespace chars, they do matter !
#WINDOWSCS  ?= GNU Tools ARM Embedded\4.7 2012q4
WINDOWSCS  ?= GNU Tools ARM Embedded\4.8 2014q3
LINUXCS    ?= /opt/gcc-arm-none-eabi-4_8-2014q3

RMDIRS     := rm -rf
RMFILES    := rm -rf
ALLFILES   := /*.*
NULLDEVICE := /dev/null
SHELLNAMES := $(ComSpec)$(COMSPEC)

# Try autodetecting the environment
ifeq ($(SHELLNAMES),)
  # Assume we are making on a Linux platform
  TOOLDIR := $(LINUXCS)
else ifeq ($(shell hostname), rluna-PC)
  TOOLDIR := C:/arm/4.8.3
  NULLDEVICE := NUL
else ifeq ($(shell hostname), QuantumSagasV)
  TOOLDIR := C:/GNU_Tools_ARM_Embedded/4.9_2015q2
  NULLDEVICE := NUL
else
  QUOTE :="
  ifneq ($(COMSPEC),)
    # Assume we are making on a mingw/msys/cygwin platform running on Windows
    # This is a convenient place to override TOOLDIR, DO NOT add trailing
    # whitespace chars, they do matter !
    TOOLDIR := $(PROGRAMFILES)/$(WINDOWSCS)
    ifeq ($(findstring cygdrive,$(shell set)),)
      # We were not on a cygwin platform
      NULLDEVICE := NUL
    endif
  else
    # Assume we are making on a Windows platform
    # This is a convenient place to override TOOLDIR, DO NOT add trailing
    # whitespace chars, they do matter !
    SHELL      := $(SHELLNAMES)
    #TOOLDIR    := $(ProgramFiles)/$(WINDOWSCS)
    TOOLDIR    := $(ProgramFiles(x86))/$(WINDOWSCS)
    RMDIRS     := rd /s /q
    RMFILES    := del /s /q
    ALLFILES   := \*.*
    NULLDEVICE := NUL
  endif
endif

# Create directories and do a clean which is compatible with parallell make
$(shell mkdir $(OBJ_DIR)>$(NULLDEVICE) 2>&1)
$(shell mkdir $(EXE_DIR)>$(NULLDEVICE) 2>&1)
$(shell mkdir $(LST_DIR)>$(NULLDEVICE) 2>&1)
ifeq (clean,$(findstring clean, $(MAKECMDGOALS)))
  ifneq ($(filter $(MAKECMDGOALS),all debug release),)
    $(shell $(RMFILES) $(OBJ_DIR)$(ALLFILES)>$(NULLDEVICE) 2>&1)
    $(shell $(RMFILES) $(EXE_DIR)$(ALLFILES)>$(NULLDEVICE) 2>&1)
    $(shell $(RMFILES) $(LST_DIR)$(ALLFILES)>$(NULLDEVICE) 2>&1)
  endif
endif

CC      = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-gcc$(QUOTE)
LD      = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-ld$(QUOTE)
AR      = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-ar$(QUOTE)
OBJCOPY = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-objcopy$(QUOTE)
DUMP    = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-objdump$(QUOTE)
SIZE    = $(QUOTE)$(TOOLDIR)/bin/arm-none-eabi-size$(QUOTE)

####################################################################
# Flags                                                            #
####################################################################

# -MMD : Don't generate dependencies on system header files.
# -MP  : Add phony targets, useful when a h-file is removed from a project.
# -MF  : Specify a file to write the dependencies to.
DEPFLAGS = -MMD -MP -MF $(@:.o=.d)

#
# Add -Wa,-ahld=$(LST_DIR)/$(@F:.o=.lst) to CFLAGS to produce assembly list files
#
override CFLAGS += -D$(DEVICE) -Wall -Wextra -mcpu=cortex-m3 -mthumb \
-mfix-cortex-m3-ldrd -ffunction-sections \
-fdata-sections -fomit-frame-pointer \
$(DEPFLAGS)

override ASMFLAGS += -x assembler-with-cpp -D$(DEVICE) -Wall -Wextra -mcpu=cortex-m3 -mthumb

#
# NOTE: The -Wl,--gc-sections flag may interfere with debugging using gdb.
#
override LDFLAGS += -Xlinker -Map=$(LST_DIR)/$(PROJECTNAME).map -mcpu=cortex-m3 \
-mthumb \
 -Wl,--gc-sections --specs=nano.specs

LIBS = -Wl,--start-group -lgcc -lc -lnosys -lm -Wl,--end-group

INCLUDEPATHS += \
-Iinc \
-Iinc/bsp \
-I../common/inc \
-I$(MODULE_DIR)/CMSIS/Include \
-I$(MODULE_DIR)/EFM32GG/Include \
-I$(MODULE_DIR)/emlib/inc \
-I$(MODULE_DIR)/efm_utils/inc \
-I$(MODULE_DIR)/ifc_lib/inc \
-I$(MODULE_DIR)/FreeRTOS_v8_2_0/inc \
-I$(MODULE_DIR)/FreeRTOS_v8_2_0/src/portable/GCC/ARM_CM3

CTAGS_INCLUDEPATHS = $(subst -I,,$(INCLUDEPATHS))

####################################################################
# Files                                                            #
####################################################################
C_SRC +=  \
src/debug_print.c \
src/main.c \
src/bsp/bsp.c \
src/bsp/bsp_rtc.c \
src/bsp/bsp_timer.c \
src/bsp/bsp_uart.c \
src/bsp/bsp_io.c \
src/bsp/bsp_watchdog.c \
src/bsp/bsp_i2c.c \
src/idle_task.c \
src/tick_hook.c \
src/lcd_nhd.c \
src/ui_task.c \
src/gps_task.c \
src/sensor_task.c \
src/bme280_driver.c \
src/tsl4531_driver.c \
src/supervisor.c \
src/osp.c \
src/bsp_trace.c\
src/byte_fifo.c\
$(MODULE_DIR)/EFM32GG/Source/system_efm32gg.c \
$(MODULE_DIR)/emlib/src/em_aes.c \
$(MODULE_DIR)/emlib/src/em_assert.c \
$(MODULE_DIR)/emlib/src/em_cmu.c \
$(MODULE_DIR)/emlib/src/em_rmu.c \
$(MODULE_DIR)/emlib/src/em_dma.c \
$(MODULE_DIR)/emlib/src/em_emu.c \
$(MODULE_DIR)/emlib/src/em_int.c \
$(MODULE_DIR)/emlib/src/em_gpio.c \
$(MODULE_DIR)/emlib/src/em_msc.c \
$(MODULE_DIR)/emlib/src/em_rtc.c \
$(MODULE_DIR)/emlib/src/em_system.c \
$(MODULE_DIR)/emlib/src/em_timer.c \
$(MODULE_DIR)/emlib/src/em_usart.c \
$(MODULE_DIR)/emlib/src/em_wdog.c \
$(MODULE_DIR)/emlib/src/em_i2c.c \
$(MODULE_DIR)/emlib/src/em_dac.c \
$(MODULE_DIR)/efm_utils/src/eeprom_emulation.c \
$(MODULE_DIR)/efm_utils/src/efm_temperature.c \
$(MODULE_DIR)/efm_utils/src/gpiointerrupt.c \
$(MODULE_DIR)/efm_utils/src/sleep.c \
$(MODULE_DIR)/ifc_lib/src/ll_ifc.c \
$(MODULE_DIR)/ifc_lib/src/ll_ifc_no_mac.c \
$(MODULE_DIR)/ifc_lib/src/ll_ifc_symphony.c \
$(MODULE_DIR)/ifc_lib/src/ifc_struct_defs.c \
$(MODULE_DIR)/FreeRTOS_v8_2_0/src/croutine.c \
$(MODULE_DIR)/FreeRTOS_v8_2_0/src/list.c \
$(MODULE_DIR)/FreeRTOS_v8_2_0/src/queue.c \
$(MODULE_DIR)/FreeRTOS_v8_2_0/src/tasks.c \
$(MODULE_DIR)/FreeRTOS_v8_2_0/src/timers.c \
$(MODULE_DIR)/FreeRTOS_v8_2_0/src/event_groups.c \
$(MODULE_DIR)/FreeRTOS_v8_2_0/src/portable/MemMang/heap_1.c \
$(MODULE_DIR)/FreeRTOS_v8_2_0/src/portable/GCC/ARM_CM3/port.c \

s_SRC +=

S_SRC +=  \
$(MODULE_DIR)/EFM32GG/Source/GCC/startup_efm32gg.S

####################################################################
# Rules                                                            #
####################################################################

C_FILES = $(notdir $(C_SRC) )
S_FILES = $(notdir $(S_SRC) $(s_SRC) )
#make list of source paths, sort also removes duplicates
C_PATHS = $(sort $(dir $(C_SRC) ) )
S_PATHS = $(sort $(dir $(S_SRC) $(s_SRC) ) )

C_OBJS = $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.o))
S_OBJS = $(if $(S_SRC), $(addprefix $(OBJ_DIR)/, $(S_FILES:.S=.o)))
s_OBJS = $(if $(s_SRC), $(addprefix $(OBJ_DIR)/, $(S_FILES:.s=.o)))
C_DEPS = $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.d))
OBJS = $(C_OBJS) $(S_OBJS) $(s_OBJS)

vpath %.c $(C_PATHS)
vpath %.s $(S_PATHS)
vpath %.S $(S_PATHS)

# Default build is debug build
all:      debug

debug:    CFLAGS += -DDEBUG -DDEBUG_EFM -DLL_EMBEDDED -DUSE_FREERTOS -O0 -g
debug:    LDFLAGS += -T$(MODULE_DIR)/EFM32GG/Source/GCC/efm32gg.ld
debug:    $(EXE_DIR)/$(PROJECTNAME).bin

release:  CFLAGS += -DDEBUG -DLL_EMBEDDED -DUSE_FREERTOS -O0 -g
release:  LDFLAGS += -T$(MODULE_DIR)/EFM32GG/Source/GCC/efm32gg_w_bootloader.ld
release:  $(EXE_DIR)/$(PROJECTNAME).bin

# Create objects from C SRC files
$(OBJ_DIR)/%.o: %.c
	@echo "Building file: $<"
	$(CC) $(CFLAGS) $(INCLUDEPATHS) -c -o $@ $<

# Assemble .s/.S files
$(OBJ_DIR)/%.o: %.s
	@echo "Assembling $<"
	$(CC) $(ASMFLAGS) $(INCLUDEPATHS) -c -o $@ $<

$(OBJ_DIR)/%.o: %.S
	@echo "Assembling $<"
	$(CC) $(ASMFLAGS) $(INCLUDEPATHS) -c -o $@ $<

# Link
$(EXE_DIR)/$(PROJECTNAME).elf: $(OBJS)
	@echo "Linking target: $@"
	$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $(EXE_DIR)/$(PROJECTNAME).elf

# Create binary file
$(EXE_DIR)/$(PROJECTNAME).bin: $(EXE_DIR)/$(PROJECTNAME).elf
	@echo "Creating binary file"
	$(OBJCOPY) -O binary $(EXE_DIR)/$(PROJECTNAME).elf $(EXE_DIR)/$(PROJECTNAME).bin
	$(SIZE) $(EXE_DIR)/$(PROJECTNAME).elf
# Create HEX file
	@echo "Creating HEX file"
	$(OBJCOPY) -O ihex $(EXE_DIR)/$(PROJECTNAME).elf $(EXE_DIR)/$(PROJECTNAME).hex
# Uncomment next line to produce assembly listing of entire program
#	$(DUMP) -h -S -C $(EXE_DIR)/$(PROJECTNAME).elf>$(LST_DIR)/$(PROJECTNAME)out.lst

clean:
ifeq ($(filter $(MAKECMDGOALS),all debug release),)
	$(RMDIRS) $(OBJ_DIR) $(LST_DIR) $(EXE_DIR)
endif

# include auto-generated dependency files (explicit rules)
ifneq (clean,$(findstring clean, $(MAKECMDGOALS)))
-include $(C_DEPS)
endif

ctags:
	ctags --c-kinds=+p -R $(C_PATHS) $(S_PATHS) $(CTAGS_INCLUDEPATHS)

download:
	@echo "Downloading and verify $(EXE_DIR)/$(PROJECTNAME).bin ..."
	@echo -e "\nh\nloadbin $(EXE_DIR)/$(PROJECTNAME).bin 0x0\nverifybin $(EXE_DIR)/$(PROJECTNAME).bin 0x0\nh\nr\ng\nSWOView $(TARGET_FREQ)\n" > commands_jlink_download
	@JLinkExe -device $(DEVICE) -CommanderScript commands_jlink_download

.PHONY: lint plotmem
lint:
	@echo "Generating files for Lint..."
	@touch empty.c
	@$(CC) $(LINT_CFLAGS) -E -dM empty.c > lint/AUTO_lint_cmac.h
	@echo $(INCLUDEPATHS) > lint/AUTO_project_includes.lnt
	@rm empty.c
	@echo $(C_SRC) > lint/AUTO_c_src.tmp
	@echo "Running lint..."
	@python ./lint/lin.py
plotmem:
	python ../../../util/plot_map_file_ram.py -f $(LST_DIR)/$(PROJECTNAME).map

