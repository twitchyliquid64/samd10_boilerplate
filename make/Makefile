##############################################################################
BUILD = build
BIN = fw

##############################################################################
.PHONY: all directory clean size

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size
OBJDUMP = arm-none-eabi-objdump

CFLAGS += -W -Wall --std=gnu99 -Os
CFLAGS += -fno-diagnostics-show-caret
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -mcpu=cortex-m0plus -mthumb
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d

LDFLAGS += -mcpu=cortex-m0plus -mthumb
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--script=../samd10c13.ld

INCLUDES += \
  -I../include \
  -I..

SRCS += \
  ../main.c \
	../busyloop.c \
	../rtc.c \
  ../startup_samd10.c

DEFINES += \
  -D__SAMD10C13A__ \
  -DDONT_USE_CMSIS_INIT \
  -DF_CPU=20000000

CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin size

$(BUILD)/$(BIN).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

directory:
	@mkdir -p $(BUILD)

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(BUILD)

#Prints the disassembly.
disasm: $(BUILD)/$(BIN).elf
	$(OBJDUMP) -d $(BUILD)/$(BIN).elf

# run 'load' to flash the board.
debug:  $(BUILD)/$(BIN).elf
	arm-none-eabi-gdb -iex "target extended-remote localhost:3333" $(BUILD)/$(BIN).elf


flash:  all
	openocd -f ../openocd.cfg -c "init" -c "flash banks" -c "reset halt" -c "flash write_bank 0 $(BUILD)/$(BIN).bin 0" -c "reset" -c "shutdown"



-include $(wildcard $(BUILD)/*.d)
