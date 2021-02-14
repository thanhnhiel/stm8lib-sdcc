export PATH := $(PATH):$(HOME)/sdcc/bin

MCU  = stm8l051f3
ARCH = stm8
DEVICE = STM8L05X_LD_VL

F_CPU   ?= 2000000
TARGET  ?= build/main.ihx

LIBDIR   = ../../STM8L15x_StdPeriph_Driver
LIBSRCDIR   = $(LIBDIR)/src
LIBSRC2DIR   = ../../lib

#SRCS    := $(wildcard *.c $(LIBSRCDIR)/*.c)
#ASRCS   := $(wildcard *.s $(LIBSRCDIR)/*.s)

LIBSRCS := \
    $(LIBDIR)/src/stm8l15x_gpio.c
#    $(LIBDIR)/src/stm8l15x_exti.c
    
LIBOBJS := $(LIBSRCS:$(LIBSRCDIR)/%.c=build/%.rel)

LIB2SRCS := \
    $(LIBSRC2DIR)/i2c.c \
    $(LIBSRC2DIR)/mpr121.c \
    $(LIBSRC2DIR)/uart.c
    
LIB2OBJS := $(LIB2SRCS:$(LIBSRC2DIR)/%.c=build/%.rel)

SRCS:= main.c

OBJS     = $(LIBOBJS) $(LIB2OBJS) $(SRCS:%.c=build/%.rel)

CC       = sdcc
LD       = sdld
AS       = sdasstm8
OBJCOPY  = sdobjcopy
ASFLAGS  = -plosgff
CFLAGS   = -m$(ARCH) -p$(MCU) --std-sdcc11 -D$(DEVICE)
CFLAGS  += -DF_CPU=$(F_CPU)UL -I. -I$(LIBDIR)/inc -I$(LIBSRC2DIR)
CFLAGS  += --stack-auto --noinduction --use-non-free
## Disable lospre (workaround for bug 2673)
#CFLAGS  += --nolospre
## Extra optimization rules - use with care
#CFLAGS  += --peep-file $(LIBDIR)/util/extra.def
LDFLAGS  = -m$(ARCH) -l$(ARCH) --out-fmt-ihx

all: build $(TARGET) size

build: 
	-mkdir -p build
    
$(TARGET): $(OBJS)
	echo $(TARGET)": "$(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@

build/%.rel: %.c
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@
    
build/%.rel: $(LIBDIR)/src/%.c
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

build/%.rel: $(LIBSRC2DIR)/%.c
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@
    
$(TARGET).bin: $(TARGET)
	@$(OBJCOPY) -I ihex --output-target=binary $(TARGET) $(TARGET).bin

build/%.rel: %.s
	$(AS) $(ASFLAGS) $<

size: $(TARGET).bin
	@echo "----------"
	@echo "Image size:"
	@stat -L -c %s $(TARGET).bin

flash: $(TARGET).bin
	sudo stm8flash -c stlinkv2 -p $(MCU) -w $(TARGET).bin

clean:
	cd build && rm -f *.map *.asm *.rel *.ihx *.o *.sym *.lk *.lst *.rst *.cdb *.bin

.PHONY: clean all flash
