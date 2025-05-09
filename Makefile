
TARGET = owleye

OPT = -Os

# тут нужно указать путь до компилятора
# ВЕРСИЯ КОМПИЛЯТОРА ВАЖНА
GCC_PATH = D:/SAPR/xpack-riscv-none-embed-gcc-10.2.0-1.1-win32-x64/xpack-riscv-none-embed-gcc-10.2.0-1.1/bin
# если собирать на windows, то тут нужно указать путь до openocd
OCD_PATH = ./libs/openocd/bin/

SRC_DIR      = src
HAL_DIR      = HAL
FreeRTOS_DIR = FreeRTOS
BUILD_DIR    = build
LDSCRIPTS_PATH = scripts

# LDSCRIPT = $(LDSCRIPTS_PATH)/ram.ld
LDSCRIPT   = $(LDSCRIPTS_PATH)/spifi.ld
# LDSCRIPT   = $(LDSCRIPTS_PATH)/eeprom.ld


C_SOURCES = $(wildcard $(SRC_DIR)/*.c) \
            $(wildcard $(HAL_DIR)/core/Source/*.c) \
            $(wildcard $(HAL_DIR)/peripherals/Source/*.c) \
            $(wildcard $(FreeRTOS_DIR)/*.c) \
            $(wildcard $(FreeRTOS_DIR)/portable/MemMang/*.c) \
            $(wildcard $(FreeRTOS_DIR)/portable/GCC/RISC-V/*.c) \
						$(SRC_DIR)/drivers/BMP280/mik32_qmc5883l.c \
						$(SRC_DIR)/drivers/QMC5883L/mik32_bmp280.c \
						$(SRC_DIR)/drivers/ICM42688P/mik32_icm42688P.c \
						$(SRC_DIR)/MSP/mik32_msp.c \
						$(SRC_DIR)/Filters/mik32_filters.c \
						$(SRC_DIR)/PID/mik32_pid_bf.c \
						$(SRC_DIR)/Pilot/mik32_pilot.c \
						$(SRC_DIR)/Mixers/mik32_mix_bf.c \
						$(SRC_DIR)/drivers/Motors/mik32_dshot.c \

CPP_SOURCES = $(wildcard $(SRC_DIR)/*.cpp)

ASM_SOURCES = \
		runtime/crt0.S \
        $(FreeRTOS_DIR)/portable/GCC/RISC-V/portASM.S

C_INCLUDES = -I$(SRC_DIR) \
             -I$(SRC_DIR)/core \
             -I$(SRC_DIR)/periphery \
             -I$(HAL_DIR)/peripherals/Include \
             -I$(HAL_DIR)/core/Include \
             -I$(FreeRTOS_DIR) \
             -I$(FreeRTOS_DIR)/include \
             -I$(FreeRTOS_DIR)/portable/MemMang \
             -I$(FreeRTOS_DIR)/portable/GCC/RISC-V \
             -I$(FreeRTOS_DIR)/portable/Common \
             -I$(FreeRTOS_DIR)/portable/GCC/RISC-V/chip_specific_extensions/SCRx \
							-I$(SRC_DIR)/drivers/BMP280 \
							-I$(SRC_DIR)/drivers/QMC5883L \
							-I$(SRC_DIR)/drivers/ICM42688P \
							-I$(SRC_DIR)/MSP \
							-I$(SRC_DIR)/Filters \
							-I$(SRC_DIR)/PID \
							-I$(SRC_DIR)/Pilot \
							-I$(SRC_DIR)/Mixers \
							-I$(SRC_DIR)/drivers/Motors \

PREFIX = riscv-none-embed-

ifeq ($(OS),Windows_NT) 
# Если Windows, то будет добавляться расширение .exe
    EXE_EXT := .exe
else
    EXE_EXT :=
endif

#Убрал ковычки
CC   = $(GCC_PATH)/$(PREFIX)gcc$(EXE_EXT)
CCPP = $(GCC_PATH)/$(PREFIX)g++$(EXE_EXT)
AS   = $(GCC_PATH)/$(PREFIX)gcc$(EXE_EXT) -x assembler-with-cpp
CP   = $(GCC_PATH)/$(PREFIX)objcopy$(EXE_EXT)
SZ   = $(GCC_PATH)/$(PREFIX)size$(EXE_EXT)

HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S 
 
CPU = -march=rv32imc -mabi=ilp32
# CPU = -march=rv32i_zicsr_zifencei -mabi=ilp32 -mstrict-align

MCU = $(CPU) -MD -fstrict-volatile-bitfields -fno-strict-aliasing -fno-common -fno-builtin-printf 
#Убрал кавычки
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -MMD -MP -MF$(@:%.o=%.d)
#Убрал кавычки
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -MMD -MP -MF $(@:%.o=%.d)


LDFLAGS = $(MCU) --specs=nano.specs -lnosys -lgcc -mcmodel=medlow $(LIBDIR) $(LIBS) -nostartfiles -ffreestanding $(OPT) -L $(LDSCRIPTS_PATH) -Wl,-Bstatic,-T,$(LDSCRIPT),-Map,$(BUILD_DIR)/$(TARGET).map,--print-memory-usage
LDFLAGS += -lstdc++


all: $(BUILD_DIR)/$(TARGET).elf $(TARGET).hex $(TARGET).bin


OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | make_folder
	$(CC) -c $(CFLAGS) -g -std=gnu17 -DSUPPORT_CPLUSPLUS -DMIK32V2 -MT$(@) -c $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | make_folder
	$(CCPP) -c $(CFLAGS) -g -std=gnu++17 -DSUPPORT_CPLUSPLUS -DMIK32V2 -MT$(@) -c $< -o $@

$(BUILD_DIR)/%.o: %.S Makefile | make_folder
	$(AS) -c $(CFLAGS) -g -DMIK32V2 -DportasmHANDLE_INTERRUPT=scr_systick_handler -MT$(@) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile | make_folder
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

%.hex: $(BUILD_DIR)/%.elf | make_folder
	$(HEX) $< $@

%.bin: $(BUILD_DIR)/%.elf | make_folder
	$(BIN) $< $@
	
make_folder:
	@mkdir $(BUILD_DIR)

rebuild:
	make clean
	make all

clean:
	-rm -fR $(BUILD_DIR)
	rm $(TARGET).hex $(TARGET).bin


MIK32_UPLOADER_DIR  = ./libs/mik32-uploader/
ELBEAR_UPLOADER_DIR = ./libs/elbear_uploader/

# linux
OCD = $(OCD_PATH)openocd$(EXE_EXT)

OPENOCD_CFG        = ./libs/Util/m-link.cfg
OPENOCD_TARGET_CFG = ./libs/Util/mik32.cfg


flash: all
	python $(MIK32_UPLOADER_DIR)/mik32_upload.py $(TARGET).hex --run-openocd --openocd-exec=$(OCD) --openocd-interface=$(OPENOCD_CFG) --openocd-target=$(OPENOCD_TARGET_CFG)

usb_upload: build
	$(ELBEAR_UPLOADER_DIR)/elbear_uploader$(EXE_EXT) $(TARGET).hex --com=COM5

usb_upload_e: build
	$(ELBEAR_UPLOADER_DIR)/elbear_uploader$(EXE_EXT) $(TARGET).hex --com=COM3 --full-erase


upload:
	$(ELBEAR_UPLOADER_DIR)elbear_uploader$(EXE_EXT) $(TARGET).hex --com=COM5
	#$(OCD) -f $(OPENOCD_CFG) -f $(OPENOCD_TARGET_CFG) -c program D:/VUZD/MKMakeProjects/MIK32Prjct/$(TARGET).elf verify reset exit
	#$(OCD) -f $(OPENOCD_CFG) -f $(OPENOCD_TARGET_CFG) -c init; reset halt
        
full_usb_upload:
	make clean
	make all
	make upload
