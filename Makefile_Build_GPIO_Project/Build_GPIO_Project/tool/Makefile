PATH_OUTPUT := ../output
PROJ_NAME := stm32f4_gpio
PRO_DIR := .

INC_DIRS := $(PRO_DIR)/include 
INC_DIRS += $(PRO_DIR)/../driver/gpio/inc

SRC_DIRS := $(PRO_DIR)/source
SRC_DIRS += $(PRO_DIR)/../driver/gpio/src

SRC_FILES	:= $(foreach SRC,$(SRC_DIRS),$(wildcard $(SRC)/*.c))
INC_FILES	:= $(foreach INC,$(INC_DIRS),$(wildcard $(INC)/*.h))
ASM_FILES	:= $(foreach ASM,$(SRC_DIRS),$(wildcard $(ASM)/*.s))
LINKER_FILE	:= ./linker/linker_script_stm32f407vg.ld

COMPILER_DIRS := D:/ToolchainGNUArm/arm-gnu-toolchain-13.2.Rel1-mingw-w64-i686-arm-none-eabi
PREFIX_GCC_COMPILER := arm-none-eabi

CC			:= $(COMPILER_DIRS)/bin/$(PREFIX_GCC_COMPILER)-gcc
ASM			:= $(COMPILER_DIRS)/bin/$(PREFIX_GCC_COMPILER)-as  
LD			:= $(COMPILER_DIRS)/bin/$(PREFIX_GCC_COMPILER)-ld

INC_DIRS_OPT	:= $(foreach INC_DIRS,$(INC_DIRS),-I$(INC_DIRS))
CC_OPT			:= -mcpu=cortex-m4 -c -O0 -g -mfloat-abi=softfp -std=gnu11 -mthumb $(INC_DIRS_OPT)
ASM_OPT			:= -mcpu=cortex-m4 -c -mthumb $(INC_DIRS_OPT)
LD_OPT			:= -T $(LINKER_FILE) -Map $(PATH_OUTPUT)/$(PROJ_NAME).map

OBJ_FILES	:= $(notdir $(SRC_FILES))
OBJ_FILES	:= $(subst .c,.o,$(OBJ_FILES))
PATH_OBJ	:= $(foreach OBJ_FILES,$(OBJ_FILES),$(PATH_OUTPUT)/$(OBJ_FILES))

# OBJ_ASM_FILES	:= $(notdir $(ASM_FILES))
# OBJ_ASM_FILES	:= $(subst .s,.o,$(OBJ_ASM_FILES))
# PATH_OBJ_ASM	:= $(foreach ASM_FILES,$(OBJ_ASM_FILES),$(PATH_OUTPUT)/$(OBJ_ASM_FILES))


# OBJ_FILES += $(OBJ_ASM_FILES)
# PATH_OBJ += $(PATH_OBJ_ASM)

vpath %.c $(SRC_DIRS)
vpath %.h $(INC_DIRS)
vpath %.s $(SRC_DIRS)

#Build program 
# #compile file Asm to .o
# %.o: %.s 
# 	mkdir -p $(PATH_OUTPUT)
# 	$(ASM) $(ASM_OPT) $< -o $(PATH_OBJ_ASM)
# 	@echo "----------------------------------Build Asm to object succesful !---------------------------------------"

%.o: %.c 
	mkdir -p $(PATH_OUTPUT)
	$(CC) $(CC_OPT) -c $< -o $(PATH_OUTPUT)/$@ 

build: $(OBJ_FILES) $(LINKER_FILE)
	$(LD) $(LD_OPT) $(PATH_OBJ) -o $(PATH_OUTPUT)/$(PROJ_NAME).elf
	$(COMPILER_DIRS)/$(PREFIX_GCC_COMPILER)/bin/objcopy.exe -O ihex "$(PATH_OUTPUT)/$(PROJ_NAME).elf" "$(PATH_OUTPUT)/$(PROJ_NAME).hex"
	size $(PATH_OUTPUT)/$(PROJ_NAME).hex
	@echo "----------------------------------Build succesful !---------------------------------------"
run:
	"$(PRO_DIR)/StlinkUtility/ST-LINK Utility/ST-LINK_CLI.exe" -p "$(PATH_OUTPUT)/$(PROJ_NAME).hex" 0x08000000
	"$(PRO_DIR)/StlinkUtility/ST-LINK Utility/ST-LINK_CLI.exe" -rst
read:
	$(COMPILER_DIRS)/$(PREFIX_GCC_COMPILER)/bin/objdump.exe -h $(PATH_OUTPUT)/$(PROJ_NAME).elf


log-%:
	@echo $($(subst log-,,$@))

.PHONY: clean
clean:
	rm -rf $(PATH_OUTPUT)/*



