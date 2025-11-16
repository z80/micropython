MCU_SERIES = f4
CMSIS_MCU = STM32F405xx
AF_FILE = boards/stm32f405_af.csv
LD_FILES = boards/stm32f405.ld boards/common_ifs.ld
TEXT0_ADDR = 0x08000000
TEXT1_ADDR = 0x08020000

# Disable internal flash storage
#MICROPY_HW_ENABLE_INTERNAL_FLASH_STORAGE = 0

# Filesystem: LittleFS v2 only
MICROPY_VFS_FAT  = 1
#MICROPY_VFS_LFS1 = 0
#MICROPY_VFS_LFS2 = 1

USER_C_MODULES = $(TOP)/examples/usercmodule


