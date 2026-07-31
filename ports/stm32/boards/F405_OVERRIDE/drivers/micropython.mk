BOSCH_DIR = $(USERMOD_DIR)/bosch/bmi08

# Add all C files to SRC_USERMOD.
SRC_USERMOD += $(USERMOD_DIR)/module_bmi08.c
SRC_USERMOD += $(BOSCH_DIR)/bmi08a.c
SRC_USERMOD += $(BOSCH_DIR)/bmi08g.c
SRC_USERMOD += $(BOSCH_DIR)/bmi08xa.c

# We can add our module folder to include paths if needed
# This is not actually needed in this example.
CFLAGS_USERMOD += -I$(BOSCH_DIR)



# Defaults (override from command line or mpconfigboard.mk)
BOARD_ENABLE_BMI08 ?= 1
BOARD_ENABLE_NRF24 ?= 1

ifeq ($(BOARD_ENABLE_BMI08),1)
BOSCH_DIR = $(USERMOD_DIR)/bosch/bmi08
SRC_USERMOD += $(USERMOD_DIR)/module_bmi08.c
SRC_USERMOD += $(BOSCH_DIR)/bmi08a.c
SRC_USERMOD += $(BOSCH_DIR)/bmi08g.c
SRC_USERMOD += $(BOSCH_DIR)/bmi08xa.c
CFLAGS_USERMOD += -I$(BOSCH_DIR) -DMODULE_BMI08_ENABLED=1
endif

ifeq ($(BOARD_ENABLE_NRF24),1)
SRC_USERMOD += $(USERMOD_DIR)/module_nrf24.c
# SRC_USERMOD += $(USERMOD_DIR)/nrf24_core.c   # if split
CFLAGS_USERMOD += -I$(USERMOD_DIR)/nrf24 -DMODULE_NRF24_ENABLED=1
endif
