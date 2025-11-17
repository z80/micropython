MY_MOD_DIR := $(USERMOD_DIR)

BOSCH_DIR = $(MY_MOD_DIR)/bosch/bmi08

# Add all C files to SRC_USERMOD.
SRC_USERMOD += $(MY_MOD_DIR)/module_bmi08.c
SRC_USERMOD += $(BOSCH_DIR)/bmi08a.c
SRC_USERMOD += $(BOSCH_DIR)/bmi08g.c
SRC_USERMOD += $(BOSCH_DIR)/bmi08xa.c

# We can add our module folder to include paths if needed
# This is not actually needed in this example.
CFLAGS_USERMOD += -I$(BOSCH_DIR)
