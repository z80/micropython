# manifest.py for F405_SERGEY_V2

# Always include the standard MicroPython libraries
include("$(MPY_DIR)/ports/stm32/boards/manifest.py")

# Add our custom boot.py and test script
freeze("modules", "boot.py")
freeze("modules", "test_leds.py")

