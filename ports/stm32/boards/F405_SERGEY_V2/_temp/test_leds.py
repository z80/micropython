import time
from machine import Pin

led1 = Pin("LED1", Pin.OUT)
led2 = Pin("LED2", Pin.OUT)

for i in range(6):
    led1.value(i % 2)
    led2.value((i + 1) % 2)
    time.sleep(0.3)

led1.off()
led2.off()
print("LED blink test complete")

