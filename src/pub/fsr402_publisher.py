# IMPORTANT: THIS FILE SHOULD RUN ON PICO
# this file should run on pico
# fsr402 is analog signal, and the signal will send to pico
# pico will collect analog signal and transfer to digital signal
# then the digital signal will send through usb to raspberry pi 4b

# FIRST STEP: READ THE FORCE SENSOR DATA ON PICO
from machine import ADC
import time

fsr = ADC(26) # 3.3V --- FSR402 --- GP26 (PICO ADC0) +++ --- 10k RESIST --- GND

def read_force():
    analog_value = fsr.read_u16() # read 0 - 65535 analogue values
    force = int(analog_value / 65535 * 100 )
    return force

# SECOND STEP: SEND THROUGH SERIAL PORT
while True:
    force = read_force()
    print(force)
    time.sleep(0.1)

# THE PRINT INFO WILL OUTPUT ON serial device /ttyACM0
# The raspberry pi 4b can use screen, minicom, or serial lib (python) to receive these outputs

# NEXT STEP:
# (1) write main.py on Pico
# (2) Pico will automatically run main.py once it is connected to Raspberry Pi
# (3) Read Pico's data on raspberry pi using serial 
