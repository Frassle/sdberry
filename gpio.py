#! /usr/bin/python

import sys
import time
import gpiozero

# pin 3 = output

#                         |SPI            |1 Bit            |4 Bit
# strand 2 == microsd 1   |N/A            |N/A              |SD Serial Data 2
# strand 4 == microsd 2   |SPI Card Select|Card Detection   |SD Serial Data 3
# strand 6 == microsd 3   |MOSI           |Command/Response |Command/Response
# strand 8 == microsd 4   |Power          |Power            |Power
# strand 10 == microsd 5  |SCLK           |Serial Clock     |Serial Clock
# strand 11 == microsd 6  |Ground         |Ground           |Ground
# strand 12 == microsd 7  |MISO           |SD Serial Data 0 |SD Serial Data 0
# strand 14 == microsd 8  |N/A            |N/A              |SD Serial Data 1


with \
    gpiozero.InputDevice(19) as miso, \
    gpiozero.InputDevice(20) as mosi, \
    gpiozero.InputDevice(21) as sckl, \
    gpiozero.InputDevice(26) as cd, \
    gpiozero.OutputDevice(3) as pinout:

    pinout.on()

    while True:
        print("miso:mosi:sckl:cd")
        print("{}   :{}   :{}   :{}".format(
            1 if miso.value else 0,
            1 if mosi.value else 0,
            1 if sckl.value else 0,
            1 if cd.value else 0))
        time.sleep(0.5)
