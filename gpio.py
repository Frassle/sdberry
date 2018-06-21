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
    gpiozero.InputDevice(21) as sclk, \
    gpiozero.InputDevice(20) as pinin, \
    gpiozero.OutputDevice(3) as pinout:

    pinout.on()

    high = False

    while True:
     #  if int(time.time()) % 2 == 0:
     #      if not pinout.value:
     #          print("on")
     #          pinout.on()
     #  else:
     #      if pinout.value:
     #          print("off")
     #          pinout.off()

     #  if not high and pinin.value:
     #      # gone high
     #      print("Low : " + str(time.time() - edge))
     #      edge = time.time()
     #      high = True
     #  elif high and not pinin.value:
     #      print("High : " + str(time.time() - edge))
     #      edge = time.time()
     #      high = False
        if not high and sclk.value:
            high = True
            if pinin.value:
                sys.stdout.write("1")
                sys.stdout.flush()
            else:
                sys.stdout.write("0")
                sys.stdout.flush()
        elif high and not sclk.value:
            high = False

     #  time.sleep(0.1)
