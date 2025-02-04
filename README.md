# micropython_rx-8035
SEP. 2024 ekspla

A MicroPython I2C Driver for Seiko Epson's 5-ppm RTC, RX-8035SA/LC.

## Basic Usage (see docstrings in the code for details.)
``` Python
>>> import machine
>>> i2c = machine.I2C(0)
>>> print([hex(x) for x in i2c.scan()])
['0x32']
>>> import rx8035
>>> rtc = rx8035.RX8035(i2c)
RTC status error. PON: True, XSTP: True, VDET: True    # RTC status is checked at initialization stage. 
>>> rtc.digital_offset(-2)                             # Digital offset may be adjusted.
>>> rtc.write_now()                                    # Set rtc.
>>> rtc.datetime()                                     # Read rtc.
(24, 10, 8, 1, 7, 46, 21)                              # (year, month, date, weekday, hours, minutes, seconds)
>>>
```