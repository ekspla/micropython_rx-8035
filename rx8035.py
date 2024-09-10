# (c) 2024 ekspla.
# MIT License.  https://github.com/ekspla/micropython_rx-8035
#
# A preliminary version of code for Seiko Epson's RX-8035SA/LC RTCs. 

import time
from micropython import const

_SLAVE_ADDRESS = const(0x32)

_SEC_REG = _DATETIME_REG = const(0x00)
_MIN_REG = const(0x01)
_HR_REG = const(0x02)
_WEEKDAY_REG = const(0x03)
_DATE_REG = const(0x04)
_MONTH_REG = const(0x05)
_YEAR_REG = const(0x06)
_DIGITAL_OFFSET_REG = const(0x07)
_WEEKLY_ALM_MIN_REG = const(0x08)
_WEEKLY_ALM_HR_REG = const(0x09)
_WEEKLY_ALM_WEKDAY_REG = const(0x0A)
_MONTHLY_ALM_MIN_REG = const(0x0B)
_MONTHLY_ALM_HR_REG = const(0x0C)
_CONTROL1_REG = const(0x0E)
_CONTROL2_REG = const(0x0F)

_minuteS_MASK = const(0x7F)
_HOUR_MASK = const(0x3F)
_WEEKDAY_MASK = const(0x07)
_YEAR_MASK = const(0xff)
_DATE_MASK = const(0x3F)
_MONTH_MASK = const(0x1F)

_24H_MASK = const(0x80)

_MONTHLY_ALM_EN_MASK = const(0x80)
_WEEKLY_ALM_EN_MASK = const(0x40)
_DEBOUNCE_SET_L_MASK = const(0x20)
_EVENT_DETECT_EN_MASK = const(0x10)
_TEST_MASK = const(0x08)
_CT2_MASK = const(0x04)
_CT1_MASK = const(0x02)
_CT0_MASK = const(0x01)

_BANK_TSFG_MASK = const(0x80)
_VDET_MASK = const(0x40)
_XSTP_MASK = const(0x20)
_PON_MASK = const(0x10)
_EVENT_DETECT_FUNC_G_MASK = const(0x08)
_CONST_TIME_INT_FUNC_G_MASK = const(0x04)
_MONTHLY_ALM_FUNC_G_MASK = const(0x02)
_WEEKLY_ALM_FUNC_G_MASK = const(0x01)

class RX8035:
    def __init__(self, i2c, address=_SLAVE_ADDRESS):
        """Initialization needs to be given an initialized I2C port
        """
        self.i2c = i2c
        self.address = address
        self._buffer = bytearray(_CONTROL2_REG - _SEC_REG + 1)
        self._bytebuf = bytearray(1)
        self._mv = memoryview(self._buffer)
        self._mv_datetime = self._mv[_SEC_REG:_YEAR_REG + 1]

        self._DATETIME_MASK = bytes((
            _minuteS_MASK, 
            _minuteS_MASK, 
            _HOUR_MASK, 
            _WEEKDAY_MASK, 
            _DATE_MASK, 
            _MONTH_MASK, 
            _YEAR_MASK))

        # Check RTC status.
        value = self.__read_byte(_CONTROL2_REG)
        if any((
            pon := bool(value & _PON_MASK), 
            xstp := bool(value & _XSTP_MASK), 
            vdet := bool(value & _VDET_MASK), 
            )):
            print(f'RTC status error. PON: {pon}, XSTP: {xstp}, VDET: {vdet}')
            self._mv[_CONTROL1_REG:_CONTROL2_REG + 1] = b'\x00\x00'
            self.__write_bytes(_CONTROL1_REG, self._mv[_CONTROL1_REG:_CONTROL2_REG + 1])

    def __write_byte(self, reg, val):
        self._bytebuf[0] = val & 0xff
        self.__write_bytes(reg, self._bytebuf)

    def __read_byte(self, reg):
        self.__read_bytes(reg, self._bytebuf)
        return self._bytebuf[0]

    def __write_bytes(self, reg, buffer):
        self.i2c.writeto_mem(self.address, (reg << 4) & 0xFF, buffer)

    def __read_bytes(self, reg, buffer):
        self.i2c.readfrom_mem_into(self.address, (reg << 4) & 0xFF, buffer)

    def __bcd2dec(self, bcd):
        return (((bcd & 0xf0) >> 4) * 10 + (bcd & 0x0f))

    def __dec2bcd(self, dec):
        tens, units = divmod(dec, 10)
        return (tens << 4) + units

    def __get_weekday(self, date, month, year):
        if month < 3:
            month += 12
            year -= 1
        weekday = (
            (-1 + date + (13 * month + 8) // 5 + year + year // 4 
            - year // 100 + year // 400)
            % 7)
        return weekday

    def datetime(self):
        """Return a tuple such as (year, month, date, weekday, hours, minutes,
        seconds).
        """
        self.__read_bytes(_DATETIME_REG, self._mv_datetime)

        seconds, minutes, hours, weekday, date, month, year = (
            self.__bcd2dec(a & b) for a, b in zip(
            self._mv_datetime, self._DATETIME_MASK))

        return (year, month, date, weekday, hours, minutes, seconds)

    def write_all(self, seconds=None, minutes=None, hours=None, weekday=None,
                  date=None, month=None, year=None):
        """Direct write un-none value.
        Range: seconds [0,59], minutes [0,59], hours [0,23],
               weekday [0,6], date [1,31], month [1,12], year [0,99].
        """
        if (seconds is None) or seconds < 0 or seconds > 59:
            raise ValueError('Seconds is out of range [0,59].')
        if (minutes is None) or minutes < 0 or minutes > 59:
            raise ValueError('Minutes is out of range [0,59].')
        if (hours is None) or hours < 0 or hours > 23:
            raise ValueError('Hours is out of range [0,23].')
        if (date is None) or date < 1 or date > 31:
            raise ValueError('Date is out of range [1,31].')
        if (month is None) or month < 1 or month > 12:
            raise ValueError('Month is out of range [1,12].')
        if (year is None) or year < 0 or year > 99:
            raise ValueError('Years is out of range [0,99].')
        if weekday is None:
            weekday = self.__get_weekday(date, month, year + 2000)
        elif weekday < 0 or weekday > 6:
            raise ValueError('Day is out of range [0,6].')

        self._buffer[_SEC_REG] = self.__dec2bcd(seconds)
        self._buffer[_MIN_REG] = self.__dec2bcd(minutes)
        self._buffer[_HR_REG] = self.__dec2bcd(hours) | _24H_MASK # 12 hour mode currently not supported.
        self._buffer[_DATE_REG] = self.__dec2bcd(date)
        self._buffer[_WEEKDAY_REG] = self.__dec2bcd(weekday)
        self._buffer[_MONTH_REG] = self.__dec2bcd(month)
        self._buffer[_YEAR_REG] = self.__dec2bcd(year)

        self.__read_byte(_CONTROL2_REG) # Read Control2 register.
        self.__write_bytes(_DATETIME_REG, self._mv_datetime) # Set datetime registers.
        self.__write_byte(_CONTROL2_REG, self._bytebuf[0]  & ~_VDET_MASK) # Clear VDET in Control2 register.

    def set_datetime(self, dt):
        """Input a tuple such as (year, month, date, weekday, hours, minutes,
        seconds).
        """
        self.write_all(dt[5], dt[4], dt[3],
                       dt[6], dt[2], dt[1], dt[0] % 100)

    def write_now(self):
        """Write the current system time to RX-8035
        """
        self.set_datetime(time.localtime())

    def digital_offset(self, value=None):
        """Read/Set clock adjustment, from 63 (-189.10 ppm) to -62 (+189.10 ppm); default is 0 (OFF).
        """
        if value is None:
            value = self._buffer[_DIGITAL_OFFSET_REG] = self.__read_byte(_DIGITAL_OFFSET_REG)
            return -(value ^ 0x7F) -1 if (value & 0x40) else value
        elif -62 <= value <= 63:
            self._buffer[_DIGITAL_OFFSET_REG] = value & 0x7F
            self.__write_byte(_DIGITAL_OFFSET_REG, self._buffer[_DIGITAL_OFFSET_REG])
        else:
            print('Value error.')
