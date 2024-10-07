# (c) 2024 ekspla.
# MIT License.  https://github.com/ekspla/micropython_rx-8035
#
# A Micropython library to use with Seiko Epson's RX-8035SA/LC RTCs. 
#
# Limitation: while functions (methods) related to registers in Bank0 are mostly
# supported, Bank1 registers such as timestamps and monthly/yearly alarm are not.  

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
_WEEKLY_ALM_MIN_REG = const(0x08) # Use these with /INTRB.
_WEEKLY_ALM_HR_REG = const(0x09)
_WEEKLY_ALM_WEEKDAY_REG = const(0x0A)
_MONTHLY_ALM_MIN_REG = const(0x0B) # Use these with /INTRA.
_MONTHLY_ALM_HR_REG = const(0x0C)
_USER_RAM_REG = const(0x0D)
_CONTROL1_REG = _CONTROL_REGS = const(0x0E)
_CONTROL2_REG = const(0x0F)

_minuteS_MASK = const(0x7F)
_HOUR_MASK = const(0x3F)
_WEEKDAY_MASK = const(0x07)
_YEAR_MASK = const(0xff)
_DATE_MASK = const(0x3F)
_MONTH_MASK = const(0x1F)

_24H_MASK = const(0x80)

# Control1 masks
_WEEKLY_ALM_EN_MASK = const(0x80) # Use this with /INTRB.
_MONTHLY_ALM_EN_MASK = const(0x40) # Use this with /INTRA.
_DEBOUNCE_SET_LEN_MASK = const(0x20)
_EVENT_DETECT_EN_MASK = const(0x10)
_TEST_MASK = const(0x08)
_CT2_MASK = const(0x04) # Use these with /INTRB.
_CT1_MASK = const(0x02)
_CT0_MASK = const(0x01)

# Control2 masks
_BANK_TSFG_MASK = const(0x80)
_VDET_MASK = const(0x40)
_XSTP_MASK = const(0x20)
_PON_MASK = const(0x10)
_EVENT_DETECT_FUNC_G_MASK = const(0x08)
_CONST_TIME_INT_FUNC_G_MASK = const(0x04) # Use this with /INTRB.
_WEEKLY_ALM_FUNC_G_MASK = const(0x02) # Use this with /INTRB.
_MONTHLY_ALM_FUNC_G_MASK = const(0x01) # Use this with /INTRA.

## Use this with /INTRB.
CONST_TIME_INT_MONTH = _CT2_MASK | _CT1_MASK | _CT0_MASK # Can be cleared by Control2 with _CONST_TIME_INT_FUNC_G_MASK.
CONST_TIME_INT_HOUR = _CT2_MASK | _CT1_MASK
CONST_TIME_INT_MIN = _CT2_MASK | _CT0_MASK
CONST_TIME_INT_SEC = _CT2_MASK
CONST_TIME_INT_1HZ = _CT1_MASK | _CT0_MASK # Pulsed out with duty 50%.
CONST_TIME_INT_2HZ = _CT1_MASK # Pulse out with duty 50%.
SUN = const(0x01)
MON = const(0x02)
TUE = const(0x04)
WED = const(0x08)
THU = const(0x10)
FRI = const(0x20)
SAT = const(0x40)

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
        self._mv_controls = self._mv[_CONTROL1_REG:_CONTROL2_REG + 1]

        self._DATETIME_MASK = bytes((
            _minuteS_MASK, 
            _minuteS_MASK, 
            _HOUR_MASK, 
            _WEEKDAY_MASK, 
            _DATE_MASK, 
            _MONTH_MASK, 
            _YEAR_MASK))

        # Check RTC status.
        self.init_error = False
        value = self.__read_byte(_CONTROL2_REG)
        if any((
            pon := bool(value & _PON_MASK), 
            xstp := bool(value & _XSTP_MASK), 
            vdet := bool(value & _VDET_MASK), 
            )):
            self.init_error = True
            print(f'RTC status error. PON: {pon}, XSTP: {xstp}, VDET: {vdet}')
            self._mv_controls[:] = b'\x00\x00'
            self.__write_bytes(_CONTROL_REGS, self._mv_controls)

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

    def const_time_int_pps(self, mode=None):
        """Set/Reset 1 Hz or 2 Hz PPS signal on /INTRB. 

           Specify either of mode=CONST_TIME_INT_1HZ or mode=CONST_TIME_INT_2HZ to set,
           otherwise reset.
        """
        self._buffer[_CONTROL1_REG] = self.__read_byte(_CONTROL1_REG) & ~(_CT2_MASK|_CT1_MASK|_CT0_MASK)
        if mode in (CONST_TIME_INT_1HZ, CONST_TIME_INT_2HZ):
            self._buffer[_CONTROL1_REG] |= mode
        self.__write_byte(_CONTROL1_REG, self._buffer[_CONTROL1_REG])

    def daily_alarm(self, hours=None, minutes=None):
        """Set/Reset daily alarm on /INTRA.
        
           Specify hours & minutes arguments to set, otherwise reset.
        """
        self.__read_bytes(_CONTROL_REGS, self._mv_controls)
        if all((hours is None, minutes is None)):
            self._buffer[_MONTHLY_ALM_HR_REG] = self._buffer[_MONTHLY_ALM_MIN_REG] = 0
            self._buffer[_CONTROL1_REG] &= ~_MONTHLY_ALM_EN_MASK
        else:
            if minutes is None: minutes = 0 
            if hours is None: hours = 0 
            if minutes < 0 or minutes > 59:
                raise ValueError('Minutes is out of range [0,59].')
            self._buffer[_MONTHLY_ALM_MIN_REG] = self.__dec2bcd(minutes) & _minuteS_MASK
            if hours < 0 or hours > 23:
                raise ValueError('Hours is out of range [0,23].')
            self._buffer[_MONTHLY_ALM_HR_REG] = self.__dec2bcd(hours) & _HOUR_MASK
            self._buffer[_CONTROL1_REG] |= _MONTHLY_ALM_EN_MASK

        self._buffer[_CONTROL2_REG] &= ~_MONTHLY_ALM_FUNC_G_MASK
        self.__write_bytes(_CONTROL_REGS, self._mv_controls)
        self.__write_bytes(_MONTHLY_ALM_MIN_REG, self._mv[_MONTHLY_ALM_MIN_REG:_MONTHLY_ALM_HR_REG + 1])

    def restart_daily_alarm(self):
        """Restart daily alarm without changing the parameters.

           If you want to stop daily alarm, use daily_alarm().
        """
        self.__read_bytes(_CONTROL_REGS, self._mv_controls)
        self._buffer[_CONTROL1_REG] |= _MONTHLY_ALM_EN_MASK
        self._buffer[_CONTROL2_REG] &= ~_MONTHLY_ALM_FUNC_G_MASK
        self.__write_bytes(_CONTROL_REGS, self._mv_controls)

    def weekly_alarm(self, hours=None, minutes=None, weekdays=None):
        """Set/Reset weekly alarm on /INTRB.
        
           Specify hours, minutes and weekdays arguments to set, otherwise reset.
           Specify weekdays asfollows:
               weekdays=MON|TUE|WED|THU|FRI
        """
        self.__read_bytes(_CONTROL_REGS, self._mv_controls)
        if all((hours is None, minutes is None, weekdays is None)):
            self._buffer[_WEEKLY_ALM_HR_REG] = self._buffer[_WEEKLY_ALM_MIN_REG] = 0
            self._buffer[_CONTROL1_REG] &= ~_WEEKLY_ALM_EN_MASK
        else:
            if minutes is None: minutes = 0 
            elif minutes < 0 or minutes > 59:
                raise ValueError('Minutes is out of range [0,59].')
            self._buffer[_WEEKLY_ALM_MIN_REG] = self.__dec2bcd(minutes) & _minuteS_MASK
            if hours is None: hours = 0 
            elif hours < 0 or hours > 23:
                raise ValueError('Hours is out of range [0,23].')
            self._buffer[_WEEKLY_ALM_HR_REG] = self.__dec2bcd(hours) & _HOUR_MASK
            if weekdays:
                self._buffer[_WEEKLY_ALM_WEEKDAY_REG] = weekdays & 0x7F
            self._buffer[_CONTROL1_REG] |= _WEEKLY_ALM_EN_MASK

        self._buffer[_CONTROL2_REG] &= ~_WEEKLY_ALM_FUNC_G_MASK
        self.__write_bytes(_CONTROL_REGS, self._mv_controls)
        self.__write_bytes(_WEEKLY_ALM_MIN_REG, self._mv[_WEEKLY_ALM_MIN_REG:_WEEKLY_ALM_WEEKDAY_REG + 1])

    def restart_weekly_alarm(self):
        """Restart weekly alarm without changing the parameters.

           If you want to stop daily alarm, use weekly_alarm().
        """
        self.__read_bytes(_CONTROL_REGS, self._mv_controls)
        self._buffer[_CONTROL1_REG] |= _WEEKLY_ALM_EN_MASK
        self._buffer[_CONTROL2_REG] &= ~_WEEKLY_ALM_FUNC_G_MASK
        self.__write_bytes(_CONTROL_REGS, self._mv_controls)
