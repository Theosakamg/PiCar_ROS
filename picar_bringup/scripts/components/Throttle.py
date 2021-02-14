#!/usr/bin/env python3
'''
**********************************************************************
* Filename    : Servo.py
* Description : Driver module for servo, with PCA9685
*               Inspired by Servo.py from SunFounder <service@sunfounder.com>
* Author      : Mickael Gaillard
* E-mail      : mick.gaillard@gmail.com
* Update      : Mickael Gaillard 2021-02-07    New release
**********************************************************************
'''

from drivers import PCA9685

class Throttle(object):
  '''Throttle driver class'''

  _DEBUG = False
  _DEBUG_INFO = 'DEBUG "Throttle.py":'

  def __init__(self, channel, bus_number=1, address=0x40):
    ''' Init a servo on specific channel, this offset
        Parameters
        ----------
        channel : int
            The pwd channel
        offset : int, optional
            The offset of value
        lock : bool, optional
            limit to min or max value of range no loop.
        bus_number : int, optional
            Bun number of smbus
        address : int, optional
            I2C address
    '''
    if channel < 0 or channel > 16:
      raise ValueError("Servo channel \"{0}\" is not in (0, 15).".format(channel))

    self._channel = channel
    self.__pwm = PCA9685.PWM(bus_number, address)
    self.__pwm.setup()
    self.write(0)

  def _debug_(self, message):
    if self._DEBUG:
      print(self._DEBUG_INFO, message)

  @property
  def driver(self):
    return self.__pwm

  def _pulse_wide(self, value):
    ''' Calculate 12-bit analog value from giving value '''
    pulse_wide = int(self.__pwm.map(value,
                              0,
                              100,
                              0,
                              4095))

    self._debug_("Value: %f°\tPulse : %dus" % (value, pulse_wide))

    return pulse_wide

  def write(self, value):
    pulse_wide = self._pulse_wide(value)
    self.__pwm.write(self._channel, 0, pulse_wide)

  def max(self):
    self.write(100)

  def stop(self):
    self.write(0)

  @property
  def debug(self):
    return self._DEBUG

  @debug.setter
  def debug(self, debug):
    ''' Set if debug information shows '''
    if debug in (True, False):
      self._DEBUG = debug
    else:
      raise ValueError('debug must be "True" (Set debug on) or "False" (Set debug off), not "{0}"'.format(debug))

    if self._DEBUG:
      print(self._DEBUG_INFO, "Set debug on")
    else:
      print(self._DEBUG_INFO, "Set debug off")