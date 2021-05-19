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

class Servo(object):
  '''Servo driver class'''

  _MIN_PULSE_WIDTH = 600
  _MAX_PULSE_WIDTH = 2400
  _MIN_DEGREE_VALUE = 0
  _MAX_DEGREE_VALUE = 180
  _DEFAULT_DEGREE_VALUE = 90

  _DEBUG = False
  _DEBUG_INFO = 'DEBUG "Servo.py":'

  def __init__(self, channel, offset=0, lock=True, bus_number=1, address=0x40):
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
    self._lock = lock
    self.__current_angle = 0
    self._min_degree_hw = self._MIN_DEGREE_VALUE
    self._max_degree_hw = self._MAX_DEGREE_VALUE
    self._min_degree_value = self._MIN_DEGREE_VALUE
    self._max_degree_value = self._MAX_DEGREE_VALUE
    self._dlt_degree_value = self._DEFAULT_DEGREE_VALUE
    self._min_pulse_width = self._MIN_PULSE_WIDTH
    self._max_pulse_width = self._MAX_PULSE_WIDTH

    self.__pwm = PCA9685.PWM(bus_number, address)
    self.__pwm.setup()
    self.offset = offset

  def _debug_(self, message):
    if self._DEBUG:
      print(self._DEBUG_INFO, message)

  def _angle_to_analog(self, angle):
    ''' Calculate 12-bit analog value from giving angle '''
    pulse_wide = self.__pwm.map(angle,
                              self._min_degree_hw,
                              self._max_degree_hw,
                              self._min_pulse_width,
                              self._max_pulse_width)

    analog_value = int(float(pulse_wide) / 1000000 * self.shared_frequency * 4096)
    self._debug_("Angle: %f°\tPulse : %dus\tAnalog_value : %d" % (angle, pulse_wide, analog_value))

    return analog_value

  @property
  def driver(self):
    return self.__pwm

  @property
  def min_degree_value(self):
    return self._min_degree_value

  @min_degree_value.setter
  def min_degree_value(self, value):
    if (value < self.max_degree_value):
      self._min_degree_value = value
      self._debug_('Set min degree value to %f°' % self._min_degree_value)

  @property
  def max_degree_value(self):
    return self._max_degree_value

  @max_degree_value.setter
  def max_degree_value(self, value):
    if (value > self.min_degree_value):
      self._max_degree_value = value
      self._debug_('Set max degree value to %f°' % self._max_degree_value)

  @property
  def dlt_degree_value(self):
    return self._dlt_degree_value

  @dlt_degree_value.setter
  def dlt_degree_value(self, value):
    if (value > self.min_degree_value and value < self.max_degree_value):
      self._dlt_degree_value = value
      self._debug_('Set default degree to %f°' % self._dlt_degree_value)

  @property
  def min_degree_hw(self):
    return self._min_degree_hw

  @min_degree_hw.setter
  def min_degree_hw(self, value):
    if (value < self.max_degree_hw):
      self._min_degree_hw = value
      self._debug_('Set min degree hardware to %f°' % self._min_degree_hw)

  @property
  def max_degree_hw(self):
    return self._max_degree_hw

  @max_degree_hw.setter
  def max_degree_hw(self, value):
    if (value > self.min_degree_hw):
      self._max_degree_hw = value
      self._debug_('Set max degree hardware to %f°' % self._max_degree_hw)

  @property
  def shared_frequency(self):
    return self.__pwm.frequency

  @shared_frequency.setter
  def shared_frequency(self, value):
    self.__pwm.frequency = value
    self._debug_('Set frequency to %d Hz' % self.__pwm.frequency)

  @property
  def offset(self):
    return self._offset

  @offset.setter
  def offset(self, value):
    ''' Set offset for much user-friendly '''
    self._offset = value
    self._debug_('Set offset to %d' % self.offset)

  def min(self):
    self._debug_("Got to minimal value : %f°" % self._min_degree_value)
    self.write(self.min_degree_value)

  def max(self):
    self._debug_("Got to maximal value : %f°" % self._max_degree_value)
    self.write(self.max_degree_value)

  def default(self):
    self._debug_("Got to center value : %f°" % self._dlt_degree_value)
    self.write(self.dlt_degree_value)

  def write(self, angle):
    ''' Turn the servo with giving angle. '''
    if self._lock:
      if angle > self.max_degree_value:
        angle = self.max_degree_value
      if angle < self.min_degree_value:
        angle = self.min_degree_value
    else:
      if angle < self.min_degree_value or angle > self.max_degree_value:
        raise ValueError("Servo \"{0}\" turn angle \"{1}\" is not in (0, 180).".format(
            self._channel, angle))

    # self._debug_('Turn angle = %f°' % angle)
    self.__current_angle = angle
    val = self._angle_to_analog(angle) + self.offset
    self.__pwm.write(self._channel, 0, val)

  @property
  def value(self):
    return self.__current_angle

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

def install():
  all_servo = [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]
  for i in range(16):
    all_servo[i] = Servo(i)
  for servo in all_servo:
    servo.setup()
    servo.write(self.dlt_degree_value)

if __name__ == '__main__':
  import sys
  if len(sys.argv) == 2:
    if sys.argv[1] == "install":
      install()
  else:
    test()
