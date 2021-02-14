#!/usr/bin/env python3
'''
**********************************************************************
* Filename    : Relay.py
* Description : Driver module for relay (ON/OFF), with PCA9685
* Author      : Mickael Gaillard
* E-mail      : mick.gaillard@gmail.com
* Update      : Mickael Gaillard 2021-02-07    New release
**********************************************************************
'''

from drivers import PCA9685

class Relay(ComponentBase):
  '''Relay driver class'''

  _DEBUG = False
  _DEBUG_INFO = 'DEBUG "Relay.py":'

  def __init__(self, channel, state=True, toggle=True, bus_number=1, address=0x40):
    ''' Init a servo on specific channel, this offset
        Parameters
        ----------
        channel : int
            The pwd channel
        state : bool, optional
            The state of relay
        toogle : bool, optional
            can toggle value
        bus_number : int, optional
            Bun number of smbus
        address : int, optional
            I2C address
    '''
    if channel < 0 or channel > 16:
      raise ValueError("Servo channel \"{0}\" is not in (0, 15).".format(channel))

    self._debug_("Debug on")
    self._channel = channel
    self._toggle = toggle
    self._state = state

    self._pwm = PCA9685.PWM(bus_number, address)
    self._pwm.setup()
    self.write(state)

  def _debug_(self, message):
    if self._DEBUG:
      print(self._DEBUG_INFO, message)

  @property
  def shared_frequency(self):
    return self._pwm.frequency

  @shared_frequency.setter
  def shared_frequency(self, value):
    self._pwm.frequency = value
    self._debug_('Set frequency to %d' % self._pwm.frequency)

  def on(self):
    self._state = True
    self._pwm.write(self._channel, 0, 4096)

  def on(self):
    self._state = False
    self._pwm.write(self._channel, 4096, 0)

  @property
  def debug(self):
    return self._DEBUG

  @debug.setter
  def debug(self, debug):
    ''' Set if debug information shows '''
    if debug in (True, False):
      self._DEBUG = debug
      self._pwm.debug = debug
    else:
      raise ValueError('debug must be "True" (Set debug on) or "False" (Set debug off), not "{0}"'.format(debug))

    if self._DEBUG:
      print(self._DEBUG_INFO, "Set debug on")
    else:
      print(self._DEBUG_INFO, "Set debug off")