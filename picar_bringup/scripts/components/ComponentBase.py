#!/usr/bin/env python3
'''
**********************************************************************
* Filename    : ComponentBase.py
* Description : Driver module base with PCA9685
* Author      : Mickael Gaillard
* E-mail      : mick.gaillard@gmail.com
* Update      : Mickael Gaillard 2021-02-07    New release
**********************************************************************
'''

from abc import ABC, abstractmethod
from drivers import PCA9685

class ComponentBase(ABC):
  '''ComponentBase driver class'''

  @property
  def shared_frequency(self):
    return self._pwm.frequency

  @shared_frequency.setter
  def shared_frequency(self, value):
    self._pwm.frequency = value
    self._debug_('Set frequency to %d' % self._pwm.frequency)

# TODO