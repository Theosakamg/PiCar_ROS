#!/usr/bin/env python3
import RPi.GPIO as GPIO


class SensorDigital(object):
  _channel : int

  def __init__(self, channel = 20):
    self._channel = channel

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self._channel, GPIO.IN) # pull_up_down=GPIO.PUD_UP


class SensorDigitalState(SensorDigital):

  def __init__(self, channel = 20):
    SensorDigital.__init__(self, channel)

  def state(self):
    return GPIO.input(self._channel)


class SensorDigitalCount(SensorDigital):

  _count_low : int
  _count_high : int

  def __init__(self, channel = 20):
    SensorDigital.__init__(self, channel)

    GPIO.add_event_detect(self._channel, GPIO.RISING)
    GPIO.add_event_callback(self._channel, self.__callback)

  def reset():
    self._count_low = 0
    self._count_high = 0

  def __callback (self, channel):
    if (GPIO.input(channel) == GPIO.LOW):
      self._count_low += 1
    else
      self._count_high += 1

  def count_high(self):
    return self._count_high
  
  def count_low(self):
    return self._count_low