#!/usr/bin/python
import numpy as np
from math import sqrt,log10



class DecawaveModel:
  #np.random.seed(0)
  def __init__ (self, x0, y0, sigma_):
    self.x0    = x0
    self.y0    = y0
    self.sigma_ = sigma_

  def getMeasurement(self, x, y):
    distance = sqrt((self.x0 - x)**2 + (self.y0 - y)**2)    
    return distance + np.random.normal(0, self.sigma_)

  def getMeasurementRaw(self, x, y):
    distance = sqrt((self.x0 - x)**2 + (self.y0 - y)**2)    
    return distance

  def x(self):
    return self.x0

  def y(self):
    return self.y0

  def sigma(self):
    return self.sigma_
