#!/usr/bin/python
import numpy as np
from math import sqrt,log10



class RSSIModel:
  #np.random.seed(0)
  def __init__ (self, x0, y0, alpha_, sigma_, L0 = 40.0, d0=1.0):
    self.x0    = x0
    self.y0    = y0
    self.sigma_ = sigma_
    self.alpha_ = alpha_
    self.L0    = L0
    self.d0    = d0

    self.rssi  = float('inf')
   

  def getMeasurementCorr(self, x, y, rand_):
    distance = sqrt((self.x0 - x)**2 + (self.y0 - y)**2)    
    return self.L0 + 10*self.alpha_*log10(distance/self.d0) + rand_ 

  def getMeasurement(self, x, y):
    
    distance = sqrt((self.x0 - x)**2 + (self.y0 - y)**2)    
    return self.L0 + 10*self.alpha_*log10(distance/self.d0) + np.random.normal(0, self.sigma_)

  def getMeasurementRaw(self, x, y):

    distance = sqrt((self.x0 - x)**2 + (self.y0 - y)**2)    
    return self.L0 + 10*self.alpha_*log10(distance/self.d0)       

  def x(self):
    return self.x0

  def y(self):
    return self.y0

  def sigma(self):
    return self.sigma_

  def alpha(self):
    return self.alpha_

  def rssi(self):
    return self.rssi