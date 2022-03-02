#!/usr/bin/env python3

import math

#######################################################
## A PID-regulator class.
##
## Fields:
##  __Kp - proportional gain
##  __Ki - integral gain
##  __Kd - derivative gain
##  __bUseLimits - whether limit are used
##  __limU - output value limit
##  __limV - output value change speed limit
##  __limReached - has a limit been reached
##  __lastErr - previous iteration error value
##  __lastI - previous iteration I-comp value
##  U - output value
##
## Methods:
##  Pid - calculates output value
##  Reset - resets params of the regulator
#######################################################
class CPidControl():
    def __init__(self, kp, ki, kd, bUseLimits = False, limU = 0.0, limV = 0.0):
        self.__Kp = kp
        self.__Kd = kd
        self.__Ki = ki
        self.__bUseLimits = bUseLimits
        self.__limU = limU
        self.__limV = limV
        self.__limReached = False
        self.__lastErr = 0
        self.__I = 0
        self.U = 0

    ## Calculate output value
    ## @param err - current error value
    ## @param dt - delta time (in seconds)
    def Pid(self, err, dt):
        P = self.__Kp * err
        if(not self.__limReached):
            # Do not integrate while a limit is reached
            self.__I += self.__Ki * err * dt
        try:
            D = self.__Kd / dt * (err - self.__lastErr)
        except:
            D = 0
        self.__lastErr = err

        newU = P + self.__I + D
        if(self.__bUseLimits):
            self.__limReached = False
            dU = newU - self.U
            curV = dU / dt
            if(abs(curV) > self.__limV):
                # Speed limit
                dUReq = math.copysign(self.__limV, curV) * dt
                newU = self.U + dUReq
                self.__limReached = True
            if(abs(newU) > self.__limU):
                # Value limit
                newU = math.copysign(self.__limU, newU)
                self.__limReached = True
        self.U = P + self.__I + D

    ## Reset pid params
    def Reset(self):
        self.__limReached = False
        self.__lastErr = 0
        self.__I = 0
        self.U = 0