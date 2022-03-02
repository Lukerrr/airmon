#######################################################
## A two-pole filter class.
##
## Fields:
##  __T1 - the first time constant value
##  __T2 - the second time constant value
##  __dU - first-order derivative of an output
##  __ddU - second-order derivative of an output
##  __uPrev1 - the previous output value
##  __uPrev2 - the previous-previous output value
##
## Methods:
##  Filter - calculates output value
##  Reset - resets params of the filter
#######################################################
class CFilter:
    def __init__(self, T1, T2):
        self.__T1 = T1
        self.__T2 = T2
        self.Reset()
    
    ## Calculate output value
    ## @param x - input value
    ## @param dt - delta time (in seconds)
    ##
    ## @return filtered output value
    def Filter(self, x, dt):
        T1 = self.__T1 * self.__T2
        T2 = self.__T1 + self.__T2
        u = (dt * dt * x + (2 * T1 + dt * T2) * self.__uPrev1 - T1 * self.__uPrev2) / (T1 + T2 * dt + dt * dt)
        self.__uPrev2 = self.__uPrev1
        self.__uPrev1 = u
        return u

    ## Resets filter params
    def Reset(self):
        self.__dU = 0.0
        self.__ddU = 0.0
        self.__uPrev1 = 0.0
        self.__uPrev2 = 0.0
