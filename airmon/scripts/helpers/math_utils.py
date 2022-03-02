#!/usr/bin/env python3

import numpy as np

#---------------------------------------------------------------
#---------------------------------------------------------------
# CLASSES

## Three-dimensional vector
class Vec3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def GetLength(self):
        return np.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    ## Rotate vector around axis with angle
    def RotateAxis(self, axis, ang):
        fCos = np.cos(ang)
        fSin = np.sin(ang)
        return Vec3\
        (\
            (fCos + axis.x * axis.x * (1. - fCos)) * self.x +\
            (axis.x * axis.y * (1. - fCos) - axis.z * fSin) * self.y +\
            (axis.x * axis.z * (1. - fCos) + axis.y * fSin) * self.z,\
            (axis.y * axis.x * (1. - fCos) + axis.z * fSin) * self.x +\
            (fCos + axis.y * axis.y * (1. - fCos)) * self.y +\
            (axis.y * axis.z * (1. - fCos) - axis.x * fSin) * self.z,\
            (axis.z * axis.x * (1. - fCos) - axis.y * fSin) * self.x +\
            (axis.z * axis.y * (1. - fCos) + axis.x * fSin) * self.y +\
            (fCos + axis.z * axis.z * (1. - fCos)) * self.z\
        )

    ## Reverse-rotate vector around axis with angle
    def UnRotateAxis(self, axis, ang):
        fCos = np.cos(ang)
        fSin = np.sin(ang)
        return Vec3\
        (\
            (fCos + axis.x * axis.x * (1. - fCos)) * self.x +\
            (axis.y * axis.x * (1. - fCos) + axis.z * fSin) * self.y +\
            (axis.z * axis.x * (1. - fCos) - axis.y * fSin) * self.z,\
            (axis.x * axis.y * (1. - fCos) - axis.z * fSin) * self.x +\
            (fCos + axis.y * axis.y * (1. - fCos)) * self.y +\
            (axis.z * axis.y * (1. - fCos) + axis.x * fSin) * self.z,\
            (axis.x * axis.z * (1. - fCos) + axis.y * fSin) * self.x +\
            (axis.y * axis.z * (1. - fCos) - axis.x * fSin) * self.y +\
            (fCos + axis.z * axis.z * (1. - fCos)) * self.z\
        )

    ## Rotate vector with given euler angles
    def Rotate(self, angles):
        return self.RotateAxis(Vec3(0., 0., 1.), angles.yaw).RotateAxis(Vec3(0., 1., 0.), angles.pitch).RotateAxis(Vec3(1., 0., 0.), angles.roll)

    ## Reverse-rotate vector with given euler angles
    def UnRotate(self, angles):
        return self.UnRotateAxis(Vec3(1., 0., 0.), angles.roll).UnRotateAxis(Vec3(0., 1., 0.), angles.pitch).UnRotateAxis(Vec3(0., 0., 1.), angles.yaw)

    def __add__(self, other):
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
        
    def __sub__(self, other):
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other):
        return Vec3(self.x * other, self.y * other, self.z * other)

    def __truediv__(self, other):
        return Vec3(self.x / other, self.y / other, self.z / other)
        
    def __iadd__(self, other):
        self.x += other.x
        self.y += other.y
        self.z += other.z
        return self

    def __isub__(self, other):
        self.x -= other.x
        self.y -= other.y
        self.z -= other.z
        return self

    def __imul__(self, other):
        self.x *= other
        self.y *= other
        self.z *= other
        return self

    def __itruediv__(self, other):
        self.x /= other
        self.y /= other
        self.z /= other
        return self

    def __abs__(self):
        return self.GetLength()

    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other):
        return Vec3(self.y * other.z - other.y * self.z, self.z * other.x - other.z * self.x, self.x * other.y - other.x * self.y)

## Euler angles structure
class Rotator:
    def __init__(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


class Quat:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    ## Conversion to euler angles
    ## Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    def GetRotator(self):
        angles = Rotator(0, 0, 0)

        # roll (x-axis rotation)
        sinr_cosp = 2 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1 - 2 * (self.x * self.x + self.y * self.y)
        angles.roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (self.w * self.y - self.z * self.x)
        if (abs(sinp) >= 1):
            angles.pitch = np.copysign(np.pi / 2, sinp) # use 90 degrees if out of range
        else:
            angles.pitch = np.arcsin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1 - 2 * (self.y * self.y + self.z * self.z)
        angles.yaw = np.arctan2(siny_cosp, cosy_cosp)

        return angles


#---------------------------------------------------------------
#---------------------------------------------------------------
# HELPER FUNCTIONS

## Clamp angle in radians to [-pi; pi] range
def ClampAngle(angle):
    if(angle == 0.0):
        sign = 0.0
    else:
        sign = abs(angle) / angle
    while (angle < -np.pi or angle > np.pi):
        angle -= 2 * np.pi * sign
    return angle

## Get a value linearly interpolated in [a; b] range
def Lerp(a, b, alpha):
    return a + alpha * (b - a)

## Reinterpret 'val' from [a1; b1] into [a2; b2] range, clamp if necessary
def Remap(val, a1, b1, a2, b2, clamp = True):
    alpha = (val - a1) / (b1 - a1)
    result = Lerp(a2, b2, alpha)
    if(clamp):
        if(result < a2):
            result = a2
        elif(result > b2):
            result = b2
    return result

## Degrees to radians
def Deg2Rad(angle):
    return np.radians(angle)

## Radians to degrees
def Rad2Deg(angle):
    return np.degrees(angle)

## Check equality with tolerance
def Equal(n1, n2, tol):
    return abs(n1 - n2) <= tol

##
# Calculates vector in cartesian coordinates (x, y | meters)
# from p1 to p2 in geographical coordinates (latitude, lontitude | degrees)
def GetCartesianOffset(gpos1, gpos2):
    # Earth max radius
    r = 6378100.0
    x = np.radians(gpos2[1] - gpos1[1]) * abs(np.cos(np.radians((gpos1[0]+gpos2[0]) / 2))) * r
    y = np.radians(gpos2[0] - gpos1[0]) * r
    return x, y