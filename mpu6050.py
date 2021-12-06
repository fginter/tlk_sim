import board
from time import sleep
import math
import adafruit_bus_device
from adafruit_bus_device.i2c_device import I2CDevice
import json
import busio

from time import sleep

# deltat.py time difference calculation for sensor fusion
# Released under the MIT License (MIT)
# Copyright (c) 2018 Peter Hinch

# Provides TimeDiff function and DeltaT class.
# The following notes cover special cases. Where the device performing fusion
# is linked to the IMU and is running MicroPython no special treatment is
# needed.
# The special cases are:
# 1. Device connected to the IMU is linked to a separate platform doing fusion.
# 2. Either or both are not running MicroPython.

# If the device providing the vectors is not running on MicroPython the user
# must supply timestamps and a function capable of differencing these. The
# function is passed to the Fusion constructor and the timestamp is provided
# along with the vector, being the time when the vector was acquired.

# If the device providing the vectors is running MicroPython but fusion is
# being performed on a device which is not, the user must provide their own
# implementation of ticks_diff which accounts for MicroPython rollover and
# must supply the returned ticks_us() values as a timestamp.

# Under MicroPython TimeDiff(start, end) uses time.ticks_diff.

# A DeltaT instance, called with function call syntax, returns a time
# difference from the previous call as a float value. Units seconds.

# If running under MicroPython and no time differencing function is supplied
# to the Fusion constructor it uses time.ticks_us as its time source and a
# default timediff function using time.ticks_diff() with a division by 1e6.
# If time differencing function is supplied a timestamp must be passsed as an
# arg to instance calls of Fusion.update() or Fusion.update_nomag(). In the
# async version the user supplied read_coro() must return a timestamp with the
# vector.

# On 1st pass dt evidently can't be computed. A notional value of 100μs is
# returned. The Madgwick algorithm takes seconds to stabilise.

try:
    import utime as time
except ImportError:
    import time

is_micropython = hasattr(time, 'ticks_diff')

class DeltaT():
    def __init__(self, timediff):
        if timediff is None:
            self.expect_ts = False
            if is_micropython:
                self.timediff = lambda start, end : time.ticks_diff(start, end)/1000000
            else:
                self.timediff = lambda start, end: (start-end)/1000000000.0
                #raise ValueError('You must define a timediff function')
        else:
            self.expect_ts = True
            self.timediff = timediff
        self.start_time = None

    def __call__(self, ts):
        if self.expect_ts:
            if ts is None:
                raise ValueError('Timestamp expected but not supplied.')
        else:
            if is_micropython:
                ts = time.ticks_us()
            else:
                ts = time.monotonic_ns()
                #raise RuntimeError('Not MicroPython: provide timestamps and a timediff function')
        # ts is now valid
        if self.start_time is None:  # 1st call: self.start_time is invalid
            self.start_time = ts
            return 0.0001  # 100μs notional delay. 1st reading is invalid in any case

        dt = self.timediff(ts, self.start_time)
        self.start_time = ts
        self.last_dt=dt
        return dt
    
# Sensor fusion for the micropython board. 25th June 2015
# Ported to MicroPython by Peter Hinch.
# Released under the MIT License (MIT)
# Copyright (c) 2017, 2018 Peter Hinch

# V0.9 Time calculations devolved to deltat.py
# V0.8 Calibrate wait argument can be a function or an integer in ms.
# V0.7 Yaw replaced with heading
# V0.65 waitfunc now optional

# Supports 6 and 9 degrees of freedom sensors. Tested with InvenSense MPU-9150 9DOF sensor.
# Source https://github.com/xioTechnologies/Open-Source-AHRS-With-x-IMU.git
# also https://github.com/kriswiner/MPU-9250.git
# Ported to Python. Integrator timing adapted for pyboard.
# See README.md for documentation.

# Portability: the only assumption is presence of time.sleep() used in mag
# calibration.
try:
    import utime as time
except ImportError:
    import time

from math import sqrt, atan2, asin, degrees, radians
#from deltat import DeltaT

class Fusion(object):
    '''
    Class provides sensor fusion allowing heading, pitch and roll to be extracted. This uses the Madgwick algorithm.
    The update method must be called peiodically. The calculations take 1.6mS on the Pyboard.
    '''
    declination = 0                         # Optional offset for true north. A +ve value adds to heading
    def __init__(self, timediff=None):
        self.magbias = (0, 0, 0)            # local magnetic bias factors: set from calibration
        self.deltat = DeltaT(timediff)      # Time between updates
        self.q = [1.0, 0.0, 0.0, 0.0]       # vector to hold quaternion
        GyroMeasError = radians(40)         # Original code indicates this leads to a 2 sec response time
        self.beta = sqrt(3.0 / 4.0) * GyroMeasError  # compute beta (see README)
        self.pitch = 0
        self.heading = 0
        self.roll = 0

    def calibrate(self, getxyz, stopfunc, wait=0):
        magmax = list(getxyz())             # Initialise max and min lists with current values
        magmin = magmax[:]
        while not stopfunc():
            if wait != 0:
                if callable(wait):
                    wait()
                else:
                    time.sleep(wait/1000)  # Portable
            magxyz = tuple(getxyz())
            for x in range(3):
                magmax[x] = max(magmax[x], magxyz[x])
                magmin[x] = min(magmin[x], magxyz[x])
        self.magbias = tuple(map(lambda a, b: (a +b)/2, magmin, magmax))

    def update_nomag(self, accel, gyro, ts=None):    # 3-tuples (x, y, z) for accel, gyro
        ax, ay, az = accel                  # Units G (but later normalised)
        gx, gy, gz = (radians(x) for x in gyro) # Units deg/s
        q1, q2, q3, q4 = (self.q[x] for x in range(4))   # short name local variable for readability
        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _4q1 = 4 * q1
        _4q2 = 4 * q2
        _4q3 = 4 * q3
        _8q2 = 8 * q2
        _8q3 = 8 * q3
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q4q4 = q4 * q4

        # Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0):
            return # handle NaN
        norm = 1 / norm        # use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        # Gradient decent algorithm corrective step
        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
        s2 = _4q2 * q4q4 - _2q4 * ax + 4 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az
        s3 = 4 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az
        s4 = 4 * q2q2 * q4 - _2q2 * ax + 4 * q3q3 * q4 - _2q3 * ay
        norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        deltat = self.deltat(ts)
        q1 += qDot1 * deltat
        q2 += qDot2 * deltat
        q3 += qDot3 * deltat
        q4 += qDot4 * deltat
        norm = 1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
        self.q = q1 * norm, q2 * norm, q3 * norm, q4 * norm
        self.heading = 0
        self.pitch = degrees(-asin(2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2])))
        self.roll = degrees(atan2(2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]),
            self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3]))

    def update(self, accel, gyro, mag, ts=None):     # 3-tuples (x, y, z) for accel, gyro and mag data
        mx, my, mz = (mag[x] - self.magbias[x] for x in range(3)) # Units irrelevant (normalised)
        ax, ay, az = accel                  # Units irrelevant (normalised)
        gx, gy, gz = (radians(x) for x in gyro)  # Units deg/s
        q1, q2, q3, q4 = (self.q[x] for x in range(4))   # short name local variable for readability
        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _2q1q3 = 2 * q1 * q3
        _2q3q4 = 2 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0):
            return # handle NaN
        norm = 1 / norm                     # use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        # Normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz)
        if (norm == 0):
            return                          # handle NaN
        norm = 1 / norm                     # use reciprocal for division
        mx *= norm
        my *= norm
        mz *= norm

        # Reference direction of Earth's magnetic field
        _2q1mx = 2 * q1 * mx
        _2q1my = 2 * q1 * my
        _2q1mz = 2 * q1 * mz
        _2q2mx = 2 * q2 * mx
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2 * _2bx
        _4bz = 2 * _2bz

        # Gradient descent algorithm corrective step
        s1 = (-_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4)
             + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
             + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s2 = (_2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az)
             + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4)
             + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s3 = (-_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az)
             + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
             + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
             + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s4 = (_2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4)
              + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
              + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        deltat = self.deltat(ts)
        q1 += qDot1 * deltat
        q2 += qDot2 * deltat
        q3 += qDot3 * deltat
        q4 += qDot4 * deltat
        norm = 1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
        self.q = q1 * norm, q2 * norm, q3 * norm, q4 * norm
        self.heading = self.declination + degrees(atan2(2.0 * (self.q[1] * self.q[2] + self.q[0] * self.q[3]),
            self.q[0] * self.q[0] + self.q[1] * self.q[1] - self.q[2] * self.q[2] - self.q[3] * self.q[3]))
        self.pitch = degrees(-asin(2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2])))
        self.roll = degrees(atan2(2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]),
            self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3]))


def vplus(a,b):
    return a[0]+b[0],a[1]+b[1],a[2]+b[2]

def vminus(a,b):
    return a[0]-b[0],a[1]-b[1],a[2]-b[2]

def vdiv(a,b):
    return a[0]/b[0],a[1]/b[1],a[2]/b[2]

def adiv(a,b):
    return a[0]/b,a[1]/b,a[2]/b


def vmul(a,b):
    return a[0]*b[0],a[1]*b[1],a[2]*b[2]

def alpha(x,A,B):#calculates alpha interpolation of x between A and B
    return vdiv(vminus(x,B),vminus(A,B))

def vnorm(x):
    n=math.sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2])
    return x[0]/n,x[1]/n,x[2]/n




class MPU6050:
    
    def __init__(self,i2c_scl_pin,i2c_sda_pin,freq=20000):
        self.iic=busio.I2C(i2c_scl_pin,i2c_sda_pin,frequency=freq)
        self.addr=0x68
        self.a=bytearray(14) #used during the comms
        self.reset()
    
    def i2clock(self):
        while not self.iic.try_lock():
            pass
        
    def i2cunlock(self):
        self.iic.unlock()
    
    def reset(self):
        self.i2clock()
        self.iic.writeto(self.addr, bytearray([107, 0]))            #Wake up!
        self.iic.writeto(self.addr, bytearray([0x1c, 0b11100000]))  #Set range to +/-2g
        self.iic.writeto(self.addr, bytearray([0x1b, 0b11100000]))  #Set accel to 250dg/sec
        #self.iic.writeto(self.addr, bytearray([0x1a, 0b00000110]))  #this was a low pass filter setup
        self.i2cunlock()
        
    def get_raw_values(self):
        self.i2clock()
        self.iic.writeto_then_readfrom(self.addr,bytearray([0x3B]),self.a) #14
        self.i2cunlock()
        return self.a

    def get_ints(self):
        l=list(self.get_raw_values())
        return l

    def bytes_toint(self, firstbyte, secondbyte):
        if not firstbyte & 0x80:
            return firstbyte << 8 | secondbyte
        return - (((firstbyte ^ 255) << 8) | (secondbyte ^ 255) + 1)

    def bytes_toint2(self, b):
        return int.from_bytes(b,'big',True)
        

    def get_values(self):
        raw_ints = self.get_raw_values()
        vals = {}
        vals["AcX"] = self.bytes_toint(raw_ints[0], raw_ints[1])
        vals["AcY"] = self.bytes_toint(raw_ints[2], raw_ints[3])
        vals["AcZ"] = self.bytes_toint(raw_ints[4], raw_ints[5])
        #vals["Tmp"] = self.bytes_toint(raw_ints[6], raw_ints[7]) / 340.00 + 36.53
        vals["GyX"] = self.bytes_toint(raw_ints[8],raw_ints[9])
        vals["GyY"] = self.bytes_toint(raw_ints[10],raw_ints[11])
        vals["GyZ"] = self.bytes_toint(raw_ints[12],raw_ints[13])
        return vals  # returned in range of Int16
        # -32768 to 32767
    
    def get_acc(self):
        v=self.get_values()
        return (v["AcX"],v["AcY"],v["AcZ"])
    
    def get_acc_gyro(self):
        v=self.get_values()
        return (v["AcX"],v["AcY"],v["AcZ"]), (v["GyX"]-5610,v["GyY"]+6020,v["GyZ"]-9310)

    def val_test(self):  # ONLY FOR TESTING! Also, fast reading sometimes crashes IIC
        
        while 1:
            print(self.get_values())
            sleep(0.05)
            
    def average_val(self,duration=2000,ms=10):
        #measures average for duration [ms] every ms [ms]
        avg=(0,0,0)
        counter=0
        for _ in range(0,duration,ms):
            a=self.get_acc()
            avg=(avg[0]+a[0],avg[1]+a[1],avg[2]+a[2])
            counter+=1
        print(avg)
        return (int(avg[0]/counter),int(avg[1]/counter),int(avg[2]/counter))

# This linear system solver is borrowed from https://github.com/ThomIves/SystemOfEquations/blob/master/ShortImplementation.py
def solve_equations(AM, BM):
    for fd in range(len(AM)):
        fdScaler = 1.0 / AM[fd][fd]
        for j in range(len(AM)):
            AM[fd][j] *= fdScaler
        BM[fd][0] *= fdScaler
        for i in list(range(len(AM)))[0:fd] + list(range(len(AM)))[fd+1:]:
            crScaler = AM[i][fd]
            for j in range(len(AM)):
                AM[i][j] = AM[i][j] - crScaler * AM[fd][j]
            BM[i][0] = BM[i][0] - crScaler * BM[fd][0]
    return BM

def minus(X,Y):
    return [X[0]-Y[0],X[1]-Y[1]]

#
#  calibrate(A,B,C,D) returns transforms TX and TY that translate the polygon defined by A,B,C,D onto the unit square
#  the transform is of x'=ax+by+cxy   and y'=dx+ey+fxy  and TX=[a,b,c] and TY=[d,e,f]
#
#   B.......C             [0,1]..........[1,1]
#   .       .             .               .
#   .       .      ->     .               .
#   .       .             .               .
#   A.......D            [0,0]..........[1,0]
#
def calibrate_square(A,B,C,D):
    #1) shift A to origin
    B,C,D=minus(B,A),minus(C,A),minus(D,A) 
    #2) find a,b,c such that x'=ax+by+cxy  mapping from input (x,y) onto unit square
    #   a,b,c is then used to get the x coordinate
    M=[[B[0],B[1],B[0]*B[1]],[C[0],C[1],C[0]*C[1]],[D[0],D[1],D[0]*D[1]]]
    XP=[[0],[1],[1]] #x-coords of the corners B,C,D
    TX=solve_equations(M,XP) #transform of X
    #3) same for y
    M=[[B[0],B[1],B[0]*B[1]],[C[0],C[1],C[0]*C[1]],[D[0],D[1],D[0]*D[1]]]
    YP=[[1],[1],[0]] #y-coords of the corners B,C,D
    TY=solve_equations(M,YP) #transform of Y
    return [TX[0][0],TX[1][0],TX[2][0]],[TY[0][0],TY[1][0],TY[2][0]]

# #
# #
# # Given a measured point X, the origin A, and calibration values TX,TY, calculate the corresponding point on the unit square
# #
# #
# def get01(X,A,TX,TY):
#     X=minus(X,A) #shift to origin
#     xp=TX[0]*X[0]+TX[1]*X[1]+TX[2]*X[0]*X[1] #x transform
#     yp=TY[0]*X[0]+TY[1]*X[1]+TY[2]*X[0]*X[1] #y transform
#     return [xp,yp] #done!
# 
# A=[0,0]
# B=[-1,3]
# C=[5,6]
# D=[4,-1]
# TX,TY=calibrate(A,B,C,D)
# 
# #this should get us [0,0],[0,1],[1,1],[1,0]
# for point in (A,B,C,D):
#     print(point,"  ->  ",get01(point,A,TX,TY))

        
class Comb:
    
    def __init__(self,i2c_scl_pin,i2c_sda_pin,freq=20000):
        self.mpu=MPU6050(i2c_scl_pin,i2c_sda_pin,freq)
        self.fusion=Fusion()
        self.reset_averages(5)
        
    def calibrate(self):
        self.roll_calib=[[0,0],[0,0]]
        self.pitch_calib=[[0,0],[0,0]]
        names=["ROLL-LEFT NOSE-UP","ROLL-LEFT NOSE-DOWN","ROLL-RIGHT NOSE-DOWN","ROLL-RIGHT NOSE-UP"]
        coords=[(0,0),(0,1),(1,1),(1,0)] 
        old_cache_size=self.cache_size
        
        measured_corners={} #key: (0,1), (1,1) etc
        for name,coord in zip(names,coords):
            print(name)
            self.reset_averages(50)
            for i in range(600):
                #print("ping",i)
                self.ping()
            measured_corners[coord]=(self.smooth_roll,self.smooth_pitch)
            
        #Now re-scale the corners into a 1x1 square
        A,B,C,D=measured_corners[(0,0)],measured_corners[(0,1)],measured_corners[(1,1)],measured_corners[(1,0)]
        self.calib_A=A #left-back (nose up) is the 0-0 point
        self.calib_TX,self.calib_TY=calibrate_square(A,B,C,D) #calculates the coefficients of the transform into 1x1 square returned as (TX,TY)
        self.reset_averages(old_cache_size)
 
    def save_calibration(self,fname="calib.json"):
         with open(fname,"wt") as f:
             json.dump((self.calib_A,self.calib_TX,self.calib_TY),f)
             
    def load_calibration(self,fname="calib.json"):
        with open(fname,"rt") as f:
            (self.calib_A,self.calib_TX,self.calib_TY)=json.load(f)
 
    def calibration2string(self):
        print(json.dumps((self.calib_A,self.calib_TX,self.calib_TY)))
    
    def calibrate_from_string(self,calib_json):
        self.calib_A,self.calib_TX,self.calib_TY=json.loads(calib_json)
              
 
    def position_01(self):
        X=minus((self.smooth_roll,self.smooth_pitch),self.calib_A) #shift to origin
        TX=self.calib_TX
        TY=self.calib_TY
        roll=TX[0]*X[0]+TX[1]*X[1]+TX[2]*X[0]*X[1] #x transform
        pitch=TY[0]*X[0]+TY[1]*X[1]+TY[2]*X[0]*X[1] #y transform
        return (roll,pitch) #done!
        
    def reset_averages(self,cache_size):
        self.measurement_id=0 #always points to the oldest value
        self.cache_size=cache_size
        self.rolls=[] #cache of measurements of rolls
        self.pitches= [] #cache of measurements of pitches
        self.sum_of_rolls=0
        self.sum_of_pitches=0
        self.smooth_roll=None
        self.smooth_pitch=None

    def ping(self):
        a,g=self.mpu.get_acc_gyro()
        g=g[0]/131.0,g[1]/131.0,g[2]/131.0
        self.fusion.update_nomag(a,g)
        self.sum_of_rolls+=self.fusion.roll
        self.sum_of_pitches+=self.fusion.pitch
        if len(self.rolls)<self.cache_size: #not full yet
            self.rolls.append(self.fusion.roll)
            self.pitches.append(self.fusion.pitch)
        else: #this is a new measurement, gotta take the last one out
            self.sum_of_rolls-=self.rolls[self.measurement_id]
            self.sum_of_pitches-=self.pitches[self.measurement_id]
            self.rolls[self.measurement_id]=self.fusion.roll
            self.pitches[self.measurement_id]=self.fusion.pitch
            self.measurement_id=(self.measurement_id+1)%self.cache_size
        self.smooth_roll=self.sum_of_rolls/len(self.rolls)
        self.smooth_pitch=self.sum_of_pitches/len(self.rolls)

# iic=busio.I2C(board.GP1,board.GP0)
# while not iic.try_lock():
#     pass
# print("SCAN",iic.scan())
# iic.writeto(0x68, bytearray([107, 0]))
# iic.writeto(0x68, bytearray([0x1c, 0b11100000]))  #Set range to +/-2g
# iic.writeto(0x68, bytearray([0x1b, 0b11100000]))  #Set accel to 250dg/sec
# iic.unlock()
# print("DONE")
#c=Comb(board.GP1,board.GP0) #scl,sda
c=Comb(board.GP9,board.GP8,100000) #scl,sda
#c.calibrate()
#print(c.calibration2string())
c.calibrate_from_string("[[66.7707, -29.4564], [0.0279107, 0.0514151, -0.00060503], [-0.0478363, 0.00493958, 0.000115104]]")


#import adafruit_hid
#from adafruit_hid.hid_gamepad import Gamepad
#import adafruit_hid.gamepad
#gp=adafruit_hid.gamepad.Gamepad()

counter=0
while True:
    c.ping()
    if counter>=10:
        #print(c.smooth_roll,c.smooth_pitch)
        roll_01,pitch_01=c.position_01()
        print("ROLL %03.1f"%(roll_01*100),"PITCH %03.1f"%(pitch_01*100),"     delta-t",c.fusion.deltat.last_dt)
        counter=0
    counter+=1
    
    