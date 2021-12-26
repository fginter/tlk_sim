# Communication with MPU6050 over I2C
#
# instance of MPU6050 represents a single connected MPU6050

import busio


class MPU6050:
    
    def __init__(self,i2c_scl_pin,i2c_sda_pin,freq=20000):
        self.iic=busio.I2C(i2c_scl_pin,i2c_sda_pin,frequency=freq)
        self.addr=0x68
        self.a=bytearray(14) #used during the comms
        self.reset()
    
    # Lock the bus
    def i2clock(self):
        while not self.iic.try_lock():
            pass
    
    # Unlock the bus
    def i2cunlock(self):
        self.iic.unlock()
    
    # Resent the MPU6050 device
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
