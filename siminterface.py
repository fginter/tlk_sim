import fusion # fusion.py; this code implements IMU orientation math
import mpu6050
import json


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


# This class glues everything together:
#
#   a) mpu communication
#   b) fusion of MPU values into angles
#   c) calibration

class SimInterface:
    
    def __init__(self,i2c_scl_pin,i2c_sda_pin,freq=20000):
        self.mpu_ae=mpu6050.MPU6050(i2c_scl_pin,i2c_sda_pin,freq)  #aileron & elevator MPU, i.e. the stick
        self.fusion_ae=fusion.Fusion()
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
        self.ping_ae()

    def ping_ae(self):
        a,g=self.mpu_ae.get_acc_gyro()
        g=g[0]/131.0,g[1]/131.0,g[2]/131.0
        self.fusion_ae.update_nomag(a,g)
        self.sum_of_rolls+=self.fusion_ae.roll
        self.sum_of_pitches+=self.fusion_ae.pitch
        if len(self.rolls)<self.cache_size: #not full yet
            self.rolls.append(self.fusion_ae.roll)
            self.pitches.append(self.fusion_ae.pitch)
        else: #this is a new measurement, gotta take the last one out
            self.sum_of_rolls-=self.rolls[self.measurement_id]
            self.sum_of_pitches-=self.pitches[self.measurement_id]
            self.rolls[self.measurement_id]=self.fusion_ae.roll
            self.pitches[self.measurement_id]=self.fusion_ae.pitch
            self.measurement_id=(self.measurement_id+1)%self.cache_size
        self.smooth_roll=self.sum_of_rolls/len(self.rolls)
        self.smooth_pitch=self.sum_of_pitches/len(self.rolls)
