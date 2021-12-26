import board
import usb_hid
from time import sleep

import gamepad
import siminterface

c=siminterface.SimInterface(board.GP9,board.GP8,100000) #scl,sda
#c.calibrate()
#print(c.calibration2string())
c.calibrate_from_string("[[65.657, -47.1609], [0.0261552, 0.0209381, -0.000537093], [-0.0359255, 0.00386623, 0.000278179]]")

while True:
    try:
        g=gamepad.Gamepad(usb_hid.devices) #if the computer is not listening, we get OSError, so just keep trying...
        break
    except OSError:
        continue


def clip(x,min_x=-127,max_x=127):
    if x<min_x:
        return min_x
    elif x>max_x:
        return max_x
    else:
        return int(x)

#main loop
while True:
    c.ping() #polls each mpu, calculates new positions
    roll_01,pitch_01=c.position_01() #01 refers to [0..1] interval in which these values are brought back
    try:
        r,p=clip(roll_01*255-127),clip(pitch_01*255-127)
        g.move_joysticks(r,-p)
    except OSError:
        pass
    
