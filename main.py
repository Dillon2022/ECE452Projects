import RPi.GPIO as GPIO
import time
from AlphaBot import AlphaBot
from Infrared_Line_Tracking import TRSensor

WHEEL_CIR = 0.64

#Encoder
cntl = 8
cntr = 7
EncR = 0.0
EncL = 0.0

#TR Sensor
CS = 5
Clock = 25
Address = 24
DataOut = 23

def updateEncoderL(channel):
    global EncL
    EncL += 1
    #print('valEncL = %d' %EncL)

    
def updateEncoderR(channel):
    global EncR
    EncR += 1
    #print('valEncR = %d' %EncR)


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(cntr, GPIO.IN)
GPIO.setup(cntl, GPIO.IN)
GPIO.setup(Clock,GPIO.OUT)
GPIO.setup(Address,GPIO.OUT)
GPIO.setup(CS,GPIO.OUT)
GPIO.setup(DataOut,GPIO.IN,GPIO.PUD_UP)
GPIO.add_event_detect(cntr, GPIO.BOTH, updateEncoderR)
GPIO.add_event_detect(cntl, GPIO.BOTH, updateEncoderL)


if __name__ == "__main__":
    '''
    1 turn of a wheel will result in 40 counts of encoder.
    ''' 
    Ab = AlphaBot()
    TR = TRSensor()
    Ab.stop()
    time.sleep(0.5)
    reset = "N"
    usr = input("Begin Calibration")

    # Wiggle the 
    for i in range(0,400):#(reset != "Y"):
        TR.calibrate()
        
    print("Max:", TR.calibratedMax)
    print("Min:", TR.calibratedMin)
    
        
    dist = int(input("Distance to Travel(inches): "))

    R0 = EncR
    R1 = EncL 
    
    nt = dist * (40 / WHEEL_CIR) 
    

    integral = 0
    last_proportional = 0
    maximum = 50
    
    Ab.backward()
    while (((EncR - R0)/40 < nt) or ((EncL - R1)/40 < nt)):

        # 0 1000 2000 3000 4000 ( Resepctive sensor readings ) 2000 suggests middle alignment
        position = TR.readLine(white_line = 0) 
        #print(position)
        
        # negative (Too left) positive (Too right) 
        proportional = position - 2000 

        # Error derivative 
        derivative = proportional - last_proportional

        # Keeps track of last position for refernce 
        last_proportional = proportional
        

        power_difference = proportional/25 #+ derivative/100 #+ integral/1000;  

        if (power_difference > maximum):
            power_difference = maximum
        if (power_difference < - maximum):
            power_difference = - maximum
        print(position,power_difference)
        if (power_difference < 0):
            Ab.setPWMB(maximum + power_difference)
            Ab.setPWMA(maximum)
        else:
            Ab.setPWMB(maximum)
            Ab.setPWMA(maximum - power_difference)

    Ab.stop()