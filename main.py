import RPi.GPIO as GPIO
import time
from AlphaBot import AlphaBot
from Infrared_Line_Tracking import TRSensor

# Wheel Circumference (Inches) 
WHEEL_CIR = 8.64

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

# Encoder Pins 
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(cntr, GPIO.IN)
GPIO.setup(cntl, GPIO.IN)
GPIO.add_event_detect(cntr, GPIO.BOTH, updateEncoderR)
GPIO.add_event_detect(cntl, GPIO.BOTH, updateEncoderL)

# PINS for IR Data Lines 
GPIO.setup(Clock,GPIO.OUT)
GPIO.setup(Address,GPIO.OUT)
GPIO.setup(CS,GPIO.OUT)
GPIO.setup(DataOut,GPIO.IN,GPIO.PUD_UP)



if __name__ == "__main__":
    # Initialize Robot and IR Sensors 
    Ab = AlphaBot()
    TR = TRSensor()
    Ab.stop()


    time.sleep(0.5)
    usr = input("Begin Calibration")

    # Wiggle the 
    for i in range(0,400):#(reset != "Y"):
        TR.calibrate()
    
    # [ L , ... , R] [0, 1000]
    # Must have large difference in Max - Min for higher sensitivty 
    print("Max:", TR.calibratedMax)
    print("Min:", TR.calibratedMin)
    
    # User Input distance to travel (Floating point) Inches
    dist = float(input("Distance to Travel(inches): "))
    
    # Starting position of the encoder, used as reference 
    start = EncR
    
    # Left Encoder not working
    #R1 = EncL 
    
    # Number of Encdoer counts 
    ncount = dist * (40 / WHEEL_CIR) 
    
    integral = 0
    last_proportional = 0
    maximum = 40
    
    Ab.backward()
    while (((EncR - start) < ncount)):
        #print(EncR, EncL)
        
        #Current Distance Traveled
        print("Current Distance:",((EncR - start) / 40) * WHEEL_CIR) 
        
        # 0 1000 2000 3000 4000 ( Resepctive sensor readings ) 2000 suggests middle alignment
        position = TR.readLine(white_line = 0) 
        #print(position)
        
        # negative (Too left) positive (Too right) 
        proportional = position - 2000 

        # Error derivative 
        derivative = proportional - last_proportional

        # Keeps track of last position for refernce 
        last_proportional = proportional
        
        # Difference required for correction
        power_difference = proportional/25 + derivative/100  
        
        #Ensures power_difference is in scope of declared duty cycle
        if (power_difference > maximum):
            power_difference = maximum
            
        if (power_difference < - maximum):
            power_difference = - maximum
        
        # Sets the PWM signal of R and L wheel to correct rotation 
        
        if (power_difference < 0):
            Ab.setPWMB(maximum + power_difference)
            Ab.setPWMA(maximum)
        else:
           Ab.setPWMB(maximum)
           Ab.setPWMA(maximum - power_difference)
            
            
    # Out of Loop, END        
    print('Reached Distance')
    Ab.stop()