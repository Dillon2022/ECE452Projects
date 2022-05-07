import RPi.GPIO as GPIO
import time
from AlphaBot import AlphaBot

Ab = AlphaBot()
Ab.stop()

cntl = 8
cntr = 7

EncR = 0.0
EncL = 0.0

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
GPIO.setup(cntr, GPIO.IN);
GPIO.setup(cntl, GPIO.IN);
GPIO.add_event_detect(cntr, GPIO.BOTH, updateEncoderR)
GPIO.add_event_detect(cntl, GPIO.BOTH, updateEncoderL)


if __name__ == "__main__":
    '''
    1 turn of a wheel will result in 40 counts of encoder.
    ''' 
    Ab.stop()    
    R0 = EncR
    nt = 2
    # Make nt turns
    while (EncR - R0)/40 < nt:
        Ab.setMotor(-20,20)
        print('EncR = %d' %EncR)
    Ab.stop()