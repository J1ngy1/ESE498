import RPi.GPIO as GPIO
import time
import smbus          #import SMBus module of I2C
GPIO.setwarnings(False)     # Ignore warning for now
GPIO.setmode(GPIO.BOARD)    # Use physical pin numberin

# ULTRASONIC FUNCTIONS
def ultrasonic_init(Trigger, Echo):
    # set up the input and output pins
    GPIO.setup(Trigger, GPIO.OUT)
    GPIO.output(Trigger, False)
    GPIO.setup(Echo, GPIO.IN)
    # let the sensor initialize
    time.sleep(.5)

def ultrasonic_read(Trigger, Echo):
    # trigger a reading
    GPIO.output(Trigger, True)
    time.sleep(0.00001)
    GPIO.output(Trigger, False)

    # find the start and end of the ultrasonic pulse
    while GPIO.input(Echo) == 0:
        start_time = time.time()
    while GPIO.input(Echo) == 1:
        end_time   = time.time()

    # Speed of sound 34300 cm/sec
    total_distance = (end_time - start_time) * 34300
    # Divide by 2, account for return trip for signal
    return round(total_distance/2, 1) 

def movingAvg(arr, position, numvals=3, wrap=1):
    # default to 3 pt moving average with wrap around on getting values 
    # arr       - array
    # posistion - start from this point on averages
    # numvals   - Number of values in moving average, default of 3
    # wrap      - wrap around to top or bottom of array if 1 (default), no if 0
    sumvals    = 0
    count      = 0    
    array_size = len(arr)
    # if less than numvals data, then just use what is available
    for i in range(numvals):
        # add an item to the list
        if (position - i >= 0 and position - 1 < array_size):
            sumvals = sumvals + arr[(position - i)]
            count   = count + 1
        # wrap backwards, goes to top of array, works in python
        elif (position - i < 0 and wrap == 1): 
            sumvals = sumvals + arr[(position - i)]
            count   = count + 1
        # wrap around to bottom of array with mod
        elif (position - i > array_size and wrap == 1):
            sumvals = sumvals + arr[(position - i)%array_size]
            count   = count + 1
    return sumvals/count

