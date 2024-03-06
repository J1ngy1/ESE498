import math
import argparse
import time
import RPi.GPIO as GPIO
import cv2
import numpy as np
from time import sleep
import mod8_func as motor
import matplotlib
import matplotlib.pyplot as plt
from picar import PiCar, test, configure


parser = argparse.ArgumentParser(description='Data for this program.')
parser.add_argument('--debug', action='store_true', default = False,
   help='specifies if debug statements are printed')
parser.add_argument('--mock_car', action='store_true', default = False,
   help='False as default')
parser.add_argument('--tim', action='store', type=int, default=4,
   help='time for program to run in seconds')
parser.add_argument('--delay', action='store', type=float, default = 0.005,
   help='delay in 0.005 sec for time between samples')
parser.add_argument('--delay2', action='store', type=float, default = 0.05,
   help='delay in 0.05 sec for time between samples')
parser.add_argument('--delay_cal', action='store', type=float, default = 0.25,
   help='delay in 0.25 sec for time between calculate motor speed')
parser.add_argument('--duty', action='store', type=float, default = 50,
   help='duty cycle in %')
parser.add_argument('--rps', action='store', type=float, default = 3,
   help='rps value desired')
parser.add_argument('--kp', action='store', type=float, default = 0,
   help='set kp which will time error (open loop gain = 1/slope)')
parser.add_argument('--ki', action='store', type=float, default = 0,
   help='set ki which will time allErrors')
parser.add_argument('--kd', action='store', type=float, default = 0,
   help='set kd which will time diffError')
parser.add_argument('--wait', action='store', type=float, default = 2,
   help='time to wait before start')
args = parser.parse_args()

wait = args.wait
rps_enter = args.rps

car = PiCar(mock_car=False, threaded=True)
car.set_motor(args.duty)

start_time = time.time() + args.wait
cur_time = start_time
delay_time = start_time
delay_time_cal = start_time
delay_time2 = start_time
delay_cal = args.delay_cal
rps_cal = 0
error_cal = 0
allErrors_cal = 0
diffError_cal = 0

value = []
time_need_array = []
delta = [0]
RPS = [0]*3000
error = [0]*3000
allErrors = [0]*3000
diffError = [0]*3000

delay = args.delay
delay2 = args.delay2
delay_cal = args.delay_cal

open_loop_gain = 12.1
duty = rps_enter*open_loop_gain
new_duty = duty
kp = args.kp
ki = args.ki
kd = args.kd

tim = args.tim
name = 'data_9c.txt'
car.set_motor(duty)
count = 0

flag = 1
onezeros = [0]*3000
index = 0
samples = (args.delay_cal/args.delay)-1
thr = [15]*3000
start = 0
stop = 0

#for objective 2 under:
angle_blue = 0
car.adc.read_adc(0)

def image_process(debug):
   img = car.get_image()
   #img = cv2.flip(img, 0) need flip for fake car
   if (img is not None):
      cv2.imwrite("og_img.jpg", img)

      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, (90, 50, 50), (125, 255, 255))
      mask_blur = cv2.blur(mask,(5,5))
      cv2.imwrite("mask.jpg", mask)


      thresh = cv2.threshold(mask_blur, 200, 255, cv2.THRESH_BINARY)[1]
      cv2.imwrite("thresh.jpg", thresh)

      width  = int(thresh.shape[1])
      height = int(thresh.shape[0])
      M = cv2.moments(thresh)

      print(M["m10"])
      print(M["m01"])
      print(M["m00"])
      print('\n')

      cur_time = time.time()

      if (M["m00"] == 0):
         print(f'Time={cur_time:0.3f} \t Angle=360 \n')
         return 360
      else:
         cX = int(M["m10"]/M["m00"])
         cY = int(M["m01"]/M["m00"])
         thresh_dot = cv2.circle(thresh, (cX,cY), 5, (0, 0, 255), 2)
         cv2.imwrite("thresh_dot.jpg", thresh_dot)

         angle = (-1) * np.tan((cX - 0.5*width)/(height - cY)) * 180/(math.pi)

         print(f'Time={(cur_time-start_time):0.3f} \t Angle={angle:0.2f} \n')
         return angle

while (cur_time - start_time < tim):
    cur_time = time.time()
#    if (cur_time - start_time < wait):             #Uncomment these to create velocityplot.png
#        motor.motor_direction(in1, in2, 0)
#    if (cur_time - start_time >= wait):
#        motor.motor_direction(in1, in2, 1)

    if (delay_time + delay < cur_time):
        delay_time = cur_time

        value.append(car.adc.read_adc(0))

        RPS[index] = rps_cal
        error[index] = error_cal
        allErrors[index] = allErrors_cal
        diffError[index] = diffError_cal
        if (index >= 1):
           delta.append(value[index]-value[index-1])
           if(index - 100 >= 0):
              thr[index] = 0.2*(max(delta[index-100:index])-min(delta[index-100:index]))

        time_need = cur_time - start_time
        time_need_array.append(time_need)
        index = index + 1

        if (delay_time_cal + delay_cal < cur_time):
            delay_time_cal = cur_time
            if(index-samples >=0):
               for n in range(len(delta)-1):
                  if flag == 1:
                     if (delta[n] > thr[n]):
                        onezeros[n] = 1
                        flag = 0
                  else:
                     if (delta[n] < -thr[n]):
                        onezeros[n] = 1
                        flag = 1
            count = index
            numTran = 0
            while(numTran < 5 and count >= 0):
               if(onezeros[count] == 1):
                  numTran += 1
                  if(numTran == 1):
                     start = count
                  if(numTran == 5):
                     stop = count
               count -= 1
            if(numTran == 5):
               rps_cal = 1/(time_need_array[start] - time_need_array[stop])
               numTran = 0
               error_cal = rps_enter - rps_cal
               diffError_cal = error_cal - error[index-1]
               allErrors_cal = allErrors_cal + error_cal

               new_duty = args.rps*open_loop_gain + kp*error_cal + ki*allErrors_cal + kd*diffError_cal
               if(new_duty > 100):
                  new_duty = 100
               if(new_duty < 0):
                  new_duty = 0
               car.set_motor(new_duty)
               print(f'rps_cal{rps_cal}')
               print(f'{new_duty:3f} = ({args.rps}*12.1) + ({kp}*{error_cal:3f})+({ki}*{allErrors_cal:3f})+({kd}*{diffError_cal:3f})')


with open(name, 'w') as f:
   for i in range(len(time_need_array)):
      f.write(f'{time_need_array[i]:.4f}\t {value[i]:.4f}\t {RPS[i]:.4f}\t {onezeros[i]}\t {delta[i]}\t {thr[i]}\n')
f.close

plt.figure()
plt.plot(time_need_array,RPS[1:len(time_need_array)+1])
plt.grid(True)
plt.ylabel("RPS value")
plt.xlabel("time(s)")

name = 'plot_9c.png'
plt.savefig(name)


plt.figure()
plt.plot(time_need_array[1:len(time_need_array)+1],value[1:len(time_need_array)+1])
plt.grid(True)
plt.ylabel("ADC value")
plt.xlabel("time(s)")
name = 'test_ADC' + str(round(duty)) + '_kp' + str(kp) + '_ki' + str(ki) + '_kd' + str(kd) + '.png'
plt.savefig(name)

GPIO.cleanup()

