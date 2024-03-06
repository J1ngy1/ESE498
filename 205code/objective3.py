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
parser.add_argument('--tim', action='store', type=int, default=15,
   help='time for program to run in seconds')
parser.add_argument('--delay', action='store', type=float, default = 0.01, #was 0.005
   help='delay in 0.005 sec for time between samples')
parser.add_argument('--delay_cal', action='store', type=float, default = 0.5, #was 0.3
   help='delay in 0.25 sec for time between calculate motor speed')
parser.add_argument('--duty', action='store', type=float, default = 40,
   help='duty cycle in %')
parser.add_argument('--rps', action='store', type=float, default = 4,
   help='rps value desired')
parser.add_argument('--kp', action='store', type=float, default = 5,
   help='set kp which will time error (open loop gain = 1/slope)')
parser.add_argument('--ki', action='store', type=float, default = 3.4,
   help='set ki which will time allErrors')
parser.add_argument('--kd', action='store', type=float, default = 0,
   help='set kd which will time diffError')
parser.add_argument('--wait', action='store', type=float, default = 2, #was 0
   help='time to wait before start')
args = parser.parse_args()

delta_angle = 0.2 # was 0.19

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

MAXSIZE = 3000
value = [0]*MAXSIZE
time_need_array = []
del_time_need = [0]*MAXSIZE
delta = [0]*MAXSIZE
RPS = [0]*MAXSIZE
error = [0]*MAXSIZE
allErrors = [0]*MAXSIZE
diffError = [0]*MAXSIZE
#numTran = [0]*MAXSIZE

delay = args.delay
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
onezeros = [0]*MAXSIZE
index = 0
samples = (args.delay_cal/args.delay)-1
thr = [40]*MAXSIZE #was 15
start = 0
stop = 0

#for objective 2 under:
angle_blue = 0
car.adc.read_adc(0)

max_steer_dc = 10 #need to configure these numbers
min_steer_dc = -10
steer_dc = 0

pre_rps_cal = args.rps

def image_process(debug):
   img = car.get_image()
   #img = cv2.flip(img, 0) need flip for fake car
   if (img is not None):
      cv2.imwrite("og_img.jpg", img)

      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, (90, 50, 50), (125, 255, 255))
      mask_blur = cv2.blur(mask,(5,5))
      #cv2.imwrite("mask.jpg", mask)


      thresh = cv2.threshold(mask_blur, 200, 255, cv2.THRESH_BINARY)[1]
      #cv2.imwrite("thresh.jpg", thresh)

      width  = int(thresh.shape[1])
      height = int(thresh.shape[0])
      M = cv2.moments(thresh)

      #print(M["m10"])
      #print(M["m01"])
      #print(M["m00"])
      #print('\n')

      #cur_time = time.time()

      if (M["m00"] == 0):
         print(f'Angle=360 \n')
         return 360
      else:
         cX = int(M["m10"]/M["m00"])
         cY = int(M["m01"]/M["m00"])
         thresh_dot = cv2.circle(thresh, (cX,cY), 5, (0, 0, 255), 2)
         cv2.imwrite("thresh_dot.jpg", thresh_dot)

         angle = (-1) * np.tan((cX - 0.5*width)/(height - cY)) * 180/(math.pi)

         print(f'Angle={angle:0.2f} \n')
         return angle
rps_cal = args.rps

while (cur_time - start_time < tim):
    cur_time = time.time()

    if (delay_time + delay < cur_time):
        delay_time = cur_time
        #print(f'Delay enter: {delay_time - start_time}')

        value[index] = car.adc.read_adc(0)
        RPS[index] = rps_cal
        error[index] = error_cal
        allErrors[index] = allErrors_cal
        diffError[index] = diffError_cal
        if (index >= 1):
           delta[index] = value[index]-value[index-1]
           if(index - 200 >= 0): #was index - 100
              thr[index] = 0.15*(max(delta[index-200:index])-min(delta[index-200:index])) #was -100 and 0.2
              if (thr[index] >= 80):
                  thr[index] = 80
              if (value[index] >= 150+max(value[index-200:index]) or value[index] <= min(value[index-200:index])-150):
                  value[index] = pre_value
              pre_value = value[index]
        time_need = cur_time - start_time
        time_need_array.append(time_need)
        if (index >= 1):
            del_time_need[index] = time_need_array[index] - time_need_array[index-1]
        index = index + 1

    if (delay_time_cal + delay_cal < cur_time):
        delay_time_cal = cur_time
        #print(f'Delay_cal enter: {delay_time_cal - start_time}')

        if(index-samples >=0):
           for n in range(len(delta)-1):
              if flag == 1:
                 if (delta[n] > thr[n]):
                    onezeros[n] = 1
                    #if (onezeros[n] == 1 and onezeros[n-1] ==1):
                    #    onezeros[n] == 0
                    flag = 0
              else:
                 if (delta[n] < -thr[n]):
                    onezeros[n] = 1
                    #if (onezeros[n] == 1 and onezeros[n-1] ==1):
                    #    onezeros[n] == 0
                    flag = 1
        count = index
        numTran = 0
        while(numTran < 4 and count >= 0): # 4 was 5
           if(del_time_need[count] >= 0.05):
              numTran = 0
           if(onezeros[count] == 1):
              #print(f'numTran{numTran}')
              if(numTran == 0):
                 start = count
                 #print(f'numTran{numTran}')
              if(numTran == 2): # was 5
                 stop2 = count
                 rps_cal = (2/4)*1/(time_need_array[start] - time_need_array[stop2])
                 #print(f'numTran{numTran}')
              if(numTran == 3): # was 5
                 stop3 = count
                 rps_cal = (3/4)*1/(time_need_array[start] - time_need_array[stop3])
                 #print(f'numTran{numTran}')
              #if(numTran == 4): # was 5
              #   stop = count
                 #print(f'numTran{numTran}')
              numTran += 1 #was numTran += 1
           count -= 1
   #     if(numTran == 3): #was 5
   #        rps_cal = (2/4)*1/(time_need_array[start] - time_need_array[stop2])
           #print(f'2start{time_need_array[start]}-stop{time_need_array[stop]}')
   #     elif(numTran == 4): #was 5
   #        rps_cal = (3/4)*1/(time_need_array[start] - time_need_array[stop3])
           #print(f'3start{time_need_array[start]}-stop{time_need_array[stop]}')
        #elif(numTran == 5): #was 5
           #print(f'4start{time_need_array[start]}-stop{time_need_array[stop]}')
           #rps_cal = 1/(time_need_array[start] - time_need_array[stop])
        error_cal = rps_enter - rps_cal
        diffError_cal = error_cal - error[index-1]
        allErrors_cal = allErrors_cal + error_cal

        distance = car.read_distance()
        if distance is None:
            distance = pre_distance
        pre_distance = distance
        if (distance >= 120):
            print(f'Distance(larger than 50)={distance:0.2f} cm')
            new_duty = args.rps*open_loop_gain + kp*error_cal + ki*allErrors_cal + kd*diffError_cal
            if(new_duty > 100):
               new_duty = 100
            if(new_duty < 0):
               new_duty = 0
            car.set_motor(new_duty)
            print(f'rps_cal{rps_cal}')
            print(f'{new_duty:3f} = ({args.rps}*12.1) + ({kp}*{error_cal:3f})+({ki}*{allErrors_cal:3f})+({kd}*{diffError_cal:3f})')
        if (distance >=90 and distance < 120):
            car.set_motor(0)
            print(f'Distance(0)={distance:0.2f} cm')
        if (distance >=65 and distance < 90):
            car.set_motor(10, forward=False)
            print(f'Distance(-10)={distance:0.2f} cm')
        if (distance >=40 and distance < 65):
            car.set_motor(25)
            print(f'Distance(25)={distance:0.2f} cm')
        if (distance < 40 and distance >= 8):
            car.set_motor(15)
            print(f'Distance(15)={distance:0.2f} cm')
        if (distance < 8):
            car.set_motor(-10,forward=False)
            print(f'Distance(-10)={distance:0.2f} cm')


        angle_blue = image_process(args.debug)
        angle_blue = 0 if angle_blue is None else angle_blue
        if (angle_blue != 360) and (-700 <= angle_blue <= 700):
           steer_dc = int(steer_dc) + 2*delta_angle*int(angle_blue)*(max_steer_dc-min_steer_dc)/180 # was 2*0.19
           if steer_dc < min_steer_dc:
              #print("DC Warning Too Low for Steer")
              #print(steer_dc)
              steer_dc = min_steer_dc
           if steer_dc > max_steer_dc:
              #print("DC Warning Too High for Steer")
              #print(steer_dc)
              steer_dc = max_steer_dc

           car.set_steer_servo(steer_dc)



with open(name, 'w') as f:
   for i in range(len(time_need_array)):
      #f.write(f'{time_need_array[i]:.4f}\t {del_time_need[i]:.4f}\t {value[i]:.4f}\t {RPS[i]:.4f}\t {onezeros[i]}\t {delta[i]}\t {thr[i]}\n')
      f.write(f'{time_need_array[i]:.4f}\t {value[i]:.4f}\t {RPS[i]:.4f}\n')
f.close

plt.figure()
plt.plot(time_need_array[1:400],RPS[1:400]) #take the part needed
plt.grid(True)
plt.ylabel("RPS value")
plt.xlabel("time(s)")

name = 'car_' + str(args.rps) + 'rps.png'
plt.savefig(name)


plt.figure()
plt.plot(time_need_array[1:len(time_need_array)-1],value[1:len(time_need_array)-1])
plt.grid(True)
plt.ylabel("ADC value")
plt.xlabel("time(s)")
name = 'test_ADC' + str(round(duty)) + '_kp' + str(kp) + '_ki' + str(ki) + '_kd' + str(kd) + '.png'
plt.savefig(name)

car.set_motor(0)

