import argparse
import time
import cv2
import numpy as np
import math
from mod9_func import ultrasonic_init as u_init
from mod9_func import ultrasonic_read
import mod7_func as motor
from picar import PiCar, test, configure

parser = argparse.ArgumentParser(description='Data for this program.')
parser.add_argument('--debug', action='store_true', default = True,
   help='specifies if debug statements are printed')
parser.add_argument('--mock_car', action='store_true', default = False,
   help='initial to be False')
parser.add_argument('--tim', action='store', type=int, default=15,
   help='time for program to run in seconds')
parser.add_argument('--delay', action='store', type=float, default = 0.05,
   help='delay')
args = parser.parse_args()

car = PiCar(mock_car=False, threaded=True) #use mock_car=True for fake car and =False for real car.

delay = args.delay
tim = args.tim

#PWM_pin   = 36
#frequency = 50
#dc        = 2.1
angle_blue = 0
#max_dc = 12.2
#min_dc = 2.1

car.set_motor(85)
car.adc.read_adc(0)

start_time = time.time()
cur_time = start_time
delay_time = start_time

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




max_servo_dc = 10
min_servo_dc = -10
servo_dc = 0

max_steer_dc = 10 #need to configure these numbers
min_steer_dc = -10
steer_dc = 0


while (cur_time - start_time < tim):
   angle_blue = image_process(args.debug)
   distance = car.read_distance()
   if distance is None:
      distance = pre_distance
   pre_distance = distance
   cur_time = time.time()

   if (delay_time + delay < cur_time):
      delay_time = cur_time
      if (angle_blue != 360) and (-700 <= angle_blue <= 700): 
         steer_dc = int(steer_dc) + 2*0.19*int(0 if angle_blue is None else angle_blue)*(max_steer_dc-min_steer_dc)/180
         if steer_dc < min_steer_dc:
            print("DC Warning Too Low for Steer")
            print(steer_dc)
            steer_dc = min_steer_dc
         if steer_dc > max_steer_dc:
            print("DC Warning Too High for Steer")
            print(steer_dc)
            steer_dc = max_steer_dc

         car.set_steer_servo(steer_dc)


   if (distance >= 200):
       car.set_motor(75)
       print(f'Distance(70)={distance:0.2f} cm')
   if (distance >= 100 and distance < 200):
       car.set_motor(57)
       print(f'Distance(60)={distance:0.2f} cm')
   if (distance >= 60 and distance < 100):
       car.set_motor(40)
       print(f'Distance(60)={distance:0.2f} cm')
   if (distance >= 13 and distance < 60):
       car.set_motor(30)
       print(f'Distance(60)={distance:0.2f} cm')
   if (distance < 13 and distance >= 5):
       car.set_motor(0)
       print(f'Distance(0)={distance:0.2f} cm')
   if (distance < 5):
       car.set_motor(30,forward=False)
       print(f'Distance(40 back)={distance:0.2f} cm')


   if args.debug:
       print(f'Time={(cur_time-start_time):0.3f} \t Angle={int(0 if angle_blue is None else angle_blue)} \t Servo_DC={servo_dc:0.3f} \t Steer_DC={steer_dc:0.3f} \t \n')

car.set_motor(0)
