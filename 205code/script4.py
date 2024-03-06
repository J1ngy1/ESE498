# import module
from picar import PiCar
import cv2
import argparse

parser = argparse.ArgumentParser(description='Data for this program.')
parser.add_argument('--mock_car', action='store_true', 
                    help='If not present, run on car, otherwise mock hardware')
parser.add_argument('--debug', action='store_true', 
                    help='If not present, set debug to false')
args = parser.parse_args()

# test on actual hardware
car = PiCar(mock_car=args.mock_car, threaded=True)

img = car.get_image()
if not (img is None):
    cv2.imwrite("testCar.png", img)
else:
    if DEBUG: print("Warning: image not ready.")

distance = 0
tempDistance = car.read_distance()
if not (tempDistance is None):
    distance = tempDistance
    print(f'Distance: {distance:.3f}')
else:
    if args.debug: print("Warning: distance reading not ready.")
