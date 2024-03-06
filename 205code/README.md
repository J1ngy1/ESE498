# Template_Module_10

This module is for the final project.

## It includes
- Several PICAR_CONFIG files for the cars
- Several example data files, good and one bad
- check_servos.py - This program is good for checking the span of the servos for a given configuration file

## Deliverables
- objective1.py # Data collection for speed calculation
- objective2.py # Goes to the wall without hitting it
- objective3.py # Goes to an object at a given speed
- objective4.py # If you did it
- car_noload_3rps.txt # car on table running at 4rps target
- car_[RPSVAL]rps.txt   # car on floor going at RPSVAL target, it will have starting and stopping values

## NOTE:
For data files, data_example.txt and data_example2.txt are examples of acceptable data files.  
They include the sampling time and note that the second file includes more data than what was 
being asked for, which is ok, as long as the required elements are present.

data_example_bad.txt has two problems.  
- it is missing the sampling period to start the first line
- note that the time samples are not equally spaced

Also, do not include extra lines without data.  So if you have 10,000 element arrays for example,
but only 2000 lines that contain actual values, do not inculde 8000 lines of 0's.
