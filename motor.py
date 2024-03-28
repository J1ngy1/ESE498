#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

motor_pin = 18  # PWM pin connected to the motor
steer_pin = 12  # PWM pin connected to the steering servo

GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_pin, GPIO.OUT)
GPIO.setup(steer_pin, GPIO.OUT)
motor_pwm = GPIO.PWM(motor_pin, 50)  # 50 Hz
steer_pwm = GPIO.PWM(steer_pin, 50)  # 50 Hz
motor_pwm.start(0)  # Start with motor off
steer_pwm.start(7.5)  # Start with steering centered

def motor_control_callback(data):
    speed, turn = map(float, data.data.split())
    rospy.loginfo("Received control command - speed: %f, turn: %f", speed, turn)

    motor_duty_cycle = speed * 100  # Assuming speed is between -1 and 1
    steer_duty_cycle = 7.5 + turn * 2.5  # Assuming turn is between -1 and 1
    motor_pwm.ChangeDutyCycle(motor_duty_cycle)
    steer_pwm.ChangeDutyCycle(steer_duty_cycle)

def motor_control_node():
    rospy.init_node('motor_control_node', anonymous=True)
    rospy.Subscriber('motor_control', String, motor_control_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        motor_control_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        motor_pwm.stop()
        steer_pwm.stop()
        GPIO.cleanup()
