#!/usr/bin/env python

import rospy
from rover1.msg import input_rope

import RPi.GPIO as GPIO

direction_pin  = 13

GPIO.setmode(GPIO.BOARD)
rope_subscriber = None

# Call before running everything
def initMotor():
    pwm.start(0)
    GPIO.setup(12, GPIO.OUT) # PWM pin
    GPIO.setup(direction_pin, GPIO.OUT) # DIR pin
    pwm = GPIO.PWM(12, 100) # Set PWM frequency as 100Hz



# Pass values from [-100, 100] as percentage to set motor speed/direction
def setMotorSpeed(speed):
    GPIO.output(direction_pin, copysign(1, speed))
    pwm.ChangeDutyCycle(abs(speed))


# Cleanup stuff
def closeMotor():
    pwm.stop()
    GPIO.cleanup()


def handleRopeCallback(data):
    setMotorSpeed(data.speed)


if __name__ == "__main__ ":
    rospy.init_node('rope_node', log_level=rospy.INFO)
    rospy.loginfo("Initializing rope node")
    initMotor()

    sensor_subscriber = rospy.Subscriber('/rope_sub', input_rope, handleRopeCallback)

    rospy.spin()

