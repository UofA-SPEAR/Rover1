import RPi.GPIO as GPIO

direction_pin  = 13
motor_direction = 1

GPIO.setmode(GPIO.BOARD)

# Call before running everything
def init():
    pwm.start(0)
    GPIO.setup(12, GPIO.OUT) # PWM pin
    GPIO.setup(direction_pin, GPIO.OUT) # DIR pin
    pwm = GPIO.PWM(12, 100) # Set PWM frequency as 100Hz


# Set motor direction, a value of 0 reverses the motor
def setDirection(direction):
    motor_direction = direction


# Pass values from [-100, 100] as percentage to set motor speed/direction
def setMotorSpeed(speed):
    if (speed > 0):
        GPIO.output(direction_pin, motor_direction)
        pwm.ChangeDutyCycle(speed)
    else:
        GPIO.output(direction_pin, !motor_direction)
        pwm.ChangeDutyCycle( abs(speed) )


# Cleanup stuff
def quit:
    pwm.stop()
    GPIO.cleanup()
