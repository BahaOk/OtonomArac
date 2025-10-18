
from gpiozero import AngularServo
from time import sleep

class ServoKontrol:
    # Initialize the servo on GPIO pin 14
    # min_pulse_width and max_pulse_width may need to be adjusted for your servo
    def __init__(self):
        servo = AngularServo(17, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)

    # Function to set the servo angle
    def set_angle(angle):
        servo.angle = angle
        sleep(1)
        
    def servo_start():
        angle = 100
        set_angle(angle)
