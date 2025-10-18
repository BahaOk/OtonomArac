import RPi.GPIO as GPIO
import time

class DcMotor:

    def __init__(self):
        self.ENA = 4
        self.IN1 = 17
        self.IN2 = 27
        self.gpio = GPIO
        self.setup()

    def setup(self):

        self.gpio.setmode(self.gpio.BOARD)
        self.gpio.setup(self.ENA, self.gpio.OUT)
        self.gpio.setup(self.IN1, self.gpio.OUT)
        self.gpio.setup(self.IN2, self.gpio.OUT)
        self.motor = self.gpio.PWM(self.ENA, 1000)
        self.motor.start(0)

    def power_on(self):
        self.motor.start(20)
        time.sleep(1)
        self.motor.stop()
        time.sleep(0.04)
        #self.gpio.output(self.ENA, self.gpio.LOW)

    def power_off(self):
        self.gpio.output(self.ENA, self.gpio.LOW)
        self.gpio.cleanup()

    def stop(self):
        self.gpio.cleanup()

    def power_reverse_on(self):
        self.gpio.output(self.ENA, self.gpio.HIGH)
        self.gpio.output(self.IN1, self.gpio.LOW)
        self.gpio.output(self.IN2, self.gpio.HIGH)

if __name__ == "__main__":
    dcMotor = DcMotor()
    dcMotor.power_on()
