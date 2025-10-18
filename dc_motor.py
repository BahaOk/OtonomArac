from gpiozero import Motor, PWMOutputDevice

class Vehicle:
    def __init__(self):
        # Sol motor (INA, INB)
        self.motor_left = Motor(forward=27, backward=22)
        # Sağ motor (INC, IND)
        self.motor_right = Motor(forward=23, backward=24)

        # ENA ve ENB PWM çıkışları
        self.pwm_left = PWMOutputDevice(18)  # ENA
        self.pwm_right = PWMOutputDevice(19) # ENB

      #  self.stop()  # İlk başta motorlar durur

    def set_speed(self, left_speed, right_speed):
        self.pwm_left.value = left_speed
        self.pwm_right.value = right_speed

    def forward_fast(self):
        self.motor_left.forward()
        self.motor_right.forward()
        self.set_speed(1.0, 1.0)

    def forward_normal(self):
        self.motor_left.forward()
        self.motor_right.forward()
        self.set_speed(0.8, 0.8)

    def forward_slow(self):
        self.motor_left.forward()
        self.motor_right.forward()
        self.set_speed(0.5, 0.5)

    def backward(self):
        self.motor_left.backward()
        self.motor_right.backward()
        self.set_speed(1.0, 1.0)

    def stop(self):
        self.set_speed(0, 0)
