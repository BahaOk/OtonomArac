from gpiozero import Motor, PWMOutputDevice
import time

# Sol motor: INA = 22, INB = 27, ENA = 18
motor_left = Motor(forward=22, backward=27)
pwm_left = PWMOutputDevice(18)

# Sağ motor: INC = 23, IND = 24, ENB = 19
motor_right = Motor(forward=23, backward=24)
pwm_right = PWMOutputDevice(19)

# Test başlasın
print("Motorlar ileri yönde dönüyor...")
motor_left.forward()
motor_right.forward()

pwm_left.value = 1.0
pwm_right.value = 1.0

time.sleep(3)

print("Motorlar duruyor...")
pwm_left.value = 0
pwm_right.value = 0
