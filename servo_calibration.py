# impostare manualmente il valore del duty cycle in modo da rilevare i valori corrispondenti a 0 e 90 grandi
# in seguito salvare questi valori e utilizzarli per mappare gli angoli tra 0 e 90

import board
import busio
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # Frequenza per servomotori (50 Hz)
servo_channel = 12
while True:
    duty = float(input("Duty cycle: "))
    duty = round(duty)
    print(duty)
    pca.channels[servo_channel].duty_cycle = duty

# di norma assume valori tra 2000 e 10000 
# trovati i valori impostare nel inf_lim_duty_cycle e sup_lim_duty_cycle nel file parameters.py