import numpy as np

color = 'blue' #l'arancione viene riconosciuto come blu, devo aggiustare i parametri della telecamera

a = 10
b = 10
f = 10
g = 10
l1 = 10
l2 = 10
dz = 10

# imposto i motori tra 90 e 180 
# TODO da verificare i valori corretti degli angoli
inf_lim_servo = 0
sup_lim_servo = np.pi/2

inf_lim_duty_cycle, sup_lim_duty_cycle = 2500, 5350  # Intevallo del duty cycle del servomotore

# Impostazione dei pin GPIO per i servomotori
PCA9685_PINS = [0, 1, 2, 3] # TODO da modificare in base ai pin utilizzati

# non so se servono, magari più avanti
x_des = 0
y_des = 0

