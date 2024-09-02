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

inf_lim_duty_cycle, sup_lim_duty_cycle = 1250, 4500  # Intevallo del duty cycle del servomotore
inf_dc_1, sup_dc_1 = 1425, 4700
inf_dc_2, sup_dc_2 = 1650, 4850
inf_dc_3, sup_dc_3 = 1725, 4925
inf_dc_4, sup_dc_4 = 1550, 4725 # 12
# Impostazione dei pin GPIO per i servomotori
PCA9685_PINS = [0, 4, 8, 12] # TODO da modificare in base ai pin utilizzati

inf_dc = [1425, 1650, 1725, 1550]
sup_dc = [4700, 4850, 4925, 4725]

# non so se servono, magari più avanti
x_des = 0
y_des = 0


A = 0
B = np.pi/2
