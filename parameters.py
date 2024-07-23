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
sup_lim_servo = np.pi
inf_lim_servo = np.pi/2




# Impostazione dei pin GPIO per i servomotori
SERVO_PINS = [17, 18, 27, 22] # TODO da modificare in base ai pin utilizzati
start_angle = np. pi # TODO da impostare

# non so se servono, magari più avanti
x_des = 0
y_des = 0

