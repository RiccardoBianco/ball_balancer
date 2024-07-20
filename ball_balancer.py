import time
import parameters as p
import inv_kin as ik
import cv2
import numpy as np
import pigpio


# inizializzazione della telecamera (0 per quella standard)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Errore nell'apertura della videocamera")
    exit()

# parametri del PID
Kp = 0
Ki = 0
Kd = 0

# inizializzo le variabili che mi servono per il primo ciclo di pid
previous_time = time.time()
cumError_x = 0
cumError_y = 0
lastError_x = 0
lastError_y = 0

x_center = 0
y_center = 0

# Connessione al daemon pigpiod
pi = pigpio.pi()
if not pi.connected:
    print("Impossibile connettersi al deamon pigpiod")
    exit()




def PID_x(input, desired, current_time):
    global cumError_x
    global lastError_x
    global previous_time
    
    elapsed_time = current_time-previous_time
    
    error = desired-input
    cumError_x += error*elapsed_time
    diffError = (error-lastError_x)/elapsed_time
    
    output_x = Kp*error + Ki*cumError_x + Kd*diffError
    
    lastError_x = error
    return output_x

def PID_y(input, desired, current_time):
    global cumError_y
    global lastError_y
    global previous_time
    
    elapsed_time = current_time-previous_time
    
    error = desired-input
    cumError_y += error*elapsed_time
    diffError = (error-lastError_y)/elapsed_time
    
    output_y = Kp*error + Ki*cumError_y + Kd*diffError
    
    lastError_y = error
    return output_y

def PID(x_input, y_input, x_des, y_des):
    global previous_time
    # salvo il tempo corrente
    current_time = time.time()
    
    # calcolo il valore del PID per la x e per la y
    x_output = PID_x(x_input, x_des, current_time)
    y_output = PID_y(y_input, y_des, current_time)
    
    # aggiorno il tempo corrente come nuovo tempo precedente
    previous_time = current_time
    
    return x_output, y_output

def set_servo_to_zero():
    # Configura i pin GPIO come uscite PWM
    for pin in p.SERVO_PINS:
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.set_servo_pulsewidth(pin, 0)  # Spegni il servo all'inizio
  
    
def limitate_servo(angle):
    if angle>p.sup_lim_servo:
        angle = p.sup_lim_servo
    if angle<p.inf_lim_servo:
        angle = p.inf_lim_servo
    return angle

# Funzione per convertire un angolo in duty cycle
def angle_to_duty_cycle(angle):
    return (np.rad2deg(angle) / 180.0) * 10 + 2.5


def move_servo(pin, angle):
    duty_cycle = angle_to_duty_cycle(angle)
    pi.set_servo_pulsewidth(pin, duty_cycle * 1000)  # Imposta il pulse width in microsecondi


def move_all_servo(angles):
    # limito il movimento dei motori entro il limite superiore e inferiore
    for angle in angles: 
        angle = limitate_servo(angle) # TODO verificare se funziona l'assegnazione in questo modo

    # muovo i 4 motori nella posizione desiderata
    for i in range(len(p.SERVO_PINS)):
        try:
            move_servo(p.SERVO_PINS[i], angles[i])
        except:
            print("Impossibile muovere il servo all'angolo desiderato")
        finally:
            # Spegni i servomotori
            for pin in p.SERVO_PINS:
                pi.set_servo_pulsewidth(pin, p.start_angle) # riporto tutti i motori a zero

    # Termina la connessione al daemon pigpiod
    pi.stop()
    return
        
def ball_tracker(color):
    global x_center
    global y_center
    
     # Leggi un nuovo frame
    ret, frame = cap.read()
    if not ret:
        print("Errore nella lettura del frame")
        return -1, -1
    
    altezza, larghezza, _ = frame.shape

    # Calcola il centro dell'immagine
    x_center = larghezza // 2
    y_center = altezza // 2
    x, y = x_center, y_center
    
    # Converti il frame da BGR a HSV (spazio dei colori più adatto per il riconoscimento del colore)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
     # Definisci il range di colori arancioni nella scala HSV
    if color == 'orange':
        lower = np.array([5, 150, 150])
        upper = np.array([15, 255, 255])
        bgr_color = (0, 165, 255)
    elif color == 'yellow':
        # Definisci il range di colori gialli nella scala HSV
        lower = np.array([20, 100, 100])
        upper = np.array([30, 255, 255])
        bgr_color = (0, 255, 255)
    
    # Applica una maschera per ottenere solo i pixel nell'intervallo di colori gialli
    mask = cv2.inRange(hsv, lower, upper)
    
    # Trova i contorni nella maschera
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Trova il contorno più grande (presumibilmente la pallina)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        # Trova il centro e il raggio del cerchio che circonda il contorno
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
        if radius>=15:
            center = (int(x), int(y))
            radius = int(radius)
            
            # Disegna il cerchio intorno alla pallina rilevata
            cv2.circle(frame, center, radius, (255, 0, 255), 2)
            cv2.circle(frame, center, 1, (255, 0, 255), 2)
    
    # Mostra il frame risultante
    cv2.imshow('Rilevamento Pallina Gialla', frame)
    return round(x, 2), round(y, 2)
    

    


def main():
    cnt = 0
    set_servo_to_zero()

    while True: # finche arrivano dati
        # leggi i dati dalla telecamera
        x_input, y_input = ball_tracker(p.color)
        if x_input==-1 or y_input==-1:
            break
        cnt +=1
        print(f"{cnt} --> x = {x_input}; y = {y_input}")


        # identificare il centro della piattaforma espresso come pixel dell'immagine
        # assicurarsi che gli assi x e y dell'immagine coincidano con quelli della piattaforma (altrimenti modificare quelli dell'immagine)
        # 0,0 corrisponde all'angolo in alto a sinistra --> se i miei assi sono diversi nell'immagine acquisita devo assicurarmi di farli coincidere
        
        
        
        # calcola il PID con x e y
        x_output, y_output = PID(x_input, y_input, x_center, y_center)
        
        # trasforma l'output x e y in angoli di rotazione pitch e roll (Inverse Kinematics)
        pitch = x_output # eventualmente trasformata in qualche modo
        roll = y_output # eventualemente trasformata in qualche modo
        angles = ik.pitch_roll_to_four_motor_angles(pitch, roll)
        
        # muovo i motori alla posizione desiderata
        move_all_servo(angles)
        
        # Esci dal loop premendo 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    
    # release delle risorse
    cap.release()
    cv2.destroyAllWindows()

    # Spegni i servomotori
    for pin in p.SERVO_PINS:
        pi.set_servo_pulsewidth(pin, p.start_angle)

    # Termina la connessione al daemon pigpiod
    pi.stop()
    
main()