import numpy as np
import parameters as p



def pitch_roll_to_four_motor_angles(pitch, roll, dz=p.dz):
    # si può modificare dz nel caso si voglia modificare l'altezza
    alfa = roll
    beta = pitch
    Tse = np.array([
        [np.cos(beta),    np.sin(alfa)*np.sin(beta),    np.cos(alfa)*np.sin(beta),    0],
        [0,               np.cos(alfa),                 -np.sin(alfa),                0],
        [-np.sin(beta),   np.sin(alfa)*np.cos(beta),    np.cos(alfa)*np.cos(beta),   dz],
        [0,               0,                            0,                            1]
        ])
    b1 = np.array([[-p.a], [-p.b], [0], [1]])
    b2 = np.array([[p.a], [-p.b], [0], [1]])
    b3 = np.array([[p.a], [p.b], [0], [1]])
    b4 = np.array([[-p.a], [p.b], [0], [1]])
    
    aE1 = np.array([[-p.f], [-p.g], [0], [1]])
    aE2 = np.array([[p.f], [-p.g], [0], [1]])
    aE3 = np.array([[p.f], [p.g], [0], [1]])
    aE4 = np.array([[-p.f], [p.g], [0], [1]])
    
    a1 = np.dot(Tse, aE1)
    a2 = np.dot(Tse, aE2)
    a3 = np.dot(Tse, aE3)
    a4 = np.dot(Tse, aE4)
    
    p1 = a1-b1
    p2 = a2-b2 
    p3 = a3-b3
    p4 = a4-b4
    
    teta1 = np.pi - (np.arcsin(p1[3, 0]/np.linalg.norm(p1))  +  np.arccos(((np.linalg.norm(p1))**2 + p.l1**2 - p.l2**2)/(2*np.linalg.norm(p1)*p.l1)))
    teta2 = np.pi - (np.arcsin(p2[3, 0]/np.linalg.norm(p2))  +  np.arccos(((np.linalg.norm(p2))**2 + p.l1**2 - p.l2**2)/(2*np.linalg.norm(p2)*p.l1)))
    teta3 = np.pi - (np.arcsin(p3[3, 0]/np.linalg.norm(p3))  +  np.arccos(((np.linalg.norm(p3))**2 + p.l1**2 - p.l2**2)/(2*np.linalg.norm(p3)*p.l1)))
    teta4 = np.pi - (np.arcsin(p4[3, 0]/np.linalg.norm(p4))  +  np.arccos(((np.linalg.norm(p4))**2 + p.l1**2 - p.l2**2)/(2*np.linalg.norm(p4)*p.l1)))
    return teta1, teta2, teta3, teta4
# t1, t2, t3, t4 = pitch_roll_to_four_motor_angles(0, 0)
# print(f"t1: {t1}\nt2: {t2}\nt3: {t3}\nt4: {t4}\n")
