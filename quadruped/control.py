import math
import numpy as np

offset = 40e-3
OA = 45e-3
AB = 65e-3
BC = 87e-3


def sandbox(t):
    """
    python simulator.py -m sandbox

    Un premier bac à sable pour faire des expériences

    La fonction reçoit le temps écoulé depuis le début (t) et retourne une position cible
    pour les angles des 12 moteurs

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """

    # Par exemple, on envoie un mouvement sinusoidal
    targets = [0]*12
    targets[0] = np.sin(t)

    return targets

def direct(alpha, beta, gamma):
    """
    python simulator.py -m direct

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument la cible (alpha, beta, gamma) des degrés de liberté de la patte, et produit
    la position (x, y, z) atteinte par le bout de la patte

    - Sliders: les angles des trois moteurs (alpha, beta, gamma)
    - Entrées: alpha, beta, gamma, la cible (radians) des moteurs
    - Sortie: un tableau contenant la position atteinte par le bout de la patte (en mètres)
    """

    return [0., 0., 0.]

def inverse(x, y, z):
    """
    python simulator.py -m inverse

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument une position cible (x, y, z) pour le bout de la patte, et produit les angles
    (alpha, beta, gamma) pour que la patte atteigne cet objectif

    - Sliders: la position cible x, y, z du bout de la patte
    - Entrée: x, y, z, une position cible dans le repère de la patte (mètres), provenant du slider
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """
    
    theta0 = np.arctan2(y,x) 

    A = np.array([OA*np.cos(theta0), OA*np.sin(theta0), 0])

    AC = np.linalg.norm(np.array([x,y,z]) - A)
    
    theta3 = np.arccos(max(-1.0, min(1.0, (AB**2 + BC**2 - AC**2)/(2*AB*BC))))
    theta2 = np.pi - theta3

    L = np.sqrt((x-A[0])**2 + (y-A[1])**2)
    theta4 = np.arctan2(z, L)
    theta1 = theta4 + np.arccos(max(-1.0, min(1.0, (AB**2 + AC**2 - BC**2)/(2*AB*AC))))

    return [theta0, theta1, theta2]



def interpolate3d(values, t):
    index = 0
    while(values[index][0] < t):
        index += 1
        if (index >= len(values)):
            index = 0
            break

    if (values[index][0] == t):
        return values[index][1:]
    t = t%1
    return (1-t) * np.array(values[index-1][1:]) + t*np.array(values[index][1:])
    

def draw(t):
    """
    python simulator.py -m draw

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Le but est, à partir du temps donné, de suivre une trajectoire de triangle. Pour ce faire, on
    utilisera une interpolation linéaire entre trois points, et la fonction inverse précédente.

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """

    t = t%3
    points = [
        (0, 0.15, 0, 0.05),
        (1, 0.15, 0.06, 0),
        (2, 0.15, -0.06, 0)
    ]

    interp = interpolate3d(points, t)
    angle = inverse(interp[0], interp[1],interp[2])
    return angle


def legs(leg1, leg2, leg3, leg4):
    """
    python simulator.py -m legs

    Le robot est figé en l'air, on contrôle toute les pattes

    - Sliders: les 12 coordonnées (x, y, z) du bout des 4 pattes
    - Entrée: des positions cibles (tuples (x, y, z)) pour le bout des 4 pattes
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """

    targets = [0]*12

    B1 = 1/np.sqrt(2) *np.array([[-1, 1 ,0],
                                 [-1,-1,0],
                                [0,0,1]])
    leg1 = B1 @ np.array(leg1) - offset * np.array([1,0,0])
    P1 = np.array(inverse(leg1[0],leg1[1],leg1[2]))
    targets[0] = P1[0]
    targets[1] = P1[1]
    targets[2] = P1[2]
    
    
    B2 = 1/np.sqrt(2) *np.array([[-1, -1 ,0],
                                 [1,-1,0],
                                 [0,0,1]])
    leg2 = B2 @ np.array(leg2) - offset * np.array([1,0,0])
    P2 = np.array(inverse(leg2[0],leg2[1],leg2[2]))
    targets[3] = P2[0]
    targets[4] = P2[1]
    targets[5] = P2[2]
                  
      
    B3 = 1/np.sqrt(2) *np.array([[1, -1 ,0],
                                 [1,1,0],
                                 [0,0,1]])
    leg3 = B3 @ np.array(leg3) - offset * np.array([1,0,0])
    P3 = np.array(inverse(leg3[0],leg3[1],leg3[2]))
    targets[6] = P3[0]
    targets[7] = P3[1]
    targets[8] = P3[2]

                                  
    B4 = 1/np.sqrt(2) *np.array([[1, 1 ,0],
                                 [-1,1,0],
                                 [0,0,1]])
    leg4 = B4 @ np.array(leg4) - offset * np.array([1,0,0])
    P4 = np.array(inverse(leg4[0],leg4[1],leg4[2]))
    targets[9] = P4[0]
    targets[10] = P4[1]
    targets[11] = P4[2]
    
    

    return targets

def frame_inv(T):
    R = T[:3, :3] # On extrait la rotation
    t = T[:3, 3:] # On extrait la translation
    upper = np.hstack((R.T, -R.T @ t))
    lower = np.array([0., 0., 0., 1.])
    return np.vstack((upper, lower))

def Rx(alpha):
    return np.array([[1, 0, 0, 0],
                     [0, np.cos(alpha), -np.sin(alpha), 0],
                     [0, np.sin(alpha), np.cos(alpha), 0],
                     [0, 0, 0, 1]])

def Ry(alpha):
    return np.array([[np.cos(alpha), 0, np.sin(alpha), 0],
                     [0, 1, 0, 0],
                     [-np.sin(alpha), 0, np.cos(alpha), 0],
                     [0, 0, 0, 1]])
        
def Rz(alpha):
    return np.array([[np.cos(alpha), -np.sin(alpha), 0, 0],
                     [np.sin(alpha), np.cos(alpha), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def translation(vector):
    return np.array([[1, 0, 0, vector[0]],
                     [0, 1, 0, vector[1]],
                     [0, 0, 1, vector[2]],
                     [0, 0, 0, 1]])



def walk(t, speed_x, speed_y, speed_rotation):
    """
    python simulator.py -m walk

    Le but est d'intégrer tout ce que nous avons vu ici pour faire marcher le robot

    - Sliders: speed_x, speed_y, speed_rotation, la vitesse cible du robot
    - Entrée: t, le temps (secondes écoulées depuis le début)
            speed_x, speed_y, et speed_rotation, vitesses cibles contrôlées par les sliders
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """

    """
    B1 = translation(offset * np.array([1, 0, 0])) @ Rz(-3/4 * np.pi)
    
    B2 = translation(offset * np.array([1, 0, 0])) @ Rz(3/4 * np.pi)
                    
    B3 = translation(offset * np.array([1, 0, 0])) @ Rz(1/4 * np.pi)

    B4 = translation(offset * np.array([1, 0, 0])) @ Rz(-1/4 * np.pi)

    C1 = frame_inv(B1)
    C2 = frame_inv(B2)
    C3 = frame_inv(B3)
    C4 = frame_inv(B4)

    targets = [0]*12
    t = t%3
    points = [
        (0, 0.15, 0, 0.05),
        (1, 0.15, 0.06, 0),
        (2, 0.15, -0.06, 0)
    ]

    
    pointsleg1 = [
        (0, C1 @ np.array([points[0][1], points[0][2], points[0][3] , 1])),
        (1, C1 @ np.array([points[1][1], points[1][2], points[1][3] , 1]) ),
        (2, C1 @ np.array([points[2][1], points[2][2], points[2][3] , 1]))
    ]

    pointsleg2 = [
        (0, C2 @ np.array([points[0][1], points[0][2], points[0][3] , 1])),
        (1, C2 @ np.array([points[1][1], points[1][2], points[1][3] , 1]) ),
        (2, C2 @ np.array([points[2][1], points[2][2], points[2][3] , 1]))
    ]
    pointsleg3 = [
        (0, C3 @ np.array([points[0][1], points[0][2], points[0][3] , 1])),
        (1, C3 @ np.array([points[1][1], points[1][2], points[1][3] , 1]) ),
        (2, C3 @ np.array([points[2][1], points[2][2], points[2][3] , 1]))
    ]
    pointsleg4 = [
        (0, C4 @ np.array([points[0][1], points[0][2], points[0][3] , 1])),
        (1, C4 @ np.array([points[1][1], points[1][2], points[1][3] , 1]) ),
        (2, C4 @ np.array([points[2][1], points[2][2], points[2][3] , 1]))
    ]

    

    P1 = interpolate3d(pointsleg1, t)
    P2 = interpolate3d(pointsleg2, t)
    P3 = interpolate3d(pointsleg3, t)
    P4 = interpolate3d(pointsleg4, t)
"""
    '''
    D = BC/2 *np.array([np.cos(100 * speed_rotation * t), np.sin(100 * speed_rotation * t), 0-1.5])
    P1 = 0.8 * (OA + AB + offset)* np.array([-1,1,0]) + D
    P2 = 0.8 * (OA + AB + offset)* np.array([-1,-1,0]) + D
    P3 = 0.8 * (OA + AB + offset)* np.array([1,-1,0]) + D
    P4 = 0.8 * (OA + AB + offset)* np.array([1,1,0]) +  D
    '''
    t%3

    leg1 = np.array([-0.1, 0.1, -0.05])
    leg2 = np.array([-0.1, -0.1, -0.05])
    leg3 = np.array([0.1, -0.1, -0.05])
    leg4 = np.array([0.1, 0.1, -0.05])

    off1 = np.array([-0.05,0,-0.05])
    off2 = np.array([0.1,0,-0.05])


    pointsleg1 = [
        (0, leg1),
        (1, leg1 + off1),
        (2, leg1 + off2)
    ]
    pointsleg2 = [
        (0, leg2 + off1),
        (1, leg2 + off2),
        (2, leg2)
    ]

    pointsleg3 = [
        (0, leg3 + off2),
        (1, leg3),
        (2, leg3 + off1)
    ]

    pointsleg4 = [
        (0, leg4),
        (1, leg4 + off1),
        (2, leg4 + off2)
    ]

    P1 = interpolate3d(pointsleg1, t)
    P2 = interpolate3d(pointsleg2, t)
    P3 = interpolate3d(pointsleg3, t)
    P4 = interpolate3d(pointsleg4, t)
    return legs(P1[0], P2[0], P3[0], P4[0])

if __name__ == "__main__":
    print("N'exécutez pas ce fichier, mais simulator.py")
