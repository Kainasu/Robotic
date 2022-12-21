import matplotlib.pyplot as pyplot
import numpy as np

def interpolate(values, t):
    index = 0    
    while(values[index][0] < t):
        index += 1
    if (values[index][0] == t):
        return values[index][1]
    a = (values[index][1]-values[index-1][1])/(values[index][0]-values[index-1][0])
    b = values[index][1] - a * values[index][0]
    return a*t+b

values = [ # Les valeurs à interpoler
    (0, 0),
    (1.2, 6),
    (2.5, -3),
    (3.3, -2),
    (4.2, -2),
    (5, 0)
]

ts = np.linspace(0, 5, 100)
vs = [interpolate(values, t) for t in ts]

pyplot.scatter(ts, vs)
pyplot.show()

def R(alpha):
    return np.array([[np.cos(alpha), -np.sin(alpha)],
                     [np.sin(alpha), np.cos(alpha)]])


def frame_inv(T):
    R = T[:2, :2] # On extrait la rotation
    t = T[:2, 2:] # On extrait la translation
    upper = np.hstack((R.T, -R.T @ t))
    lower = np.array([0., 0., 1.])
    return np.vstack((upper, lower))
