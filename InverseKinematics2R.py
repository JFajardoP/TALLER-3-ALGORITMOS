import numpy
import math
from sympy import *
from roboticstoolbox import *
from spatialmath.base import *

def InverseKinematics2R(l1, l2, Px, Py):
    # b es la distancia del origen al punto (hipotenusa)
    b = sqrt(Px**2 + Py**2)

    # Theta 2 (Equivalente al cálculo del codo en un sistema 2R)
    # Usamos ley de cosenos para el ángulo entre l1 y l2
    cos_theta2 = (b**2 - l1**2 - l2**2) / (2 * l1 * l2)
    sen_theta2 = sqrt(1 - (cos_theta2)**2)
    theta2 = float(atan2(sen_theta2, cos_theta2))
    
    # Theta 1 (Ángulo de la base)
    # alpha es el ángulo al punto, phi es la compensación por la flexión del codo
    alpha = math.atan2(Py, Px)
    phi = math.atan2(l2 * sen_theta2, l1 + l2 * cos_theta2)
    theta1 = float(alpha - phi)
    
    # Ajuste de rango (mismo formato que tenías)
    if theta1 <= -numpy.pi:
        theta1 = (2 * numpy.pi) + theta1

    # Impresiones manteniendo tu estilo
    print(f'theta 1 = {numpy.rad2deg(theta1):.4f}')
    print(f'theta 2 = {numpy.rad2deg(theta2):.4f}')
    
    return theta1, theta2

# Para probarlo con los datos de tu robot de MATLAB:
# l1 = 10.5, l2 = 10.5, Px = 21, Py = 0
# InverseKinematics2R(10.5, 10.5, 21, 0)
