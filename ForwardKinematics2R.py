import numpy
import math
from sympy import *
from roboticstoolbox import *
from spatialmath.base import *

def ForwardKinematics2R(l1, l2, q1, q2):
    
    R = []
    # Para un robot 2R plano (como el que usas en MATLAB):
    # Eslabón 1: rotación q1, longitud l1
    R.append(RevoluteDH(d=0, alpha=0, a=l1, offset=0))
    # Eslabón 2: rotación q2, longitud l2
    R.append(RevoluteDH(d=0, alpha=0, a=l2, offset=0))

    Robot = DHRobot(R, name='Bender_2R')
    print(Robot)

    # Ajustamos los límites de la gráfica y los grados de libertad a 2 [q1, q2]
    Robot.plot([q1, q2], limits=[-25, 25, -25, 25, -25, 25])

    # Calculamos la cinemática directa (Matriz de Transformación Homogénea)
    MTH = Robot.fkine([q1, q2])
    print(MTH)
    
    # Mostramos la orientación del extremo
    print(f'Roll, Pitch, Yaw = {tr2rpy(MTH.R, "deg", "zyx")}')
    
    return MTH
