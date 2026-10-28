import numpy as np
np.set_printoptions(precision=3, suppress=True)

L_1 = 0.1400
L_2 = 0.3500
L_3 = 0.0500
L_4 = 0.2500
L_5 = 0.1500
BETA = np.arctan(L_3/L_2)

LAMBDA_1 = L_1
LAMBDA_2 = np.sqrt(L_2**2 + L_3**2)
LAMBDA_3 = L_4 + L_5

def forward_kin(phi):
    '''
    Implement a forward kinematics function for the QArm

    Input:
        phi: the joint angles in radians, in order of base, shoulder, elbow, wrist

    Output:
        location: the end effector coordinate system origin position in the base {0} coordinate system
        rotation: rotation matrix from the end effector {4} coordinate system to the base {0} coordinate system
    '''

    phi = np.array(phi, dtype=float, copy=True)

    # == your code here ==

    def dh(theta, d, a, alpha):
        # Homogeneous transform from frame {i-1} to frame {i}
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d   ],
            [0,   0,      0,     1   ],
        ])

    # DH table: theta, d, a, alpha
    T_01 = dh(phi[0],                  LAMBDA_1, 0,        -np.pi/2)
    T_12 = dh(phi[1] - np.pi/2 + BETA, 0,        LAMBDA_2,  0)
    T_23 = dh(phi[2] - BETA,           0,        0,        -np.pi/2)
    T_34 = dh(phi[3],                  LAMBDA_3, 0,         0)

    T_04 = T_01 @ T_12 @ T_23 @ T_34

    location = T_04[:3, 3]
    rotation = T_04[:3, :3]

    # == end of your code ==

    return location, rotation



if __name__ == '__main__':

    # Function input is radians

    phi = np.radians(np.array([0, -11, -75, 0]))
    location, rotation = forward_kin(phi)
    print('Input 1 -> location:', location, 'rotation:', rotation)
    
    phi = np.radians(np.array([-90, 73.5, -75, -50]))
    location, rotation = forward_kin(phi)
    print('Input 2 -> location:', location, 'rotation:', rotation)

    phi = np.radians(np.array([-90, 73.5, -75, 0]))
    location, rotation = forward_kin(phi)
    print('Input 3 -> location:', location, 'rotation:', rotation)

    phi = np.radians(np.array([90, -85, 75, 90]))
    location, rotation = forward_kin(phi)
    print('Input 4 -> location:', location, 'rotation:', rotation)