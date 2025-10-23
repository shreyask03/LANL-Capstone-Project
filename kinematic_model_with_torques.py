import visual_kinematics.RobotSerial as RS
import visual_kinematics as vk
import numpy as np

radius = 0.01; L1 = .2; L2 = 0.1; L3 = 0.1; L4 = 0.1; L_end = .1

dh = np.array([ [radius, 0.0, 0.5*np.pi, 0], # 1: pitch (z0 // z1) 
               [L1, 0.0, 0.5*np.pi, 0.0], # 2: roll 
               [L2, 0.0, 0.5*np.pi, 0.0], # 3: pitch 
               [L3, 0, .5*np.pi, .5*np.pi], # 4: roll 
               [0, L4, -0.5*np.pi, 0.5*np.pi], # 5: yaw 
               [0, L_end, 0, 0.0], # 6: pitch # 
               ])



robot = RS.RobotSerial(dh)

always_0 = 0

theta = np.array([0, 0, np.pi, 0, always_0, 0])
f = robot.forward(theta)
robot.show()

# Manually calculate joint positions via DH

def _dh_T(d, a, alpha, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ ct, -st*ca,  st*sa, a*ct],
        [ st,  ct*ca, -ct*sa, a*st],
        [  0,     sa,     ca,    d],
        [  0,      0,      0,    1]
    ])

def joint_positions_from_visual_kinematics_DH(dh, q):
    """
    dh: (n,4) array in visual_kinematics order [d, a, alpha, theta_offset]
    q : (n,) joint variables (rad). Use d += q[i] for prismatic joints instead.
    returns: (n+1,3) array of origins: [base, J1, J2, ..., Jn]
    """
    T = np.eye(4)
    pts = [T[:3, 3].copy()]
    for i in range(dh.shape[0]):
        d, a, alpha, theta0 = dh[i]
        T = T @ _dh_T(d, a, alpha, theta0 + q[i])
        pts.append(T[:3, 3].copy())
    return np.round(np.vstack(pts), 3)


q = theta.astype(float)

P = joint_positions_from_visual_kinematics_DH(dh, q)
# rows correspond to base, J1, ..., Jn

print("Joint origins (J0...J5):\n", P[1:-1])

### Joint torques

# 5 lb tip load -> tip force in Newtons (assumes gravity along -x of base frame)
MASS_LBS = 5.0
g = 9.80665
F_tip = MASS_LBS * 0.45359237 * np.array([-g, 0.0, 0.0])  # [N]


# Vector from each joint origin to the end-effector:
r = P[-1] - P[1:-1]                 # shape (n,3)

# Moment at each joint due to the tip force: M_i = r_i x F
Mvec = np.cross(r, F_tip)          # shape (n,3) [N·m]


print("\nTip force [N]:", np.round(F_tip, 3))
print("\nMoment vectors at joints [N·m] (Mx, My, Mz):\n", np.round(Mvec, 3))
