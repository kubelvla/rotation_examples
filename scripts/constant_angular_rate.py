import numpy as np
from scipy.spatial.transform import Rotation

# in this script, the quat is formatted as [x, y, z, w]! We need it like that for Unlike the Madgwick implementation!

def qDot(q, w):
    q_dot = np.array([0.0, 0.0, 0.0, 1.0])

    q_dot[0] = 0.5 * ( q[3] * w[0] + q[1] * w[2] - q[2] * w[1])    # x!!
    q_dot[1] = 0.5 * ( q[3] * w[1] - q[0] * w[2] + q[2] * w[0])    # y!!
    q_dot[2] = 0.5 * ( q[3] * w[2] + q[0] * w[1] - q[1] * w[0])    # z!!
    q_dot[3] = 0.5 * (-q[0] * w[0] - q[1] * w[1] - q[2] * w[2])    # w!!
    return q_dot


q_current = np.array([0.0, 0.0, 0.0, 1.0])
w = np.array([1, -1, 2])
T = 1.5
samples = 10  # type: int



# Incrementally, by computing q_dot

dt = T/samples

for i in range(samples):
    q_current += dt * qDot(q_current, w)
    att_now = Rotation.from_quat(q_current)
    eul_now = att_now.as_euler('zyx', degrees=True)
    print(eul_now)

print("-------------------------------------------------------")

# In one shot, eq. (214) from Sola, 2015
q_direct_vect = w*(1.0/np.linalg.norm(w))*np.sin(np.linalg.norm(w)*(0.5*T))
q_direct_scalar = np.array([np.cos(np.linalg.norm(w)*(0.5*T))]);
q_direct = np.hstack((q_direct_vect, q_direct_scalar))


att_direct = Rotation.from_quat(q_direct)
eul_direct = att_direct.as_euler('zyx', degrees=True)
print(eul_direct)


print("-------------------------------------------------------")

# In one shot, directly from a rotation vector
rot_vector = w * T
att_direct_from_rotvec = Rotation.from_rotvec(rot_vector)
eul_direct_from_rotvec = att_direct_from_rotvec.as_euler('zyx', degrees=True)
print(eul_direct_from_rotvec)



