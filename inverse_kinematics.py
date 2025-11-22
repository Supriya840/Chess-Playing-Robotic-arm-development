import numpy as np

# link lengths (mm)
L1, L2, L3 = 50, 100, 100  

def inverse_kinematics(x, y, z, pitch=0):
    # Base
    theta1 = np.arctan2(y, x)

    # Planar reduction
    r = np.sqrt(x**2 + y**2)
    z_p = z - L1

    # Elbow (law of cosines)
    cos_theta3 = (r**2 + z_p**2 - L2**2 - L3**2) / (2 * L2 * L3)
    theta3 = np.arccos(np.clip(cos_theta3, -1, 1))

    # Shoulder
    theta2 = np.arctan2(z_p, r) - np.arctan2(L3*np.sin(theta3), L2 + L3*np.cos(theta3))

    # Wrist (to maintain pitch)
    theta4 = pitch - (theta2 + theta3)

    return np.degrees([theta1, theta2, theta3, theta4])

# Example: target at (120, 0, 50)
angles = inverse_kinematics(150, 30, 50, pitch=0)
print("Joint angles (deg):", angles)
