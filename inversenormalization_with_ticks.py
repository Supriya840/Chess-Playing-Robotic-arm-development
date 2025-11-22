import numpy as np

# link lengths (mm)
L1, L2, L3 = 50, 100, 100  
RESOLUTION = 4096  # servo resolution
FULL_ROTATION = 360.0

def normalize_angle(angle_deg):
    """Normalize negative degrees to [0, 360)"""
    return float(angle_deg % FULL_ROTATION)  # force Python float

def deg_to_ticks(angle_deg):
    """Convert degrees to servo ticks after normalization"""
    angle_norm = normalize_angle(angle_deg)
    ticks = int((angle_norm / FULL_ROTATION) * RESOLUTION)
    return angle_norm, ticks

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

    # Convert to degrees
    angles_deg = [float(a) for a in np.degrees([theta1, theta2, theta3, theta4])]

    # Normalize + convert to ticks
    angles_norm, angles_ticks = [], []
    for a in angles_deg:
        norm, ticks = deg_to_ticks(a)
        angles_norm.append(norm)
        angles_ticks.append(ticks)

    return angles_deg, angles_norm, angles_ticks

# Example: target at (150, 30, 50)
angles_deg, angles_norm, angles_ticks = inverse_kinematics(150, 30, 50, pitch=0)

print("Raw angles (deg):", angles_deg)
print("Normalized (0-360):", angles_norm)
print("Ticks (0-4095):", angles_ticks)
