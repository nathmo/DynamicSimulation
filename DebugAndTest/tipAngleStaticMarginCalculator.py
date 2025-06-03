import math

# Given parameters
a = 0.7         # meters
delta_L = 0.1   # meters
L = 1.0         # meters
M_t = 100       # kg
m_f = 10        # kg

# Convert input gravity angle theta_deg to radians (for current gravity)
theta_deg = -10  # degrees
theta = math.radians(theta_deg)

# Compute tilt angle phi due to leg shortening
phi = math.atan(delta_L / a)

# Compute z_CoM height of center of mass above table plane
z_CoM = ((M_t + 2 * m_f) * L - m_f * delta_L) / (M_t + 4 * m_f)

# Current gravity tipping check
left_side = math.tan(phi + theta)
right_side = a / (2 * z_CoM)

print(f"phi (deg): {math.degrees(phi):.3f}")
print(f"z_CoM (m): {z_CoM:.3f}")
print(f"tan(phi + theta): {left_side:.3f}")
print(f"a / (2 z_CoM): {right_side:.3f}")

if left_side > right_side:
    print("The table WILL TIP over at current gravity angle.")
else:
    print("The table will NOT tip over at current gravity angle.")

# Compute critical gravity angle theta_critical_left (tipping toward short leg side)
theta_critical_left = math.atan(a / (2 * z_CoM)) - phi

# Compute critical gravity angle theta_critical_right (tipping toward long leg side)
theta_critical_right = math.atan(-a / (2 * z_CoM)) + phi

print()
print(f"Critical gravity angle to tip LEFT  (deg): {math.degrees(theta_critical_left):.3f}")
print(f"Critical gravity angle to tip RIGHT (deg): {math.degrees(theta_critical_right):.3f}")

# Compute stability margins relative to current gravity angle
margin_left_deg = math.degrees(theta_critical_left - theta)
margin_right_deg = math.degrees(theta - theta_critical_right)

print()
print(f"Stability margin before tipping LEFT  (deg): {margin_left_deg:.3f}")
print(f"Stability margin before tipping RIGHT (deg): {margin_right_deg:.3f}")