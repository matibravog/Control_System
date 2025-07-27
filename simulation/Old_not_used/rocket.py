import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Physical parameters
Ixx = 0.1  # Moment of inertia around X axis (kg*m^2)
Iyy = 0.2  # Moment of inertia around Y axis (kg*m^2)
Izz = 0.3  # Moment of inertia around Z axis (kg*m^2)

# Time span for simulation
t_span = (0, 20)  # Start and end time (seconds)
t_eval = np.linspace(*t_span, 1000)  # Time vector for solution output

def torques(t, state):
    return np.array([0.0, 0.0, 0.0])

# Euler's equations for a rigid body (rocket)
def rocket_rotational_dynamics(t, y):
    # p: roll rate (body X), q: pitch rate (body Y), r: yaw rate (body Z)
    # phi: roll angle, theta: pitch angle, psi: yaw angle (Euler angles)
    p, q, r, phi, theta, psi = y
    Tx, Ty, Tz = torques(t, y)

    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    t_theta = np.tan(theta)
    s_phi = np.sin(phi)
    c_phi = np.cos(phi)

    if np.abs(c_theta) < 1e-6:
        c_theta = 1e-6  # avoid division by zero

    # motion equations (body frame)
    p_dot = (Tx - (Izz - Iyy) * q * r) / Ixx
    q_dot = (Ty - (Ixx - Izz) * p * r) / Iyy
    r_dot = (Tz - (Iyy - Ixx) * p * q) / Izz

    # euler rate (inertial frame)
    phi_dot = p + (s_phi * q + c_phi * r) * t_theta
    theta_dot = c_phi * q - s_phi * r
    psi_dot = (s_phi / c_theta) * q + (c_phi / c_theta) * r

    return [p_dot, q_dot, r_dot, phi_dot, theta_dot, psi_dot]

# Initial conditions: [p, q, r, phi, theta, psi]
p0 = 0      # Initial roll rate (rad/s)
q0 = 0      # Initial pitch rate (rad/s)
r0 = 0      # Initial yaw rate (rad/s)
phi0 = 0  # Initial roll angle (rad)
theta0 = 2 # Initial pitch angle (rad)
psi0 = 0  # Initial yaw angle (rad)
y0 = [p0, q0, r0, phi0, theta0, psi0]

# Integrate the equations of motion
sol = solve_ivp(rocket_rotational_dynamics, t_span, y0, t_eval=t_eval)

p = sol.y[0]     # Roll rate (body X)
q = sol.y[1]     # Pitch rate (body Y)
r = sol.y[2]     # Yaw rate (body Z)
phi = sol.y[3]   # Roll angle (inertial)
theta = sol.y[4] # Pitch angle (inertial)
psi = sol.y[5]   # Yaw angle (inertial)

# -------------------- Kalman Filter Initialization --------------------
dt = t_eval[1] - t_eval[0]  # Time step from simulation

# notaaaaaaaaaaaaaaaa
# 1 vs 1.004 afecta el resultado

X = np.zeros((3, 1))    # State vector: [phi, theta, psi]
#A = np.eye(3)*1.004   # Transition matrix
A = np.eye(3)*1.000   # Transition matrix
B = np.eye(3) * dt      # Control matrix, Control input directly affects angles
P = np.zeros((3, 3))    # Process uncertainty and noise covariance
Q = B @ B.T * 0.05**2    # Process noise covariance
H = np.eye(3)           # Observation matrix
I = np.eye(3)           # Identity matrix 
R = np.eye(3)*0.05**2    # Sensor noise covariance

# -------------------- Kalman Filter Function --------------------
def kalman_2d(u, Z):
    global X, P, K
    # State and uncertainty prediction
    X = A @ X + B @ u
    P = A @ P @ A.T + Q
    # kalman gain
    Y = Z - H @ X
    L = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(L)
    # State and process covariance update
    X = X + K @ Y
    P = (I - K @ H) @ P
    # Output
    kalmanPhi = X[0, 0]
    kalmanTheta = X[1, 0]
    kalmanPsi = X[2, 0]
    return kalmanPhi, kalmanTheta, kalmanPsi

# -------------------- Apply Kalman Filter --------------------
kalman_phi = []
kalman_theta = []
kalman_psi = []

for i in range(len(t_eval)):
    # Control input: body angular velocities (p, q, r)
    u = np.array([[p[i]], [q[i]], [r[i]]])
    # Measurement: Euler angles (phi, theta, psi)
    Z = np.array([[phi[i]], [theta[i]], [psi[i]]])
    # Apply Kalman filter
    kphi, ktheta, kpsi = kalman_2d(u, Z)
    kalman_phi.append(kphi)
    kalman_theta.append(ktheta)
    kalman_psi.append(kpsi)

kalman_phi = np.array(kalman_phi)
kalman_theta = np.array(kalman_theta)
kalman_psi = np.array(kalman_psi)

# --- Plot angular velocities in body frame ---
plt.figure(figsize=(10, 4))
plt.plot(sol.t, p, label='p (body, rad/s)')
plt.plot(sol.t, q, label='q (body, rad/s)')
plt.plot(sol.t, r, label='r (body, rad/s)')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Rocket Angular Velocities (Body Frame)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# --- Plot Euler angles (inertial frame) ---
plt.figure(figsize=(10, 4))
plt.plot(sol.t, np.degrees(phi), label='Roll φ (deg)')
plt.plot(sol.t, np.degrees(theta), label='Pitch θ (deg)')
plt.plot(sol.t, np.degrees(psi), label='Yaw ψ (deg)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.title('Rocket Attitude (Euler Angles, Inertial Frame)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# --- Plot Kalman estimated angles vs true angles ---
plt.figure(figsize=(10, 4))
plt.plot(t_eval, np.degrees(phi), label='True Roll φ (deg)', color='blue')
plt.plot(t_eval, np.degrees(theta), label='True Pitch θ (deg)', color='green')
plt.plot(t_eval, np.degrees(psi), label='True Yaw ψ (deg)', color='red')
plt.plot(t_eval, np.degrees(kalman_phi), '--', label='Kalman Roll φ (deg)', color='blue', alpha=0.6)
plt.plot(t_eval, np.degrees(kalman_theta), '--', label='Kalman Pitch θ (deg)', color='green', alpha=0.6)
plt.plot(t_eval, np.degrees(kalman_psi), '--', label='Kalman Yaw ψ (deg)', color='red', alpha=0.6)
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.title('Rocket Attitude: True vs Kalman (Euler Angles)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()
