import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Inputs for dynamic equation
def torques(t, state):
    f = 2;
    omega = 2*np.pi*f
    torque_x = 0;
    torque_y = 10*np.sin(omega*t)
    torque_z = 10*np.sin(omega*t + 0*np.pi/2)
    return np.array([torque_x, torque_y, torque_z])

# Euler's equations for a rigid body (rocket)
def rocket_rotational_dynamics(t, y):
    # Physical parameters
    m = 100
    l = 2
    r = 0.1
    Ixx = (1/2)*m*r**2   # Moment of inertia around X axis (kg*m^2)
    Iyy = (1/12)*m*l**2  # Moment of inertia around Y axis (kg*m^2)
    Izz = (1/12)*m*l**2  # Moment of inertia around Z axis (kg*m^2)
    
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

# Rotational matrix from Euler angles
def rot_matrix(phi,theta,psi):
    R_phi   = np.array([[1,0,0],[0,np.cos(phi),np.sin(phi)],[0,-np.sin(phi),np.cos(phi)]])
    R_theta = np.array([[np.cos(theta),0,-np.sin(theta)],[0,1,0],[np.sin(theta),0,np.cos(theta)]])
    R_psi   = np.array([[np.cos(psi),np.sin(psi),0],[-np.sin(psi),np.cos(phi),0],[0,0,1]])
    R_total = R_phi @ (R_theta @ R_psi)
    return R_total


# Roll and pitch from gravity measured in body
def lifting_atan2(y,x):
    last = 0
    out = []
    for i in range(len(x)):
        angle = np.arctan2(y[i], x[i])
        while angle < last-np.pi: angle += 2*np.pi
        while angle > last+np.pi: angle -= 2*np.pi
        last = angle
        out.append(angle)
    return out

def euler_from_acceleration_vec (a):
    ax = a[1,:]
    ay = a[0,:]
    az = a[2,:]
    phi_an = -np.array(lifting_atan2(-ax,az))
    theta_an = -np.array(lifting_atan2(ay, np.sqrt(ax * ax + az * az)))
    return phi_an,theta_an

def euler_from_acceleration (a):
    ax = a[1]
    ay = a[0]
    az = a[2]
    phi_an = -np.arctan2(-ax, az)
    theta_an = -np.arctan2(ay, np.sqrt(ax * ax + az * az))
    return phi_an,theta_an

# Simple Kalman that uses euler angles from acceleration as measurements, and forward euler as model
def kalman_2d(u,Z,X,A,B,P,Q,H,I,R):
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
    return X

# Time span for simulation
t_span = (0, 20)  # Start and end time (seconds)
t_eval = np.linspace(*t_span, 1000)  # Time vector for solution output

# Initial conditions: [p, q, r, phi, theta, psi]
p0 = 0      # Initial roll rate (rad/s)
q0 = 0      # Initial pitch rate (rad/s)
r0 = 0      # Initial yaw rate (rad/s)
phi0 = 0  # Initial roll angle (rad)
theta0 = 0 # Initial pitch angle (rad)
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
dt = t_eval[1] - t_eval[0]  # Time step from simulation


# Computation of euler angles from gravity measurement in body
phi_measured = np.zeros(shape=sol.t.shape)
theta_measured = np.zeros(shape=sol.t.shape)
g = np.array([0,0,9.8]) # gravity vector in inertial
gb = np.zeros(shape=(3, len(sol.t))) # gravity vector in body
gbr = np.zeros(shape=(3, len(sol.t))) # gravity vector in body with noise
a_stdv = 0.1 # standard deviation of the acceleration sensor

for i in range(len(sol.t)):
    R = rot_matrix(phi[i],theta[i],psi[i])
    gb[:,i] = R @ g # gravity in body by rotation of gravity

gbr = gb + np.random.normal(0, a_stdv, size=gb.shape) # add noise

psi_stdv = a_stdv/2 # standard deviation of the acceleration sensor
phi_measured,theta_measured = euler_from_acceleration_vec(gbr)
psi_measured = psi + np.random.normal(0, psi_stdv, size=psi.shape) # add noise

omega_stdv = 0.1;
noisy_p = p + np.random.normal(0, omega_stdv, size=p.shape)
noisy_q = q + np.random.normal(0, omega_stdv, size=q.shape)
noisy_r = r + np.random.normal(0, omega_stdv, size=r.shape)

# Linear Kalman using "measured" euler angles and omega as inputs
X = np.zeros((3, 1))    # State vector: [roll, pitch, yaw]
A = np.eye(3) # Transition matrix # Angles evolve without bias dynamics
B = np.eye(3) * dt      # # Control matrix, Control input directly affects angles
P = np.zeros((3, 3))    # Process uncertainty and noise covariance
Q = B @ B.T * 2**2    # Process noise covariance
H = np.eye(3)           # Observation matrix # Measurements directly observe angles
I = np.eye(3)           # Identity matrix 
R = np.eye(3)*omega_stdv**2    # Sensor noise covariance


kalman_roll = [X[0, 0]]
kalman_pitch = [X[1, 0]]
kalman_yaw = [X[2, 0]]

for i in range(len(sol.t)-1):
    
    u = np.array([[noisy_p[i]], [noisy_q[i]], [noisy_r[i]]]) # Control input (noisy angular velocity in body frame)
    Z = np.array([[phi_measured[i+1]],[theta_measured[i+1]],[psi_measured[i+1]]]) # Measurements (noisy angles)

    # Apply Kalman filter
    X = kalman_2d(u,Z,X,A,B,P,Q,H,I,R)
    # Store results
    kalman_roll.append(X[0, 0])
    kalman_pitch.append(X[1, 0])
    kalman_yaw.append(X[2, 0])




# IVP plots
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
plt.plot(sol.t, np.degrees(phi), label='Roll φ (rad)')
plt.plot(sol.t, np.degrees(theta), label='Pitch θ (rad)')
plt.plot(sol.t, np.degrees(psi), label='Yaw ψ (rad)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Rocket Attitude (Euler Angles, Inertial Frame)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# "Measured" euler angle plots
plt.figure(figsize=(10, 4))
plt.plot(sol.t, phi, label='Roll φ (rad)')
plt.plot(sol.t, phi_measured, label='Roll measured φ (rad)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Rocket Attitude (Euler Angles, Inertial Frame)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 4))
plt.plot(sol.t, theta, label='Pitch θ (rad)')
plt.plot(sol.t, theta_measured, label='Pitch measured θ (rad)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Rocket Attitude (Euler Angles, Inertial Frame)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 4))
plt.plot(sol.t, psi, label='Yaw ψ (rad)')
plt.plot(sol.t, psi_measured, label='Yaw measured ψ (rad)')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Rocket Attitude (Euler Angles, Inertial Frame)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()


# Kalman estimation plots
plt.figure(figsize=(12, 8))
# Roll
plt.subplot(3, 1, 1)
plt.plot(sol.t, phi, label='True Roll (rad)', color='blue')
plt.plot(sol.t, phi_measured, label='Noisy Roll (rad)', color='green', alpha=0.5)
plt.plot(sol.t, kalman_roll, label='Kalman Roll (rad)', color='red')
plt.title("Roll")
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.legend()
plt.grid()

# Pitch
plt.subplot(3, 1, 2)
plt.plot(sol.t, theta, label='True Pitch (rad)', color='blue')
plt.plot(sol.t, theta_measured, label='Noisy Pitch (rad)', color='green', alpha=0.5)
plt.plot(sol.t, kalman_pitch, label='Kalman Pitch (rad)', color='red')
plt.title("Pitch")
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.legend()
plt.grid()

# Yaw
plt.subplot(3, 1, 3)
plt.plot(sol.t, psi, label='True Yaw (rad)', color='blue')
plt.plot(sol.t, psi_measured, label='Noisy Yaw (rad)', color='green', alpha=0.5)
plt.plot(sol.t, kalman_yaw, label='Kalman Yaw (rad)', color='red')
plt.title("Yaw")
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()