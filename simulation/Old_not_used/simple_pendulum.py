import numpy as np
import matplotlib.pyplot as plt

# -------------------- Simulate Trajectory ------------------------------

time = np.linspace(0, 5 * 60, 1000)  # Time vector
dt = 0.05  # Time step

true_pitch = 10 * np.cos(0.1 * time)  # Sinusoidal pitch trajectory
true_roll = 5 * np.sin(0.1 * time)    # Sinusoidal roll trajectory
true_yaw = 2 * time + 3 * np.sin(0.05 * time)  # Linear yaw with sinusoidal variation

# -------------------- Kalman Filter Initialization --------------------

X = np.zeros((3, 1))    # State vector: [pitch, roll, yaw]

# A = np.eye(3)*1.004   # Transition matrix # Angles evolve without bias dynamics
A = np.eye(3)*1.00 # Transition matrix # Angles evolve without bias dynamics
B = np.eye(3) * dt      # # Control matrix, Control input directly affects angles
P = np.zeros((3, 3))    # Process uncertainty and noise covariance
Q = B @ B.T * 0.5**2    # Process noise covariance
H = np.eye(3)           # Observation matrix # Measurements directly observe angles
I = np.eye(3)           # Identity matrix 
R = np.eye(3)*0.5**2    # Sensor noise covariance

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
    kalmanPitch = X[0, 0]
    kalmanRoll = X[1, 0]
    kalmanYaw = X[2, 0]

    return kalmanPitch, kalmanRoll, kalmanYaw


# Compute angular velocity
true_roll_rate = np.gradient(true_roll, dt)
true_pitch_rate = np.gradient(true_pitch, dt)
true_yaw_rate = np.gradient(true_yaw, dt)

# -------------------- Simulate Noisy Measurements --------------------
np.random.seed(42)
noisy_roll = true_roll + np.random.normal(0, 2, size=true_roll.shape)
noisy_pitch = true_pitch + np.random.normal(0, 2, size=true_pitch.shape)
noisy_yaw = true_yaw + np.random.normal(0, 2, size=true_yaw.shape)

# Simulate noisy angular velocity measurements
noisy_roll_rate = true_roll_rate + np.random.normal(0, 1, size=true_roll_rate.shape)
noisy_pitch_rate = true_pitch_rate + np.random.normal(0, 1, size=true_pitch_rate.shape)
noisy_yaw_rate = true_yaw_rate + np.random.normal(0, 1, size=true_yaw_rate.shape)

# -------------------- Run simulation --------------------
kalman_roll = []
kalman_pitch = []
kalman_yaw = []

for i in range(len(time)):
    
    u = np.array([[noisy_roll_rate[i]], [noisy_pitch_rate[i]], [noisy_yaw_rate[i]]]) # Control input (noisy angular velocity in body frame)
    Z = np.array([[noisy_roll[i]], [noisy_pitch[i]], [noisy_yaw[i]]]) # Measurements (noisy angles)

    # Apply Kalman filter
    roll, pitch, yaw = kalman_2d(u, Z)

    # Store results
    kalman_roll.append(roll)
    kalman_pitch.append(pitch)
    kalman_yaw.append(yaw)

# Convert results to numpy arrays
kalman_roll = np.array(kalman_roll)
kalman_pitch = np.array(kalman_pitch)
kalman_yaw = np.array(kalman_yaw)

# -------------------- Plot Results --------------------
plt.figure(figsize=(12, 8))

# Roll
plt.subplot(3, 1, 1)
plt.plot(time, true_roll, label='True Roll (°)', color='blue')
plt.plot(time, noisy_roll, label='Noisy Roll (°)', color='cyan', alpha=0.5)
plt.plot(time, kalman_roll, label='Kalman Roll (°)', color='red')
plt.title("Roll")
plt.xlabel("Time (s)")
plt.ylabel("Angle (°)")
plt.legend()
plt.grid()

# Pitch
plt.subplot(3, 1, 2)
plt.plot(time, true_pitch, label='True Pitch (°)', color='blue')
plt.plot(time, noisy_pitch, label='Noisy Pitch (°)', color='cyan', alpha=0.5)
plt.plot(time, kalman_pitch, label='Kalman Pitch (°)', color='red')
plt.title("Pitch")
plt.xlabel("Time (s)")
plt.ylabel("Angle (°)")
plt.legend()
plt.grid()

# Yaw
plt.subplot(3, 1, 3)
plt.plot(time, true_yaw, label='True Yaw (°)', color='blue')
plt.plot(time, noisy_yaw, label='Noisy Yaw (°)', color='cyan', alpha=0.5)
plt.plot(time, kalman_yaw, label='Kalman Yaw (°)', color='red')
plt.title("Yaw")
plt.xlabel("Time (s)")
plt.ylabel("Angle (°)")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()

# -------------------- Rotation Matrix Function --------------------
# def rotation_matrix(roll, pitch, yaw):
#     cr, sr = np.cos(np.radians(roll)), np.sin(np.radians(roll))
#     cp, sp = np.cos(np.radians(pitch)), np.sin(np.radians(pitch))
#     cy, sy = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))

#     Rz = np.array([[cy, -sy, 0],
#                    [sy,  cy, 0],
#                    [ 0,   0, 1]])
#     Ry = np.array([[cp,  0, sp],
#                    [ 0,  1,  0],
#                    [-sp, 0, cp]])
#     Rx = np.array([[1,  0,   0],
#                    [0, cr, -sr],
#                    [0, sr,  cr]])

#     return Rx @ Ry @ Rz