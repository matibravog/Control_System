import numpy as np
import matplotlib.pyplot as plt

# =========================
# Parámetros configurables
# =========================

# Simulación
t_span = (0, 30)
n_steps = 2000
t_eval = np.linspace(*t_span, n_steps)
dt = t_eval[1] - t_eval[0]

g = np.array([0, 0, 9.806])
h = np.array([19172, 1562, 15144])

# Ruido sensores (stddev)
acc_noise = 0.0
gyro_noise = 0.0
mag_noise = 0.0

# PID límites de ángulo servo (rad)
servo_angle_limit_deg = 20
servo_angle_limits = (-np.deg2rad(servo_angle_limit_deg), np.deg2rad(servo_angle_limit_deg))

# Ganancias para Roll y Pitch/Yaw según setpoint (en grados)
roll_setpoints = np.array([0, 45, 0])
roll_gains = {
    'Kp': np.array([7.71, 1.0, 7.71]),
    'Ki': np.array([0.0, 0.10, 0.0]),
    'Kd': np.array([0.26, 0.05, 0.26])
}

pitchyaw_setpoints = np.array([0, 45, 0])
pitchyaw_gains = {
    'Kp': np.array([8.99, 9.26, 8.99]),
    'Ki': np.array([0.0, 0.0, 0.0]),
    'Kd': np.array([0.8, 0.81, 0.8])
}

# Parámetros del motor TVC
motor_force_kgf = 1.4
motor_force = motor_force_kgf * 9.81  # Newtons
lever_arm = 0.2  # m

# Perturbación externa
ext_torque_amp = 1
ext_torque_freq = 0.0001  # Hz

initial_pqr = np.deg2rad([0, 0, 0])  # rad/s
initial_angles = np.deg2rad([0, 0, 0])  # rad
state = np.concatenate((initial_pqr, initial_angles)).astype(float)

# Funciones de setpoint
def incremental_setpoint(t):
    if t < 4:
        return 0
    elif t < 8:
        return 45
    elif t < 12:
        return 0
    else:
        return 0

def setpoint_phi(t): return np.deg2rad(incremental_setpoint(t))
def setpoint_theta(t): return np.deg2rad(incremental_setpoint(t))
def setpoint_psi(t): return np.deg2rad(incremental_setpoint(t))

# =========================
# Funciones y clases
# =========================

def rocket_rotational_dynamics(y, torque):
    p, q, r, phi, theta, psi = y
    Tx, Ty, Tz = torque

    masa = 1.0
    radio = 0.2
    largo = 0.5
    damping = 0.025

    Izz = Iyy = (1/12) * masa * largo**2
    Ixx = (1/12) * masa * radio**2

    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    t_theta = np.tan(theta)
    s_phi = np.sin(phi)
    c_phi = np.cos(phi)
    if np.abs(c_theta) < 1e-6:
        c_theta = 1e-6

    p_dot = (Tx - (Izz - Iyy) * q * r - damping * p) / Ixx
    q_dot = (Ty - (Ixx - Izz) * p * r - damping * q) / Iyy
    r_dot = (Tz - (Iyy - Ixx) * p * q - damping * r) / Izz

    phi_dot = p + (s_phi * q + c_phi * r) * t_theta
    theta_dot = c_phi * q - s_phi * r
    psi_dot = (s_phi / c_theta) * q + (c_phi / c_theta) * r

    return np.array([p_dot, q_dot, r_dot, phi_dot, theta_dot, psi_dot])

class PID:
    def __init__(self, Kp, Ki, Kd, dt, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0
        self.prev_error = 0
        self.min_output, self.max_output = output_limits

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        if self.max_output is not None and output > self.max_output:
            output = self.max_output
        if self.min_output is not None and output < self.min_output:
            output = self.min_output

        self.prev_error = error
        return output

def rot_matrix(phi, theta, psi):
    R_phi   = np.array([[1, 0, 0], [0, np.cos(phi), np.sin(phi)], [0, -np.sin(phi), np.cos(phi)]])
    R_theta = np.array([[np.cos(theta), 0, -np.sin(theta)], [0, 1, 0], [np.sin(theta), 0, np.cos(theta)]])
    R_psi   = np.array([[np.cos(psi), np.sin(psi), 0], [-np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
    return R_phi @ R_theta @ R_psi

def euler_from_acceleration_vec(a, h):
    ax, ay, az = a
    hx, hy, hz = h
    phi = np.arctan2(ay, az)
    theta = -np.arctan2(ax, np.sqrt(ay**2 + az**2))
    psi = np.arctan2(-hy*np.cos(phi) + hz*np.sin(phi),
                      hx*np.cos(theta) + hy*np.sin(phi)*np.sin(theta) + hz*np.cos(phi)*np.sin(theta))
    return np.array([phi, theta, psi])

def kalman_2d(u, Z, X, A, B, P, Q, H, I, R):
    X = A @ X + B @ u
    P = A @ P @ A.T + Q
    Y = Z - H @ X
    L = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(L)
    X = X + K @ Y
    P = (I - K @ H) @ P
    return X, P

# Función para interpolar ganancias segun el setpoint actual
def interpolate_gains(angle_deg, sp_array, gains_array):
    if angle_deg <= sp_array[0]:
        return gains_array[0]
    elif angle_deg >= sp_array[-1]:
        return gains_array[-1]
    else:
        return np.interp(angle_deg, sp_array, gains_array)

# =========================
# Inicialización Kalman y PID
# =========================

pid_phi = PID(1.0, 0.1, 0.05, dt, output_limits=servo_angle_limits)
pid_theta = PID(1.0, 0.1, 0.05, dt, output_limits=servo_angle_limits)
pid_psi = PID(1.0, 0.1, 0.05, dt, output_limits=servo_angle_limits)  # ganancias serán actualizadas igual que pitch

X = np.zeros((3, 1))
A = np.eye(3)
P = np.eye(3) * 0.1
H = np.eye(3)
I = np.eye(3)
R = np.diag([acc_noise**2, gyro_noise**2, mag_noise**2])

log_true = []
log_kalman = []
log_meas = []
log_pid = []
log_external = []
log_control = []
log_targets = []

# =========================
# Loop principal
# =========================

for i in range(n_steps):
    t = t_eval[i]

    sp_deg = incremental_setpoint(t)

    phi_target = np.deg2rad(sp_deg)
    theta_target = np.deg2rad(sp_deg)
    psi_target = np.deg2rad(sp_deg)
    log_targets.append([phi_target, theta_target, psi_target])

    # Ganancias adaptativas
    pid_phi.Kp = interpolate_gains(sp_deg, roll_setpoints, roll_gains['Kp'])
    pid_phi.Ki = interpolate_gains(sp_deg, roll_setpoints, roll_gains['Ki'])
    pid_phi.Kd = interpolate_gains(sp_deg, roll_setpoints, roll_gains['Kd'])

    # Pitch y Yaw comparten ganancias
    pid_theta.Kp = interpolate_gains(sp_deg, pitchyaw_setpoints, pitchyaw_gains['Kp'])
    pid_theta.Ki = interpolate_gains(sp_deg, pitchyaw_setpoints, pitchyaw_gains['Ki'])
    pid_theta.Kd = interpolate_gains(sp_deg, pitchyaw_setpoints, pitchyaw_gains['Kd'])

    pid_psi.Kp = pid_theta.Kp
    pid_psi.Ki = pid_theta.Ki
    pid_psi.Kd = pid_theta.Kd

    # Perturbación externa
    ext_torque = np.array([
        ext_torque_amp * np.sin(2*np.pi*ext_torque_freq*t),
        ext_torque_amp * np.sin(2*np.pi*ext_torque_freq*t),
        ext_torque_amp * np.sin(2*np.pi*ext_torque_freq*t)
    ])

    # Rotación actual y ruido simulado sensores
    R_mat = rot_matrix(state[3], state[4], state[5])
    acc_body = R_mat @ g + np.random.normal(0, acc_noise, 3)
    mag_body = R_mat @ h + np.random.normal(0, mag_noise, 3)
    gyro_body = state[:3] + np.random.normal(0, gyro_noise, 3)

    # Medición Euler desde aceleración + magnetómetro
    meas_euler = euler_from_acceleration_vec(acc_body, mag_body)

    # Matriz B para Kalman
    B = np.array([
        [0, dt*np.cos(X[1,0]), -dt*np.sin(X[1,0])],
        [dt, dt*np.sin(X[1,0])*np.tan(X[0,0]), dt*np.cos(X[1,0])*np.tan(X[0,0])],
        [0, dt*np.sin(X[1,0])/np.cos(X[0,0]), dt*np.cos(X[1,0])/np.cos(X[0,0])]
    ])
    u = gyro_body.reshape(3, 1)
    Z = meas_euler.reshape(3, 1)
    Q = B @ B.T * 25**2

    # Actualización Kalman
    X, P = kalman_2d(u, Z, X, A, B, P, Q, H, I, R)

    # Errores
    e_phi = phi_target - X[0,0]
    e_theta = theta_target - X[1,0]
    e_psi = psi_target - X[2,0]

    # Control PID
    angle_servo_phi = pid_phi.update(e_phi)
    angle_servo_theta = pid_theta.update(e_theta)
    angle_servo_psi = pid_psi.update(e_psi)

    # Torques TVC
    torque_phi = motor_force * lever_arm * np.sin(angle_servo_phi)
    torque_theta = motor_force * lever_arm * np.sin(angle_servo_theta)
    torque_psi = motor_force * lever_arm * np.sin(angle_servo_psi)

    control_torque = np.array([torque_phi, torque_theta, torque_psi])
    total_torque = control_torque + ext_torque

    # Dinámica rotacional
    dot = rocket_rotational_dynamics(state, total_torque)
    state += dot * dt

    # Logs
    log_true.append(state.copy())
    log_kalman.append(X.flatten())
    log_meas.append(meas_euler.copy())
    log_pid.append([angle_servo_phi, angle_servo_theta, angle_servo_psi])
    log_external.append(ext_torque.copy())
    log_control.append(control_torque.copy())

# =========================
# Gráficos
# =========================

log_true = np.array(log_true)
log_kalman = np.array(log_kalman)
log_meas = np.array(log_meas)
log_pid = np.array(log_pid)
log_external = np.array(log_external)
log_control = np.array(log_control)
log_targets = np.array(log_targets)
todeg = 180/np.pi

plt.figure(figsize=(14, 8))
for i, label in enumerate(["Roll", "Pitch", "Yaw"]):
    plt.subplot(3,1,i+1)
    plt.plot(t_eval, log_true[:,3+i]*todeg, label='True', linewidth=1.5)
    plt.plot(t_eval, log_meas[:,i]*todeg, label='Measured', alpha=0.4)
    plt.plot(t_eval, log_kalman[:,i]*todeg, label='Kalman', color='red', linestyle='-')
    plt.plot(t_eval, log_targets[:,i]*todeg, '--k', label='Target')
    plt.ylabel("Angle (°)")
    plt.title(f"{label} Tracking")
    plt.grid()
    if i == 0: plt.legend()

plt.xlabel("Time (s)")
plt.tight_layout()
plt.show()

plt.figure(figsize=(14,6))
for i, label in enumerate(["Servo angle Roll", "Servo angle Pitch", "Servo angle Yaw"]):
    plt.subplot(3,1,i+1)
    plt.plot(t_eval, np.rad2deg(log_pid[:,i]), label='Servo angle (deg)')
    plt.ylabel("Angle (°)")
    plt.title(label)
    plt.grid()
    if i == 0: plt.legend()
plt.xlabel("Time (s)")
plt.tight_layout()
plt.show()

plt.figure(figsize=(14, 8))
torque_labels = ['Roll Torque', 'Pitch Torque', 'Yaw Torque']
for i in range(3):
    plt.subplot(3,1,i+1)
    plt.plot(t_eval, log_control[:,i], label='Control Torque', color='green')
    plt.plot(t_eval, log_external[:,i], label='External Torque', color='orange', linestyle='--')
    plt.ylabel("Torque (Nm)")
    plt.title(torque_labels[i])
    plt.grid()
    if i == 0: plt.legend()
plt.xlabel("Time (s)")
plt.tight_layout()
plt.show()
