import numpy as np
import matplotlib.pyplot as plt

# =========================
# Parámetros configurables
# =========================

t_span = (0, 30)
n_steps = 2000
t_eval = np.linspace(*t_span, n_steps)
dt = t_eval[1] - t_eval[0]

g = np.array([0, 0, 9.806])
h = np.array([19172, 1562, 15144])

acc_noise = 0.1
gyro_noise = 0.1
mag_noise = 0.1

servo_angle_limit_deg = 20
servo_angle_limits = (-np.deg2rad(servo_angle_limit_deg), np.deg2rad(servo_angle_limit_deg))

motor_force_kgf = 1.4
motor_force = motor_force_kgf * 9.81
lever_arm = 0.2

ext_torque_amp = 0
ext_torque_freq = 0.0

initial_pqr = np.deg2rad([0, 0, 0])
initial_angles = np.deg2rad([0, 0,0])
state = np.concatenate((initial_pqr, initial_angles)).astype(float)

def incremental_setpoint(t):
    if t < 4:
        return np.deg2rad(0)
    elif t < 8:
        return np.deg2rad(45)
    elif t < 12:
        return np.deg2rad(0)
    elif t < 16:
        return np.deg2rad(0)
    else:
        return np.deg2rad(0)
    

def setpoint_phi(t): return incremental_setpoint(t)
def setpoint_theta(t): return incremental_setpoint(t)
def setpoint_psi(t): return incremental_setpoint(t)

# =========================
# Adaptive PID Controller
# =========================

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

        output = np.clip(output, self.min_output, self.max_output)
        self.prev_error = error
        return output

class AdaptivePID(PID):
    def __init__(self, axis, dt, output_limits=(None, None)):
        super().__init__(0, 0, 0, dt, output_limits)
        self.axis = axis

    def update_with_setpoint(self, error, setpoint_rad):
        setpoint_deg = np.rad2deg(setpoint_rad)
        gains = get_gains_from_setpoint(setpoint_deg, self.axis)
        self.Kp, self.Ki, self.Kd = gains["Kp"], gains["Ki"], gains["Kd"]
        return self.update(error)

def get_gains_from_setpoint(setpoint_deg, axis):
    # Limita el setpoint a un máximo de 45°
    setpoint_deg = min(setpoint_deg, 45)

    if axis == "phi":
        gains_map = {
            0: {"Kp": 0.0, "Ki": 0.0, "Kd": 0.0},      # opcional: puedes ajustar esto
            # 15: {"Kp": 1.00, "Ki": 0.10, "Kd": 0.05},
            # 30: {"Kp": 4.0,  "Ki": 0.05, "Kd": 0.15},  # opcional: interpolación manual
            45: {"Kp": 7.76, "Ki": 0.00, "Kd": 0.26},
            0: {"Kp": 7.71, "Ki": 0.0, "Kd": 0.26},
            0: {"Kp": 0.0, "Ki": 0.0, "Kd": 0.0},
            0: {"Kp": 0.0, "Ki": 0.0, "Kd": 0.0},
        }
    elif axis == "theta":
        gains_map = {
            0: {"Kp": 0.0, "Ki": 0.0, "Kd": 0.0},
            # 15: {"Kp": 1.00, "Ki": 0.10, "Kd": 0.05},
            # 30: {"Kp": 5.0,  "Ki": 0.02, "Kd": 0.3},
            45: {"Kp": 8.99, "Ki": 0.03, "Kd": 0.80},
            0: {"Kp": 0.0, "Ki": 0.0, "Kd": 0.0},
            0: {"Kp": 0.0, "Ki": 0.0, "Kd": 0.0},
            0: {"Kp": 0.0, "Ki": 0.0, "Kd": 0.0},
        }
    else:
        return {"Kp": 0.5, "Ki": 0.0, "Kd": 0.1}

    closest = min(gains_map.keys(), key=lambda k: abs(setpoint_deg - k))
    return gains_map[closest]


# =========================
# Dinámica y Utilidades
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
    return X

# =========================
# Inicialización PID y Kalman
# =========================

pid_phi = AdaptivePID("phi", dt=dt, output_limits=servo_angle_limits)
pid_theta = AdaptivePID("theta", dt=dt, output_limits=servo_angle_limits)
pid_psi = PID(Kp=0.5, Ki=0.0, Kd=0.1, dt=dt, output_limits=servo_angle_limits)

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
# Simulación
# =========================

for i in range(n_steps):
    t = t_eval[i]

    phi_target = setpoint_phi(t)
    theta_target = setpoint_theta(t)
    psi_target = setpoint_psi(t)
    log_targets.append([phi_target, theta_target, psi_target])

    ext_torque = np.array([
        ext_torque_amp * np.sin(2*np.pi*ext_torque_freq*t),
        ext_torque_amp * np.sin(2*np.pi*ext_torque_freq*t),
        ext_torque_amp * np.sin(2*np.pi*ext_torque_freq*t)
    ])

    R_mat = rot_matrix(state[3], state[4], state[5])
    acc_body = R_mat @ g + np.random.normal(0, acc_noise, 3)
    mag_body = R_mat @ h + np.random.normal(0, mag_noise, 3)
    gyro_body = state[:3] + np.random.normal(0, gyro_noise, 3)

    meas_euler = euler_from_acceleration_vec(acc_body, mag_body)

    B = np.array([
        [0, dt*np.cos(X[1,0]), -dt*np.sin(X[1,0])],
        [dt, dt*np.sin(X[1,0])*np.tan(X[0,0]), dt*np.cos(X[1,0])*np.tan(X[0,0])],
        [0, dt*np.sin(X[1,0])/np.cos(X[0,0]), dt*np.cos(X[1,0])/np.cos(X[0,0])]
    ])
    u = gyro_body.reshape(3, 1)
    Z = meas_euler.reshape(3, 1)
    Q = B @ B.T * 25**2
    X = kalman_2d(u, Z, X, A, B, P, Q, H, I, R)

    e_phi = phi_target - X[0,0]
    e_theta = theta_target - X[1,0]
    e_psi = psi_target - X[2,0]

    angle_servo_phi = pid_phi.update_with_setpoint(e_phi, phi_target)
    angle_servo_theta = pid_theta.update_with_setpoint(e_theta, theta_target)
    angle_servo_psi = pid_psi.update(e_psi)

    torque_phi = motor_force * lever_arm * np.sin(angle_servo_phi)
    torque_theta = motor_force * lever_arm * np.sin(angle_servo_theta)
    torque_psi = motor_force * lever_arm * np.sin(angle_servo_psi)

    control_torque = np.array([torque_phi, torque_theta, torque_psi])
    total_torque = control_torque + ext_torque

    dot = rocket_rotational_dynamics(state, total_torque)
    state += dot * dt

    log_true.append(state.copy())
    log_kalman.append(X.flatten())
    log_meas.append(meas_euler.copy())
    log_pid.append([angle_servo_phi, angle_servo_theta, angle_servo_psi])
    log_external.append(ext_torque.copy())
    log_control.append(control_torque.copy())

# =========================
# Resultados
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
