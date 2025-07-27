import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# Parámetros físicos (dos inercias distintas)
Ixx = (1/12) * 1.0 * 0.2**2   # Inercia eje X (Roll)
Iyy = (1/12) * 1.0 * 0.5**2   # Inercia eje Y (Pitch)
d = 0.025                     # Amortiguamiento igual para ambos

# Tiempo de simulación
t_total = 20
dt = 0.01
t = np.arange(0, t_total, dt)

# Setpoints escalonados cada 2 segundos para ambos ángulos (en grados)
setpoints_roll = [0, 45, 0]
setpoints_pitch = [0, 45, 0]
setpoints_roll_rad = np.deg2rad(setpoints_roll)
setpoints_pitch_rad = np.deg2rad(setpoints_pitch)
setpoint_interval = int(2 / dt)

# Inicializaciones
y_roll = np.zeros_like(t)
y_pitch = np.zeros_like(t)
u_roll = np.zeros_like(t)
u_pitch = np.zeros_like(t)
e_roll = np.zeros_like(t)
e_pitch = np.zeros_like(t)
r_roll = np.zeros_like(t)
r_pitch = np.zeros_like(t)

# Condiciones iniciales (ángulos en grados)
initial_roll_deg =30
initial_pitch_deg = 30
y_roll[0] = np.deg2rad(initial_roll_deg)
y_pitch[0] = np.deg2rad(initial_pitch_deg)

# Condiciones iniciales de velocidad (rad/s)
initial_roll_rate = 0.0
initial_pitch_rate = 0.0

# Inicial ganancias PID (puedes cambiar)
Kp_r, Ki_r, Kd_r = 1.0, 0.1, 0.05
Kp_p, Ki_p, Kd_p = 1.0, 0.1, 0.05

integral_r = 0
prev_error_r = 0
integral_p = 0
prev_error_p = 0

torque_limit = 2.0  # Nm límite para ambos

# Función objetivo para cada canal (Roll o Pitch)
def objective_pid(x, y0, r_seq, I, d):
    kp, ki, kd = x
    if np.any(np.array(x) < 0):
        return 1e6

    y_sim = np.zeros_like(r_seq)
    u_sim = np.zeros_like(r_seq)
    e_sim = np.zeros_like(r_seq)
    y_sim[0] = y0
    integ = 0
    prev_e = 0

    for i in range(1, len(r_seq)):
        e_sim[i] = r_seq[i] - y_sim[i-1]
        integ += e_sim[i] * dt
        der = (e_sim[i] - prev_e) / dt
        u_sim[i] = kp * e_sim[i] + ki * integ + kd * der

        # Saturar torque
        if u_sim[i] > torque_limit:
            u_sim[i] = torque_limit
        elif u_sim[i] < -torque_limit:
            u_sim[i] = -torque_limit

        prev_e = e_sim[i]

        if i == 1:
            y_dot = 0
        else:
            y_dot = (y_sim[i-1] - y_sim[i-2]) / dt

        y_ddot = (u_sim[i] - d * y_dot) / I
        y_sim[i] = y_sim[i-1] + y_dot * dt + 0.5 * y_ddot * dt**2

    ise = np.trapz(e_sim**2, dx=dt)
    iue = np.trapz(u_sim**2, dx=dt)
    return ise + 0.01 * iue

# Simulación con ajuste adaptativo para Roll y Pitch
for i in range(1, len(t)):
    idx_sp = i // setpoint_interval
    if idx_sp >= len(setpoints_roll_rad):
        idx_sp = len(setpoints_roll_rad) - 1

    r_roll[i] = setpoints_roll_rad[idx_sp]
    r_pitch[i] = setpoints_pitch_rad[idx_sp]

    # Optimización al inicio de nuevo setpoint para Roll
    if i % setpoint_interval == 0:
        r_local_roll = r_roll[i:i+int(2/dt)]
        r_local_roll = np.pad(r_local_roll, (0, max(0, 200 - len(r_local_roll))), 'edge')
        result_r = minimize(objective_pid, [Kp_r, Ki_r, Kd_r], args=(y_roll[i-1], r_local_roll, Ixx, d),
                            bounds=[(0, 50), (0, 50), (0, 10)], method='L-BFGS-B')
        Kp_r, Ki_r, Kd_r = result_r.x
        print(f"Roll setpoint: {np.rad2deg(setpoints_roll_rad[idx_sp]):.1f}°, Kp={Kp_r:.2f}, Ki={Ki_r:.2f}, Kd={Kd_r:.2f}")

        r_local_pitch = r_pitch[i:i+int(2/dt)]
        r_local_pitch = np.pad(r_local_pitch, (0, max(0, 200 - len(r_local_pitch))), 'edge')
        result_p = minimize(objective_pid, [Kp_p, Ki_p, Kd_p], args=(y_pitch[i-1], r_local_pitch, Iyy, d),
                            bounds=[(0, 50), (0, 50), (0, 10)], method='L-BFGS-B')
        Kp_p, Ki_p, Kd_p = result_p.x
        print(f"Pitch setpoint: {np.rad2deg(setpoints_pitch_rad[idx_sp]):.1f}°, Kp={Kp_p:.2f}, Ki={Ki_p:.2f}, Kd={Kd_p:.2f}")

    # Control Roll
    e_roll[i] = r_roll[i] - y_roll[i-1]
    integral_r += e_roll[i] * dt
    derivative_r = (e_roll[i] - prev_error_r) / dt
    u_roll[i] = Kp_r * e_roll[i] + Ki_r * integral_r + Kd_r * derivative_r
    if u_roll[i] > torque_limit:
        u_roll[i] = torque_limit
    elif u_roll[i] < -torque_limit:
        u_roll[i] = -torque_limit
    prev_error_r = e_roll[i]

    if i == 1:
        y_dot_r = initial_roll_rate
    else:
        y_dot_r = (y_roll[i-1] - y_roll[i-2]) / dt
    y_ddot_r = (u_roll[i] - d * y_dot_r) / Ixx
    y_roll[i] = y_roll[i-1] + y_dot_r * dt + 0.5 * y_ddot_r * dt**2

    # Control Pitch
    e_pitch[i] = r_pitch[i] - y_pitch[i-1]
    integral_p += e_pitch[i] * dt
    derivative_p = (e_pitch[i] - prev_error_p) / dt
    u_pitch[i] = Kp_p * e_pitch[i] + Ki_p * integral_p + Kd_p * derivative_p
    if u_pitch[i] > torque_limit:
        u_pitch[i] = torque_limit
    elif u_pitch[i] < -torque_limit:
        u_pitch[i] = -torque_limit
    prev_error_p = e_pitch[i]

    if i == 1:
        y_dot_p = initial_pitch_rate
    else:
        y_dot_p = (y_pitch[i-1] - y_pitch[i-2]) / dt
    y_ddot_p = (u_pitch[i] - d * y_dot_p) / Iyy
    y_pitch[i] = y_pitch[i-1] + y_dot_p * dt + 0.5 * y_ddot_p * dt**2

# Graficar resultados
plt.figure(figsize=(12,8))

plt.subplot(3,1,1)
plt.plot(t, np.rad2deg(r_roll), 'k--', label='Setpoint Roll')
plt.plot(t, np.rad2deg(y_roll), 'b', label='Salida Roll')
plt.ylabel('Roll (°)')
plt.legend()
plt.grid()
plt.title('Control adaptativo PID con torque limitado para Roll y Pitch')

plt.subplot(3,1,2)
plt.plot(t, np.rad2deg(r_pitch), 'k--', label='Setpoint Pitch')
plt.plot(t, np.rad2deg(y_pitch), 'g', label='Salida Pitch')
plt.ylabel('Pitch (°)')
plt.legend()
plt.grid()

plt.subplot(3,1,3)
plt.plot(t, u_roll, 'b-', label='Torque Roll (Nm)')
plt.plot(t, u_pitch, 'g-', label='Torque Pitch (Nm)')
plt.xlabel('Tiempo (s)')
plt.ylabel('Torque (Nm)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
