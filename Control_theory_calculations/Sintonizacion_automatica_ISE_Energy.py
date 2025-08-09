import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import TransferFunction, lsim
from scipy.optimize import minimize

# Parámetros físicos
I = (1/12) * 1.0 * 0.2**2
d = 0.025

# Planta G(s)
num_g = [1]
den_g = [I, d, 0]
sys_g = TransferFunction(num_g, den_g)

# Función para TF en lazo cerrado con ganancia proporcional
def closed_loop_p_tf(I, d, Kp):
    num = [Kp]
    den = [I, d, Kp]
    return TransferFunction(num, den)

# Función objetivo (ISE + esfuerzo de control)
def objective_pid_manual(x, alpha=1.0, beta=0.01):
    Kp, Ki, Kd = x
    if np.any(np.array(x) < 0):
        return 1e6

    t = np.linspace(0, 10, 1000)
    r = np.ones_like(t)

    y = np.zeros_like(t)
    u = np.zeros_like(t)
    e = np.zeros_like(t)

    integral = 0
    prev_error = 0
    dt = t[1] - t[0]

    for i in range(1, len(t)):
        e[i] = r[i] - y[i-1]
        integral += e[i] * dt
        derivative = (e[i] - prev_error) / dt

        u[i] = Kp * e[i] + Ki * integral + Kd * derivative
        u[i] = np.clip(u[i], -1, 1)  # Saturación del control

        prev_error = e[i]

        if i == 1:
            y_dot = 0
        else:
            y_dot = (y[i-1] - y[i-2]) / dt

        y_ddot = (u[i] - d * y_dot) / I
        y[i] = y[i-1] + y_dot * dt + 0.5 * y_ddot * dt**2

    ise = np.trapezoid(e**2, t)
    iue = np.trapezoid(u**2, t)
    J = alpha * ise + beta * iue

    return J

# Optimización de ganancias PID
x0 = [1, 1, 1]
bounds = [(0, 1), (0, 1), (0, 1)]
result = minimize(objective_pid_manual, x0, bounds=bounds, method='L-BFGS-B')

Kp_opt, Ki_opt, Kd_opt = result.x
print(f"Ganancias optimizadas:\nKp={Kp_opt:.4f}, Ki={Ki_opt:.4f}, Kd={Kd_opt:.4f}")

# Simulación final con las ganancias optimizadas
t = np.linspace(0, 10, 1000)
r = np.ones_like(t)
y = np.zeros_like(t)
u = np.zeros_like(t)
e = np.zeros_like(t)

integral = 0
prev_error = 0
dt = t[1] - t[0]

for i in range(1, len(t)):
    e[i] = r[i] - y[i-1]
    integral += e[i] * dt
    derivative = (e[i] - prev_error) / dt

    u[i] = Kp_opt * e[i] + Ki_opt * integral + Kd_opt * derivative
    u[i] = np.clip(u[i], -10., 0.1)  # Saturación del control

    prev_error = e[i]

    if i == 1:
        y_dot = 0
    else:
        y_dot = (y[i-1] - y[i-2]) / dt

    y_ddot = (u[i] - d * y_dot) / I
    y[i] = y[i-1] + y_dot * dt + 0.5 * y_ddot * dt**2

# Graficar resultados
plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
plt.plot(t, r, 'k--', label='Referencia')
plt.plot(t, y, 'b', label='Salida')
plt.ylabel('Salida')
plt.title('Respuesta del sistema con saturación de control ±1')
plt.legend()
plt.grid()

plt.subplot(2, 1, 2)
plt.plot(t, u, 'r', label='Señal de control u(t)')
plt.xlabel('Tiempo [s]')
plt.ylabel('Control (torque)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
