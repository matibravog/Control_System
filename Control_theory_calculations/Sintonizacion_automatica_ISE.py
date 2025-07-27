import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import TransferFunction, step, lsim

# Parámetros físicos (ejemplo eje Roll)
I = (1/12)*1.0*0.05**2
d = 0.025

def closed_loop_pid_tf(I, d, Kp, Ki, Kd):
    num_pid = [Kd, Kp, Ki]
    den_pid = [1, 0]  # s (denominador del PID)
    num_g = [1]
    den_g = [I, d, 0]
    num_ol = np.polymul(num_pid, num_g)
    den_ol = np.polymul(den_pid, den_g)
    den_cl = np.polyadd(den_ol, num_ol)
    return TransferFunction(num_ol, den_cl)

def objective(x):
    Kp, Ki, Kd = x
    if np.any(np.array(x) < 0):
        return 1e6
    sys_cl = closed_loop_pid_tf(I, d, Kp, Ki, Kd)
    t = np.linspace(0, 10, 1000)
    t_out, y = step(sys_cl, T=t)
    e = 1 - y
    ISE = np.trapz(e**2, t)
    return ISE

# Optimización
from scipy.optimize import minimize
x0 = [1.0, 1.0, 0.1]
bounds = [(0, 50), (0, 50), (0, 10)]
result = minimize(objective, x0, bounds=bounds, method='L-BFGS-B')

if result.success:
    Kp_opt, Ki_opt, Kd_opt = result.x
    print(f"Ganancias óptimas:\nKp={Kp_opt:.4f}, Ki={Ki_opt:.4f}, Kd={Kd_opt:.4f}")
else:
    raise ValueError("Optimización fallida")

# Simulación de salida
sys_opt = closed_loop_pid_tf(I, d, Kp_opt, Ki_opt, Kd_opt)
t = np.linspace(0, 10, 1000)
t_out, y = step(sys_opt, T=t)

# Calcular acción de control u(t)
# u(t) = Kp*e + Ki*∫e dt + Kd*de/dt
e = 1 - y
dt = t[1] - t[0]
integral_e = np.cumsum(e)*dt
derivative_e = np.diff(e)/dt
derivative_e = np.append(derivative_e, derivative_e[-1])  # igualamos último valor para mantener tamaño

u = Kp_opt * e + Ki_opt * integral_e + Kd_opt * derivative_e

# Graficar resultados
plt.figure(figsize=(12,5))
plt.subplot(1,2,1)
plt.plot(t_out, y, label='Salida (Ángulo [rad])')
plt.plot(t_out, np.ones_like(t_out), 'k--', label='Setpoint')
plt.title('Respuesta al Escalón')
plt.xlabel('Tiempo [s]')
plt.ylabel('Ángulo [rad]')
plt.grid()
plt.legend()

plt.subplot(1,2,2)
plt.plot(t_out, u, label='Acción de control u(t)')
plt.title('Señal de control')
plt.xlabel('Tiempo [s]')
plt.ylabel('Torque [N·m] (aprox.)')
plt.grid()
plt.legend()

plt.tight_layout()
plt.show()
