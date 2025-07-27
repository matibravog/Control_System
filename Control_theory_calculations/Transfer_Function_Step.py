import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import TransferFunction, step

# Parámetros numéricos
Ixx = (1/12)*1.0*0.2**2     # 0.003333...
Iyy = (1/12)*1.0*0.5**2     # 0.020833...
Izz = Iyy                   # Igual que Iyy
d = 0.025

Kp = 10.0
Ki = 0.0
Kd = 0.0

# Crear función para obtener TF en lazo cerrado
def closed_loop_tf(I, d, Kp, Ki, Kd):
    # Numerador PID: Kd s^2 + Kp s + Ki
    num = [Kd, Kp, Ki]
    # Denominador: I s^3 + (d + Kd) s^2 + Kp s + Ki
    den = [I, d + Kd, Kp, Ki]
    return TransferFunction(num, den)

# Crear TF para cada eje
TF_roll = closed_loop_tf(Ixx, d, Kp, Ki, Kd)
TF_pitch = closed_loop_tf(Iyy, d, Kp, Ki, Kd)
TF_yaw = closed_loop_tf(Izz, d, Kp, Ki, Kd)

# Tiempo para simulación
t = np.linspace(0, 5, 500)

# Simular respuesta al escalón
t_roll, y_roll = step(TF_roll, T=t)
t_pitch, y_pitch = step(TF_pitch, T=t)
t_yaw, y_yaw = step(TF_yaw, T=t)

# Graficar respuestas
plt.figure(figsize=(10,6))
plt.plot(t_roll, y_roll, label='Roll (φ)')
plt.plot(t_pitch, y_pitch, label='Pitch (θ)')
# plt.plot(t_yaw, y_yaw, label='Yaw (ψ)')

plt.title('Respuesta al escalón del sistema en lazo cerrado con PID')
plt.xlabel('Tiempo [s]')
plt.ylabel('Ángulo [rad]')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
