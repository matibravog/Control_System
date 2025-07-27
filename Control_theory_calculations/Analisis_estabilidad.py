import numpy as np
import matplotlib.pyplot as plt

# Parámetros numéricos (mismos que antes)
Ixx = (1/12)*1.0*0.2**2
Iyy = (1/12)*1.0*0.5**2
Izz = Iyy
d = 0.025
Kp = 1.0
Ki = 0.5
Kd = 0.1

def calcular_polos(I, d, Kp, Ki, Kd):
    coef = [I, d + Kd, Kp, Ki]
    polos = np.roots(coef)
    return polos

# Calcular polos para cada eje
polos_roll = calcular_polos(Ixx, d, Kp, Ki, Kd)
polos_pitch = calcular_polos(Iyy, d, Kp, Ki, Kd)
polos_yaw = calcular_polos(Izz, d, Kp, Ki, Kd)

# Plot polos
plt.figure(figsize=(8,6))
plt.axvline(0, color='k', lw=1)  # Línea eje imaginario
plt.axhline(0, color='k', lw=1)  # Línea eje real

plt.scatter(polos_roll.real, polos_roll.imag, marker='o', color='r', label='Roll Poles')
plt.scatter(polos_pitch.real, polos_pitch.imag, marker='x', color='g', label='Pitch Poles')
plt.scatter(polos_yaw.real, polos_yaw.imag, marker='^', color='b', label='Yaw Poles')

plt.title("Diagrama de polos en el plano complejo")
plt.xlabel("Parte Real")
plt.ylabel("Parte Imaginaria")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
