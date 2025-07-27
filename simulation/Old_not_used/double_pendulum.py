import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Parámetros físicos
L = 1.0       # Longitud de la barra (m)
g = 9.81      # Gravedad (m/s^2)

# Dinámica del péndulo esférico sin torque externo
def spherical_pendulum(t, y):
    theta, phi, omega_theta, omega_phi = y

    theta_ddot = (np.sin(theta) * np.cos(theta) * omega_phi**2 - (g / L) * np.sin(theta))
    phi_ddot = (-2 * omega_theta * omega_phi / np.tan(theta + 1e-8))
    return [omega_theta, omega_phi, theta_ddot, phi_ddot]

# Condiciones iniciales
theta0 = np.radians(0)
phi0 = np.radians(0)
omega_theta0 = 1
omega_phi0 = 0

# solve edo
t_span = (0, 15)
t_eval = np.linspace(*t_span, 2000)
y0 = [theta0, phi0, omega_theta0, omega_phi0]
sol = solve_ivp(spherical_pendulum, t_span, y0, t_eval=t_eval)

theta = sol.y[0]
phi = sol.y[1]
omega_theta = sol.y[2]
omega_phi = sol.y[3]

# Convertir a coordenadas cartesianas para visualización
x = L * np.sin(theta) * np.cos(phi)
y = L * np.sin(theta) * np.sin(phi)
z = -L * np.cos(theta)

pitch = np.arctan2(x, -z)  # rotación alrededor de Y (eje X local)
roll = np.arctan2(y, -z)   # rotación alrededor de X (eje Y local)

pitch_dot = np.gradient(pitch, t_eval)
roll_dot = np.gradient(roll, t_eval)

# Ahora puedes graficar pitch y roll en grados:
plt.figure(figsize=(10, 4))
plt.plot(sol.t, np.degrees(pitch), label='Pitch (deg)', color='orange')
plt.plot(sol.t, np.degrees(roll), label='Roll (deg)', color='purple')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.title('Pendulum Pitch and Roll (from Spherical Coordinates)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# Graficar velocidades angulares de pitch y roll
plt.figure(figsize=(10, 4))
plt.plot(sol.t, pitch_dot, label='Pitch_dot (rad/s)', color='orange')
plt.plot(sol.t, roll_dot, label='Roll_dot (rad/s)', color='purple')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Pendulum Pitch and Roll Angular Velocities')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# Gráfico de ángulos
plt.figure(figsize=(10, 4))
plt.plot(sol.t, np.degrees(theta), label='θ (deg)', color='blue')
plt.plot(sol.t, np.degrees(phi), label='φ (deg)', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.title('Spherical Pendulum Angles')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# Gráfico de velocidades angulares
plt.figure(figsize=(10, 4))
plt.plot(sol.t, omega_theta, label='θ_dot (rad/s)', color='blue')
plt.plot(sol.t, omega_phi, label='φ_dot (rad/s)', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Spherical Pendulum Angular Velocities')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# 3D Animation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-L, L])
ax.set_ylim([-L, L])
ax.set_zlim([-L, 0.1])
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Animated 3D Spherical Pendulum')

ax.scatter([0], [0], [0], color='red', label='Pivot')
line, = ax.plot([], [], [], 'o-', lw=2, color='blue', label='Pendulum')
trace, = ax.plot([], [], [], color='cyan', alpha=0.5, label='Tip Trace')

def init():
    line.set_data([], [])
    line.set_3d_properties([])
    trace.set_data([], [])
    trace.set_3d_properties([])
    return line, trace

def animate(i):
    line.set_data([0, x[i]], [0, y[i]])
    line.set_3d_properties([0, z[i]])
    trace.set_data(x[:i+1], y[:i+1])
    trace.set_3d_properties(z[:i+1])
    return line, trace

ani = animation.FuncAnimation(
    fig, animate, frames=len(x), init_func=init,
    interval=1000 * (sol.t[1] - sol.t[0]), blit=True
)

ax.legend()
plt.show()
