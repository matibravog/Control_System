import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import TransferFunction, lsim, step

# Parámetros físicos
Ixx = (1/12)*1.0*0.2**2
d = 0.025

# Función para TF lazo cerrado proporcional puro
def closed_loop_p(I, d, Kp):
    num = [Kp]
    den = [I, d, Kp]
    return TransferFunction(num, den)

# Simula respuesta a entrada escalón
def sim_response(tf, t):
    u = np.ones_like(t)
    t_out, y, _ = lsim(tf, U=u, T=t)
    return t_out, y

# Busca Kcr variando Kp buscando oscilaciones "casi sostenidas"
def find_Kcr(I, d, Kp_start=0.1, Kp_end=50, Kp_step=0.1, t_end=20, tol=0.05):
    t = np.linspace(0, t_end, 2000)
    Kp_vals = np.arange(Kp_start, Kp_end, Kp_step)
    for Kp in Kp_vals:
        tf = closed_loop_p(I, d, Kp)
        t_out, y = sim_response(tf, t)

        y_last = y[-int(0.2*len(y)):]
        peaks = (np.diff(np.sign(np.diff(y_last))) < 0).nonzero()[0] + 1
        if len(peaks) >= 2:
            period_est = t_out[peaks[-1]] - t_out[peaks[-2]]
            amp_diff = np.abs(y_last[peaks[-1]] - y_last[peaks[-2]])
            if amp_diff < tol:
                print(f"Ganancia crítica Kcr encontrada: {Kp:.2f}")
                print(f"Período crítico Tcr estimado: {period_est:.2f} s")
                plt.plot(t_out, y)
                plt.title(f"Respuesta para Kp={Kp:.2f} (Oscilación casi sostenida)")
                plt.xlabel("Tiempo [s]")
                plt.ylabel("Salida")
                plt.grid()
                plt.show()
                return Kp, period_est
    print("No se encontró Kcr en el rango dado")
    return None, None

# Función TF en lazo cerrado PID
def closed_loop_pid(I, d, Kp, Ki, Kd):
    num = [Kd, Kp, Ki]
    den = [I, d + Kd, Kp, Ki]
    return TransferFunction(num, den)

# Función para calcular ganancias Ziegler-Nichols
def ziegler_nichols(Kcr, Tcr):
    Kp = 0.6 * Kcr
    Ki = 2 * Kp / Tcr
    Kd = Kp * Tcr / 8
    return Kp, Ki, Kd

# Tiempo simulación
t_sim = np.linspace(0, 10, 1000)

# Buscar Kcr y Tcr para el eje Roll (puedes repetir para Pitch y Yaw)
Kcr, Tcr = find_Kcr(Ixx, d)
if Kcr is None:
    # Si no encuentra Kcr, asignar valores de respaldo
    Kcr, Tcr = 5.0, 1.0

# Ganancias óptimas ZN
Kp_opt, Ki_opt, Kd_opt = ziegler_nichols(Kcr, Tcr)

print(f"Ganancias críticas (solo P): Kp = {Kcr:.3f}")
print(f"Ganancias Ziegler-Nichols: Kp={Kp_opt:.3f}, Ki={Ki_opt:.3f}, Kd={Kd_opt:.3f}")

# TF con ganancias críticas (solo P)
TF_crit = closed_loop_p(Ixx, d, Kcr)

# TF con ganancias ZN (PID)
# determinadas con optimizacion Kp=4.0052, Ki=0.8986, Kd=2.197 
TF_pid = closed_loop_pid(Ixx, d, 4.0052, 0.8986, 2.197)
# TF_pid = closed_loop_pid(Ixx, d, Kp_opt, Ki_opt, Kd_opt)

# Simular respuestas
t1, y1 = step(TF_crit, T=t_sim)
t2, y2 = step(TF_pid, T=t_sim)

# Graficar comparación
plt.figure(figsize=(10,6))
plt.plot(t1, y1, label='Respuesta con Ganancia Crítica (P puro)')
plt.plot(t2, y2, label='Respuesta con PID Ziegler-Nichols')
plt.title("Comparación Respuesta al Escalón")
plt.xlabel("Tiempo [s]")
plt.ylabel("Ángulo [rad]")
plt.grid()
plt.legend()
plt.tight_layout()
plt.show()
