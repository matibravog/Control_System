import sympy as sp

# Laplace variable
s = sp.symbols('s')

# Constantes simbólicas para cada eje
Ixx, Iyy, Izz = sp.symbols('Ixx Iyy Izz', positive=True)
d = sp.symbols('d', positive=True)

# PID ganancias (pueden ser distintas por eje si quieres)
Kp, Ki, Kd = sp.symbols('Kp Ki Kd', real=True)

# ========================================
# Transferencia planta (misma forma en los 3 ejes)
# G(s) = 1 / (I * s^2 + d * s)
# PID = (Kd s² + Kp s + Ki) / s
# ========================================
def closed_loop_tf(I):
    G = 1 / (I * s**2 + d * s)
    C = (Kd * s**2 + Kp * s + Ki) / s
    T = sp.simplify((G * C) / (1 + G * C))
    return T

# Transferencias para cada ángulo
T_roll = closed_loop_tf(Ixx)
T_pitch = closed_loop_tf(Iyy)
T_yaw = closed_loop_tf(Izz)

# Mostrar resultados
print("=== Función de transferencia en lazo cerrado: ROLL ===")
sp.pprint(T_roll, use_unicode=True)

print("\n=== Función de transferencia en lazo cerrado: PITCH ===")
sp.pprint(T_pitch, use_unicode=True)

print("\n=== Función de transferencia en lazo cerrado: YAW ===")
sp.pprint(T_yaw, use_unicode=True)
