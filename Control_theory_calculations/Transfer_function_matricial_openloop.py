import sympy as sp

# Variables simbólicas
s = sp.symbols('s', complex=True)  # Variable Laplace

# Parámetros físicos (positivos)
Ixx, Iyy, Izz = sp.symbols('Ixx Iyy Izz', positive=True)
d = sp.symbols('d', positive=True)  # Amortiguamiento/damping

# Variables Laplace para ángulos y torques
Phi, Theta, Psi = sp.symbols('Phi Theta Psi')
Tx_s, Ty_s, Tz_s = sp.symbols('Tx_s Ty_s Tz_s')

# Ecuaciones en Laplace (linealizadas y desacopladas)

# Roll (phi)
eq_phi = sp.Eq(s**2 * Phi, (Tx_s - d * s * Phi) / Ixx)
TF_phi = sp.solve(eq_phi, Phi)[0] / Tx_s
TF_phi = sp.simplify(TF_phi)

# Pitch (theta)
eq_theta = sp.Eq(s**2 * Theta, (Ty_s - d * s * Theta) / Iyy)
TF_theta = sp.solve(eq_theta, Theta)[0] / Ty_s
TF_theta = sp.simplify(TF_theta)

# Yaw (psi)
eq_psi = sp.Eq(s**2 * Psi, (Tz_s - d * s * Psi) / Izz)
TF_psi = sp.solve(eq_psi, Psi)[0] / Tz_s
TF_psi = sp.simplify(TF_psi)

# Mostrar resultados
print("\n=== Función de transferencia Roll (Φ(s)/Tx(s)) ===")
sp.pprint(TF_phi)

print("\n=== Función de transferencia Pitch (Θ(s)/Ty(s)) ===")
sp.pprint(TF_theta)

print("\n=== Función de transferencia Yaw (Ψ(s)/Tz(s)) ===")
sp.pprint(TF_psi)
