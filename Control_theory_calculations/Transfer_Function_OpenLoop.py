import sympy as sp

# =========================
# Define symbols
# =========================
t = sp.symbols('t')
s = sp.symbols('s')

# Angles (functions of time)
phi = sp.Function('phi')(t)
theta = sp.Function('theta')(t)
psi = sp.Function('psi')(t)

# Angular velocities (functions of time)
p = sp.Function('p')(t)
q = sp.Function('q')(t)
r = sp.Function('r')(t)

# Moments of inertia and damping
Ixx, Iyy, Izz = sp.symbols('Ixx Iyy Izz', positive=True)
d = sp.symbols('d', positive=True)  # damping

# Torques (input functions)
Tx = sp.Function('Tx')(t)
Ty = sp.Function('Ty')(t)
Tz = sp.Function('Tz')(t)

# =========================
# Define the linearized equations
# =========================
# Assumption: small angles => sin ~ x, cos ~ 1, tan ~ x

# Decoupled linear equations:
# p_dot = (Tx - d*p) / Ixx
# q_dot = (Ty - d*q) / Iyy
# r_dot = (Tz - d*r) / Izz
# phi_dot = p
# theta_dot = q
# psi_dot = r

# So:
# phï = ṗ = (Tx - d*phi̇) / Ixx → s²Φ(s) = (Tx(s) - d*sΦ(s)) / Ixx
# thetä = q̇ = (Ty - d*thetȧ) / Iyy
# psï = ṙ = (Tz - d*psi̇) / Izz

# Define Laplace transforms
Phi, Theta, Psi = sp.symbols('Phi Theta Psi', real=True)
Tx_s, Ty_s, Tz_s = sp.symbols('Tx_s Ty_s Tz_s')

# === Roll ===
eq_phi = sp.Eq(s**2 * Phi, (Tx_s - d * s * Phi) / Ixx)
TF_phi = sp.simplify(sp.solve(eq_phi, Phi)[0] / Tx_s)

# === Pitch ===
eq_theta = sp.Eq(s**2 * Theta, (Ty_s - d * s * Theta) / Iyy)
TF_theta = sp.simplify(sp.solve(eq_theta, Theta)[0] / Ty_s)

# === Yaw ===
eq_psi = sp.Eq(s**2 * Psi, (Tz_s - d * s * Psi) / Izz)
TF_psi = sp.simplify(sp.solve(eq_psi, Psi)[0] / Tz_s)

# =========================
# Display Transfer Functions
# =========================

print("\n=== Transfer Function: Roll (Φ(s)/Tx(s)) ===")
sp.pprint(TF_phi)

print("\n=== Transfer Function: Pitch (Θ(s)/Ty(s)) ===")
sp.pprint(TF_theta)

print("\n=== Transfer Function: Yaw (Ψ(s)/Tz(s)) ===")
sp.pprint(TF_psi)
