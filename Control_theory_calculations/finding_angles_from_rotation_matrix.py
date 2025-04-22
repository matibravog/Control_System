import sympy as sp

# Define symbols for the angles and the vector
phi, theta, psi = sp.symbols('phi theta psi')  # Rotation angles
Mbx, Mby, Mbz = sp.symbols('Mbx Mby Mbz')  # Components of the input vector
Mix, Miy, Miz = sp.symbols('Mix Miy Miz')  # Components of the resulting vector

# Define the rotation matrix (example: ZYX rotation order)
Rz = sp.Matrix([
    [sp.cos(psi), -sp.sin(psi), 0],
    [sp.sin(psi), sp.cos(psi), 0],
    [0, 0, 1]
])
Ry = sp.Matrix([
    [sp.cos(theta), 0, sp.sin(theta)],
    [0, 1, 0],
    [-sp.sin(theta), 0, sp.cos(theta)]
])
Rx = sp.Matrix([
    [1, 0, 0],
    [0, sp.cos(phi), -sp.sin(phi)],
    [0, sp.sin(phi), sp.cos(phi)]
])

# Combined rotation matrix (Rz * Ry * Rx)
R = Rz * Ry * Rx

# Define the input vector and resulting vector
input_vector = sp.Matrix([Mbx, Mby, Mbz])
resulting_vector = sp.Matrix([Mix, Miy, Miz])

# Define the equation system: R * input_vector = resulting_vector
equations = R * input_vector - resulting_vector

# Solve the system for the angles
solutions = sp.solve(equations, [phi, theta, psi])

# Display the solutions
sp.pprint(solutions)
