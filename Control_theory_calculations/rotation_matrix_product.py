import sympy as sp

# Define symbols for the angles and the vector
phi, theta, psi = sp.symbols('phi theta psi')  # Rotation angles
ax, ay, az = sp.symbols('ax ay az')  # Axis components (not used in this example)
a = sp.Matrix([ax, ay, az])  # Axis vector (not used in this example)

# Define the rotation matrix (example: ZYX rotation order)
Rx = sp.Matrix([
    [1, 0, 0],
    [0, sp.cos(phi), -sp.sin(phi)],
    [0, sp.sin(phi), sp.cos(phi)]
])

Ry = sp.Matrix([
    [sp.cos(theta), 0, sp.sin(theta)],
    [0, 1, 0],
    [-sp.sin(theta), 0, sp.cos(theta)]
])

Rz = sp.Matrix([
    [sp.cos(psi), -sp.sin(psi), 0],
    [sp.sin(psi), sp.cos(psi), 0],
    [0, 0, 1]
])



# Combined rotation matrix (Rz * Ry * Rx)
R = Rx@Ry
Ra = R*a

# Display each rotation matrix
print("Rotation matrix Rz (around Z-axis):")
sp.pprint(Rz)
print("\nRotation matrix Ry (around Y-axis):")
sp.pprint(Ry)
print("\nRotation matrix Rx (around X-axis):")
sp.pprint(Rx)

# Print the result of the multiplication of the matrices
print("\nResult of the multiplication of the matrices (Ry * Rx):")
sp.pprint(R)

# Print the result of the multiplication of the matrices
print("\nResult of the multiplication of the matrices (R*a):")
sp.pprint(Ra)

# Define the vector w with components x, y, z
# x, y, z = sp.symbols('x y z')
# w = sp.Matrix([x, y, z])

# # Compute the product of the rotation matrix R and the vector w
# Rw = R * w

# # Print the vector w
# print("\nVector w:")
# sp.pprint(w)

# # Print the result of the product R * w
# print("\nResult of the product R * w:")
# sp.pprint(Rw)

# # Compute the product of the transposed rotation matrix R.T and the vector w
# Rw_transposed = R.T * w

# # Print the result of the product R.T * w
# print("\nResult of the product R.T * w:")
# sp.pprint(Rw_transposed)
