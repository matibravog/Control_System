import sympy as sp

# Definir variables
x = sp.Matrix(sp.symbols('x1:7'))  # x1, x2, ..., x6

# Definir momentos y momentos de inercia como variables simbólicas
Lp, Mp, Np, ix, iy, iz = sp.symbols('Lp Mp Np ix iy iz')

# p         0 
# q         1
# r         2
# phi       3   
# theta     4
# psi       5

# Definir función vectorial f(x)
f = sp.Matrix([
    (Lp - x[1]*x[2]*(iz-iy))/ix,
    (Mp - x[0]*x[2]*(ix-iz))/iy,
    (Np - x[0]*x[1]*(iy-ix))/iz,
    x[0] + (x[1]*sp.sin(x[3])+x[2]*sp.cos(x[3]))*sp.tan(x[4]),
    x[1]*sp.cos(x[3]) - x[2]*sp.sin(x[3]),
    (x[1]*sp.sin(x[3]) + x[2]*sp.cos(x[3]))/sp.cos(x[4]),
])

# Inicializar impresión para mejorar el formato en la terminal
sp.init_printing()

# Derivar: Jacobiano de f con respecto a x
J = f.jacobian(x)

# Imprimir el Jacobiano completo
print("Jacobiano completo (J):")
sp.pretty_print(J)
print("\n")

# Imprimir el Jacobiano columna por columna
print("Jacobiano por columnas:")
for i in range(J.shape[1]):
    print(f"Columna {i + 1}:")
    sp.pretty_print(J[:, i])  # Imprimir cada columna
    print()  # Línea en blanco para separar columnas

# Crear un vector x = 0
x_zero = sp.Matrix([0, 0, 0, 0, 0, 0])

# Evaluar f(x) en x = 0
f_at_zero = f.subs([(x[i], 0) for i in range(len(x))])
print("f(x) evaluado en x = 0:")
sp.pretty_print(f_at_zero)
print("\n")

# Evaluar el Jacobiano J en x = 0
J_at_zero = J.subs([(x[i], 0) for i in range(len(x))])
print("Jacobiano (J) evaluado en x = 0:")
sp.pretty_print(J_at_zero)
print("\n")

# Multiplicar el Jacobiano evaluado en x = 0 por el vector x evaluado en x = 0
result = J_at_zero * x_zero
print("Resultado de J(x=0) * x(x=0):")
sp.pretty_print(result)
print("\n")

# Comparar el resultado con f(x=0)
print("Comparación de J(x=0) * x(x=0) con f(x=0):")
if result == f_at_zero:
    print("El resultado coincide con f(x=0).")
else:
    print("El resultado NO coincide con f(x=0).")
    print("Diferencia:")
    sp.pretty_print(result - f_at_zero)
