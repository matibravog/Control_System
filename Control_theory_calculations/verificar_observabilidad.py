import numpy as np
from numpy.linalg import matrix_rank

def verificar_observabilidad(A, C):
    n = A.shape[0]
    O = np.vstack([C @ np.linalg.matrix_power(A, i) for i in range(n)])
    rango = matrix_rank(O)
    return rango == n

# Ejemplo
A = np.array([
    [0, 1*np.cos(0), -1*np.sin(0)],
    [1, 1*np.sin(0)*np.tan(0), 1*np.cos(0)*np.tan(0)],
    [0, 1*np.sin(0)/np.cos(0), 1*np.cos(0)/np.cos(0)]
])
C = np.array([[1, 1, 1]])

if verificar_observabilidad(A, C):
    print("✅ El sistema es observable. Puedes usar Kalman.")
else:
    print("❌ El sistema NO es observable. Kalman no funcionará bien.")
