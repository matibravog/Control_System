import numpy as np
import control as ct

def eq_lin(eq_point):
    # Physical parameters
    m = 100
    l = 2
    r = 0.1
    Ixx = (1/2)*m*r**2   # Moment of inertia around X axis (kg*m^2)
    Iyy = (1/12)*m*l**2  # Moment of inertia around Y axis (kg*m^2)
    Izz = (1/12)*m*l**2  # Moment of inertia around Z axis (kg*m^2)
    Ip = (Izz-Iyy)/Ixx
    Iq = (Ixx-Izz)/Iyy
    Ir = (Iyy-Ixx)/Izz
    
    p0        = eq_point[0]
    q0        = eq_point[1]
    r0        = eq_point[2]
    phidot0   = eq_point[3]
    thetadot0 = eq_point[4]
    psidot0   = eq_point[5]
    phi0      = eq_point[6]
    theta0    = eq_point[7]
    psi0      = eq_point[8]
    
    # state vector: x = [p,q,r,phi,theta,psi], all in delta format
    # input vector: u = [L,M,N], all in delta format
    A1 =  [0,-Ip*r0,-Ip*q0,0,0,0]
    A2 =  [-Iq*r0,0,-Iq*p0,0,0,0]
    A3 =  [-Ir*q0,-Ir*p0,0,0,0,0]
    A4  = [1,np.sin(phi0)*np.tan(theta0),np.cos(phi0)*np.tan(theta0)]
    A42 = [np.tan(theta0)*(q0*np.cos(phi0)-r0*np.sin(phi0)),(1/(np.cos(theta0))**2)*(q0*np.sin(phi0)+r0*np.cos(phi0)),0]
    A4.extend(A42)
    A5 =  [0,np.cos(phi0),-np.sin(phi0),-(q0*np.sin(phi0)+r0*np.cos(phi0)),0,0]
    A6  = [0,np.sin(phi0)/np.cos(theta0),np.cos(phi0)/np.cos(theta0)]
    A62 = [(q0*np.cos(phi0)-r0*np.sin(phi0))/np.cos(theta0),(q0*np.sin(phi0)+r0*np.cos(phi0))*(np.tan(theta0)/np.cos(theta0)),0]
    A6.extend(A62)
    Alin = np.array([A1,A2,A3,A4,A5,A6])
    Blin = [[1/Ixx,0,0],[0,1/Iyy,0],[0,0,1/Izz],[0,0,0],[0,0,0],[0,0,0]]
    return Alin,Blin

# Define equilibrium point
# Not modifiable
p_eq        = 0
q_eq        = 0
r_eq        = 0
phidot_eq   = 0
thetadot_eq = 0
psidot_eq   = 0
L_eq        = 0
M_eq        = 0
N_eq        = 0

# modifiable
phi_eq      = 0
theta_eq    = 30*np.pi/180
psi_eq      = 0

# Compute linearised state and input matrices A and B so that xdot = Ax+Bu
# Note x = [p,q,r,phi,theta,psi] and u = [L,M,N], all in delta format
eq = np.array([p_eq,q_eq,r_eq,phidot_eq,thetadot_eq,psidot_eq,phi_eq,theta_eq,psi_eq,L_eq,M_eq,N_eq])
A,B = eq_lin(eq)

# Define measurement matrix C
C = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0]])

# Compute the observability matrix
OBS = ct.obsv(A,C)
rank = np.linalg.matrix_rank(OBS)
print(A)
print(B)
print(OBS.shape)
print(rank)