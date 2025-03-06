
import numpy as np
import matplotlib.pyplot as plt
from sympy import *



# Initialize  extended kalman filter state
# x = [x1,x2,x3,x4,x5]
# x1 = x
# x2 = y
# x3 = yaw
# x4 = v_x
# x5 = v_y
x=MatrixSymbol('x',5,1)

# Initialize control input
# u = [yaw_rate, ax, ay]
u=MatrixSymbol('u',3,1)

# Initialize noise input
# n = [n_omega, n_ax, n_ay]
n=MatrixSymbol('n',3,1)

#parameters
dt=symbols('dt')

# the process model 
# x_{k+1} = x_k + v_x_k*dt*cos(yaw_k) - v_y_k*dt*sin(yaw_k)
# y_{k+1} = y_k + v_x_k*dt*sin(yaw_k) + v_y_k*dt*cos(yaw_k)
# yaw_{k+1} = yaw_k + omega_k*dt + n_omega_k*dt
# v_x_{k+1} = v_x_k + ax_k*dt + dt*v_y_k*omega_k + n_ax_k*dt + dt*v_y_k*n_omega_k
# v_y_{k+1} = v_y_k + ay_k*dt - dt*v_x_k*omega_k + n_ay_k*dt - dt*v_x_k*n_omega_k
# x[0] = x_k
# x[1] = y_k
# x[2] = yaw_k
# x[3] = v_x_k
# x[4] = v_y_k
f=Matrix([x[0]+x[3]*dt*cos(x[2])-x[4]*dt*sin(x[2]),
            x[1]+x[3]*dt*sin(x[2])+x[4]*dt*cos(x[2]),
            x[2]+u[0]*dt + n[0]*dt,
            x[3]+u[1]*dt + dt*x[4]*u[0] + n[1]*dt + dt*x[4]*n[0],
            x[4]+u[2]*dt - dt*x[3]*u[0] + n[2]*dt - dt*x[3]*n[0]])


f=f.subs({dt:0.1})

JacobianState =f.jacobian(x)
JacobianInput =f.jacobian(u)
JacobianNoise =f.jacobian(n)


xvector=np.array([[0.2],[0.2],[0.2],[0.2],[0.2]])
uvector=np.array([[0.1],[0.1],[0.1]])
nvector=np.array([[0.0],[0.0],[0.0]])

x_vector = np.array([[0.0],[0.0],[0.0],[0.0],[0.0]])

P = np.eye(5, 5)

test=f.subs(x,Matrix(xvector)).subs(u,Matrix(uvector)).subs(n,Matrix(nvector))

for i in range(5):
    x_vector[i]=test[i]
    
F=JacobianState.subs(x,Matrix(xvector)).subs(u,Matrix(uvector)).subs(n,Matrix(nvector))
P = F @ P @ F.T + np.eye(5)

print("Jacobian P matrix")
for i in range(5):
    row = []
    for j in range(5):
        row.append(P[i, j])
    print(row)

