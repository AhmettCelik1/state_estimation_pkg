import symforce.symbolic as sf
from sympy import *

# Define symbolic variables
x, y, theta, L_x, L_y = symbols("x y theta L_x L_y")

# Robot's pose (x, y, theta) and the landmark position (L_x, L_y)
robot_pose = sf.Pose2(t=sf.V2(x, y), R=sf.Rot2.from_angle(theta))  
landmark = sf.V2(L_x, L_y)

# Transform the landmark into the robot's local frame
landmark_local = robot_pose.inverse() * landmark

# The relative x and y measurements
x_rel = landmark_local[0]
y_rel = landmark_local[1]

# Measurement model
z = Matrix([x_rel, y_rel])
print("------------")
print(z)
print("------------")

# Define the state variables as a vector
x_states = Matrix([x, y, theta])

# Compute the Jacobian (partial derivatives of z with respect to x, y, theta)
jacobian = Matrix([[diff(z[0], var), diff(z[1], var)] for var in x_states])

# print("Jacobian of the measurement model:")
# print(jacobian)
