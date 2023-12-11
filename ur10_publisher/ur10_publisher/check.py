from math import sqrt
import numpy as np
import matplotlib.pyplot as plt
import sympy 

# declare symbols
a,d, alpha, theta, t1, t2, t3, t4, t5, t6 = sympy.symbols('a d alpha theta theta1 theta2 theta3 theta4 theta5 theta6')

# Find transformation matrix from given DH parameters
def Transformation(theta, alpha, a,d):
    transformation_matrix = sympy.Matrix(
        [[sympy.cos(theta), -sympy.sin(theta)*sympy.cos(alpha), sympy.sin(theta)*sympy.sin(alpha), a*sympy.cos(theta)], 
         [sympy.sin(theta), sympy.cos(theta)*sympy.cos(alpha), -sympy.cos(theta)*sympy.sin(alpha), a*sympy.sin(theta)],
         [0, sympy.sin(alpha), sympy.cos(alpha), d],
         [0, 0, 0, 1]])

    return transformation_matrix


# Declaring link lengths as symbols
d1,d3,d5,d7,a3 = sympy.symbols('d1 d3 d5 d7 a3')

# Transformation matrices of each frame with respect to previous frames(A1 to A6) and Dummy frame


T1=Transformation(t1,sympy.pi/2,0, 0.128)
# T1 = Transformation(t1, -sympy.pi/2, 0, 0.128)
print ("T1=")
sympy.pprint(T1)

# T2=Transformation((sympy.pi/2)+t2,0,0.6127,0)
T2=Transformation((sympy.pi/2)+t2,0,0.6127,-0.176+0.5)
# T2 = Transformation(t2, sympy.pi, -0.6127, 0)
print ("T2=")
sympy.pprint(T2)

# T3=Transformation(t3,-sympy.pi, 0.5716, 0)
T3=Transformation(t3,-sympy.pi, 0.5716, 0.1639)
# T3 = Transformation(t3, 0, -0.571`6, 0)
print ("T3=")
sympy.pprint(T3)

T4=Transformation((sympy.pi/2) + t4,sympy.pi/2,0,0.1639)
# T4 = Transformation(t4, -sympy.pi/2, 0, 0.1639)
print ("T4=")
sympy.pprint(T4)

T5=Transformation(t5, -sympy.pi/2, 0 , -0.1157)
# T5 = Transformation(t5, -sympy.pi/2, 0, 0.1157)
print ("T5=")
sympy.pprint(T5)

T6=Transformation(t6, 0, 0, 0.1922)
# T6 = Transformation(t6, 0,0,0.1922)
print ("T6=")
sympy.pprint(T6)

# Transformation matrix of end effector w.r.t. fixed base frame
Final_Transformation = T1*T2*T3*T4*T5*T6


T0_1 = T1
T0_2 = T0_1 * T2
T0_3 = T0_2 * T3
T0_4 = T0_3 * T4
T0_5 = T0_4 * T5
T0_6 = T0_5 * T6



# Find Z0_i for each matrix computed above
Z0_0 = sympy.Matrix([0,0,1])
Z0_1=T0_1.col(2)
Z0_1.row_del(3)

Z0_2=T0_2.col(2)
Z0_2.row_del(3)

Z0_3=T0_3.col(2)
Z0_3.row_del(3)

Z0_4=T0_4.col(2)
Z0_4.row_del(3)

Z0_5=T0_5.col(2)
Z0_5.row_del(3)

Z0_6=T0_6.col(2)
Z0_6.row_del(3)

X0_6=T0_6.col(3)
X0_6.row_del(3)

# Find partial derivate of X0_6 w.r.t each joint angle
M1=X0_6.diff(t1)
M2=X0_6.diff(t2)
M3=X0_6.diff(t3)
M4=X0_6.diff(t4)
M5=X0_6.diff(t5)
M6=X0_6.diff(t6)

# Find jacobian matrix using second method
Jacobian=sympy.Matrix([[M1,M2,M3,M4,M5,M6],
                     [Z0_0, Z0_1,Z0_2,Z0_3,Z0_4,Z0_5]])

print("Jacobian Matrix =")
sympy.pprint(Jacobian)

# Let's plot the trajectory now
num_data_points=100
# Trajectrory need to be plotted within 5 sec
Time=20
delta_t=Time/num_data_points

# r*w = perimeter/Total Time =2*pi*r/Time
t=sympy.symbols('t')
w=2*sympy.pi/Time
r=0.100
# k=0.100

# Parametric equation of a circle
y=0.3561
# z=r*sympy.sin(w*t)+0.100
z=0.1328+r*sympy.sin(w*t)
x = r*sympy.cos(w*t)

# Linear velocities w.r.t time. Angular velocities are zero
time1,time2=sympy.symbols('time1 time2')
th=sympy.symbols('th')
y_dot=0
# z_dot=r*(sympy.sin(w*time2)-sympy.sin(w*time1))/delta_t
# x_dot=r*(sympy.cos(w*time2)-sympy.cos(w*time1))/delta_t
z_dot = w*r*(sympy.cos(th))
x_dot = -w*r*(sympy.sin(th))



# Set time to 0. Populate initial position vector and joint angle vector
time=0

q=sympy.Matrix([[0.0, -1.57, 0.0001, -1.57, 0.0001, 0.0]])
q=q.transpose()

# X=sympy.Matrix([[0.0, 0.3561, 1.4280, 0.0, 0, 0]])
# X=X.transpose()

X_DOT=sympy.Matrix([[x_dot, y_dot, z_dot, 0.0, 0.0, 0.0]])
X_DOT=X_DOT.transpose()

# Populate coordinate vector for pen tip w.r.t. end effector frame
# pen_pos_wrt_end_effector=sympy.Matrix([[0, 0, 0, 1]])
# pen_pos_wrt_end_effector=pen_pos_wrt_end_effector.transpose()

ax = plt.axes(projection ="3d")

# ax.axes.set_xlim3d(left=-0.7, right=0.7) 
# ax.axes.set_ylim3d(bottom=0, top=0.5)
# ax.axes.set_zlim3d(bottom=0.0, top=1.5) 



T0_6_curr=T0_6.subs([(t1, q[0].evalf(4)), (t2, q[1].evalf(4)), (t3, q[2].evalf(4)), (t4, q[3].evalf(4)), (t5, q[4].evalf(4)),(t6, q[5].evalf(4))])
print("T06_curr : ")
sympy.pprint(T0_6_curr)
# For loop used to plot data points one by one
# for i in range(0,num_data_points):
    # First plot the point using forward kinematics with known joint angles
for i in np.linspace(float(sympy.pi/2), float(5*sympy.pi/2),num= num_data_points):
    # T0_6_curr=T0_6.subs([(t1, q[0].evalf(4)), (t2, q[1].evalf(4)), (t3, q[2].evalf(4)), (t4, q[3].evalf(4)), (t5, q[4].evalf(4)),(t6, q[5].evalf(4))])
    
    X_DOT_curr=X_DOT.subs(th,i)
    
    # pen_curr=T0_6_curr*pen_pos_wrt_end_effector
    # print("q : ")
    # print(T0_6_curr.shape)
    # print(type(T0_6_curr))
    # sympy.pprint(T0_6_curr)
    # print(q[0].evalf(4))
    # temp = np.array(T0_6_curr)
    # # ax.scatter(T0_6_curr[0][3].evalf(4),T0_6_curr[1][3].evalf(4),T0_6_curr[2][3].evalf(4))
    # ax.scatter(temp[0][3],temp[1][3],temp[2][3])
    # plt.pause(1)
    
    # Compute joint angles for next point to be plotted
    Jacobian_curr=Jacobian.subs([(t1,q[0].evalf(4)), (t2,q[1].evalf(4)), (t3, q[2].evalf(4)), (t4, q[3].evalf(4)), (t5, q[4].evalf(4)),(t6, q[5].evalf(4))])
    # X_DOT_curr=X_DOT.subs([(time1,time),(time2,time+delta_t)])
    # X_DOT_curr=X_DOT.subs(th,i)


    # Find Inverse of Jacobian
    Jacobian_curr=np.array(Jacobian_curr)
    Jacobian_curr=Jacobian_curr.astype(np.float32)
    Jacobian_inv=np.linalg.pinv(Jacobian_curr)

    # Compute q_dot
    q_dot=Jacobian_inv*X_DOT_curr

    # Next q by numerical integration
    q=q+(q_dot*delta_t)
    print(q)
    T0_6_curr=T0_6.subs([(t1, q[0].evalf(4)), (t2, q[1].evalf(4)), (t3, q[2].evalf(4)), (t4, q[3].evalf(4)), (t5, q[4].evalf(4)),(t6, q[5].evalf(4))])
    temp = np.array(T0_6_curr)
    # ax.scatter(T0_6_curr[0][3].evalf(4),T0_6_curr[1][3].evalf(4),T0_6_curr[2][3].evalf(4))
    print(temp[0][3],temp[1][3],temp[2][3])
    ax.scatter(temp[0][3],temp[1][3],temp[2][3])
    plt.pause(delta_t)

    # Increment time 
    time=time+delta_t

# Show plot
plt.axis('equal')
plt.show()
