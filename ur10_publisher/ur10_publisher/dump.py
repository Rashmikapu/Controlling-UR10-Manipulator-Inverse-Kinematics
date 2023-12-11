import numpy as np
import matplotlib.pyplot as plt
import sympy 
import json



def Transformation(theta, alpha, a,d):
    transformation_matrix = sympy.Matrix(
        [[sympy.cos(theta), -sympy.sin(theta)*sympy.cos(alpha), sympy.sin(theta)*sympy.sin(alpha), a*sympy.cos(theta)], 
        [sympy.sin(theta), sympy.cos(theta)*sympy.cos(alpha), -sympy.cos(theta)*sympy.sin(alpha), a*sympy.sin(theta)],
        [0, sympy.sin(alpha), sympy.cos(alpha), d],
        [0, 0, 0, 1]])
    return transformation_matrix


def Kinematics() :
    a,d, alpha,theta, t1, t2, t3, t4, t5, t6 = sympy.symbols('a d alpha theta theta1 theta2 theta3 theta4 theta5 theta6')
    # Declaring link lengths as symbols
    d1,d3,d5,d7,a3 = sympy.symbols('d1, d3, d5, d7, a3')


    T1= Transformation(t1,sympy.pi/2,0, 0.128+0.5)  # Offset from world 0,0,0
    # T1 = Transformation(t1, -sympy.pi/2, 0, 0.128)
    # print ("T1=")
    # sympy.pprint(T1)

    T2= Transformation(t2,0,0.6127,-0.176)
    # T2 = Transformation(t2, sympy.pi, -0.6127, 0)
    # print ("T2=")
    # sympy.pprint(T2)

    T3= Transformation(t3,-sympy.pi, 0.5716, 0.1639)
    # T3 = Transformation(t3, 0, -0.5716, 0)
    # print ("T3=")
    # sympy.pprint(T3)

    T4= Transformation( t4,sympy.pi/2,0,0.1639)
    # T4 = Transformation(t4, -sympy.pi/2, 0, 0.1639)
    # print ("T4=")
    # sympy.pprint(T4)

    T5= Transformation(t5, -sympy.pi/2, 0 , -0.1157)
    # T5 = Transformation(t5, -sympy.pi/2, 0, 0.1157)
    # print ("T5=")
    # sympy.pprint(T5)

    T6= Transformation(t6, 0, 0, 0.1922)
    # T6 = Transformation(t6, 0,0,0.1922)
    # print ("T6=")
    # sympy.pprint(T6)

    # Transformation matrix of end effector w.r.t. fixed base frame
    Final_Transformation = T1*T2*T3*T4*T5*T6


    T0_1 = T1
    T0_2 = T0_1 * T2
    T0_3 = T0_2 * T3
    T0_4 = T0_3 * T4
    T0_5 = T0_4 * T5
    T0_6 = T0_5 * T6



    # Find Z0_i for each matrix 
    Z0_0 = sympy.Matrix([0.0,0.0,1.0])
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

    # Find jacobian matrix by arranging the above computed matrices
    Jacobian=sympy.Matrix([[M1,M2,M3,M4,M5,M6],
                        [Z0_0, Z0_1,Z0_2,Z0_3,Z0_4,Z0_5]])

    print("Jacobian Matrix =")
    print(Jacobian)

    # plot the trajectory
    num_points=100
    Time=20 
    delta_t=Time/num_points
    t=sympy.symbols('t')
    w=2*sympy.pi/20
    r=0.200

    # Linear velocities w.r.t time. Angular velocities are zero
    th=sympy.symbols('th')

    y_dot=0
    z_dot = w*r*(sympy.cos(th))
    x_dot = -w*r*(sympy.sin(th))

    # Set time to 0. 
    time=0

    # Joint angle and position matrices
    # 0.00001 to avoid singularity
    # q=sympy.Matrix([[0.0000, sympy.pi/2, 0.0, sympy.pi/2, 0.00001, 0.0000]])
    q=sympy.Matrix([[0.0, 0.0, 0.0001, 0.0, 0.0001, 0.0]])
    q=q.transpose()

    X_DOT=sympy.Matrix([[x_dot, y_dot, z_dot, 0.0, 0.0, 0.0]])
    X_DOT=X_DOT.transpose()

    # Substitute joint angles to extract transformation of end effector wrt base frame
    T0_6_curr=T0_6.subs([(t1, q[0].evalf(4)), (t2, q[1].evalf(4)), (t3, q[2].evalf(4)), (t4, q[3].evalf(4)), (t5, q[4].evalf(4)),(t6, q[5].evalf(4))])
    print("T06_curr : ")
    sympy.pprint(T0_6_curr)
    # offset_link2 = -1.57
    # offset_link4 = -1.57
    # angles = Float64MultiArray()
    #  joint_position_pub.publish([float(q[0]), float(q[1]), float(q[2]), float(q[3]), float(q[4]), float(q[5])])
    angles = []
    for i in np.linspace(float(sympy.pi/2), float(5*sympy.pi/2),num= num_points):

        # angles.data = [float(q[0]), float(q[1]), float(q[2]), float(q[3]), float(q[4]), float(q[5])]
        # for count in range(500):
        #      joint_position_pub.publish(angles)
        for k in range(6) :
            q[k] = float((q[k] + sympy.pi) % (2 * sympy.pi) - sympy.pi)
        angles.append(q)
        # print(angles)
        # Increment time 
        X_DOT_curr=X_DOT.subs(th,i)
        # Compute joint angles for next point to be plotted
        Jacobian_curr=Jacobian.subs([(t1,q[0].evalf(4)), (t2,q[1].evalf(4)), (t3, q[2].evalf(4)), (t4, q[3].evalf(4)), (t5, q[4].evalf(4)),(t6, q[5].evalf(4))])

        # Find Inverse of Jacobian
        Jacobian_curr=np.array(Jacobian_curr)
        Jacobian_curr=Jacobian_curr.astype(np.float32)
        Jacobian_inv=np.linalg.pinv(Jacobian_curr)

        # Compute q_dot
        q_dot=Jacobian_inv*X_DOT_curr

        # Next q by numerical integration
        q=q+(q_dot*delta_t)
        
        T0_6_curr=T0_6.subs([(t1, q[0].evalf(4)), (t2, q[1].evalf(4)), (t3, q[2].evalf(4)), (t4, q[3].evalf(4)), (t5, q[4].evalf(4)),(t6, q[5].evalf(4))])
        temp = np.array(T0_6_curr)
        # ax.scatter(T0_6_curr[0][3].evalf(4),T0_6_curr[1][3].evalf(4),T0_6_curr[2][3].evalf(4))
        # print(temp[0][3],temp[1][3],temp[2][3])
        # ax.scatter(temp[0][3],temp[1][3],temp[2][3])
        # plt.pause(delta_t)
        time=time+delta_t
    # angles = angles.toList()
    matrix_as_list = matrix_to_list(angles)
    file_path = '/home/rashmikapu/Desktop/Modeling/ros2_ws/src/list_of_lists.json'
    with open(file_path, 'w') as file:
        json.dump(matrix_as_list, file)

    # file_path_input = 'input_file.txt'
    with open(file_path, 'r') as input_file:
        loaded_list_of_lists = json.load(input_file)
    # print(loaded_list_of_lists[1])

def matrix_to_list(matrix):
    return [[float(element) for element in row] for row in matrix]



def main(args=None):
    # rclpy.init(args=args)

    # control_publisher = Control_publisher()

    # rclpy.spin(control_publisher)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # control_publisher.destroy_node()
    # rclpy.shutdown()
    Kinematics()


if __name__ == '__main__':
    main()
    