#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        joint_trajectory_errors = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6=symbols('alpha0:7')
            a0,a1,a2,a3,a4,a5,a6=symbols('a0:7')
            # Joint angle symbols
            q1,q2,q3,q4,q5,q6,q7= symbols('q1:8')
            d1,d2,d3,d4,d5,d6,d7= symbols('d1:8')

            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6=symbols('alpha0:7')
            a0,a1,a2,a3,a4,a5,a6=symbols('a0:7')
            # Joint angle symbols
            q1,q2,q3,q4,q5,q6,q7= symbols('q1:8')
            d1,d2,d3,d4,d5,d6,d7= symbols('d1:8')
            # Modified DH params
            s = {alpha0:     0, a0:      0, d1:  0.75,
                 alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
                 alpha2:     0, a2:   1.25, d3:     0,
                 alpha3: -pi/2, a3: -0.054, d4:  1.50,
                 alpha4:  pi/2, a4:      0, d5:     0,
                 alpha5: -pi/2, a5:      0, d6:     0,
                 alpha6:     0, a6:      0, d7: 0.303, q7: 0}

            # Create individual transformation matrices
            # Create individual transformation matrices
            T0_1=Matrix([[cos(q1),-sin(q1),0,a0],
                         [sin(q1)*cos(alpha0),cos(q1)*cos(alpha0),-sin(alpha0),-sin(alpha0)*d1],
                         [sin(q1)*sin(alpha0),cos(q1)*sin(alpha0),cos(alpha0),cos(alpha0)*d1],
                         [0,0,0,1]])
            T0_1=T0_1.subs(S)

            T1_2=Matrix([[cos(q2),-sin(q2),0,a1],
                         [sin(q2)*cos(alpha1),cos(q2)*cos(alpha1),-sin(alpha1),-sin(alpha1)*d2],
                         [sin(q2)*sin(alpha1),cos(q2)*sin(alpha1),cos(alpha1),cos(alpha1)*d2],
                         [0,0,0,1]])
            T1_2=T1_2.subs(S)

            T2_3=Matrix([[cos(q3),-sin(q3),0,a2],
                         [sin(q3)*cos(alpha2),cos(q3)*cos(alpha2),-sin(alpha2),-sin(alpha2)*d3],
                         [sin(q3)*sin(alpha2),cos(q3)*sin(alpha2),cos(alpha2),cos(alpha2)*d3],
                         [0,0,0,1]])
            T2_3=T2_3.subs(S)

            T3_4=Matrix([[cos(q4),-sin(q4),0,a3],
                         [sin(q4)*cos(alpha3),cos(q4)*cos(alpha3),-sin(alpha3),-sin(alpha3)*d4],
                         [sin(q4)*sin(alpha3),cos(q4)*sin(alpha3),cos(alpha3),cos(alpha3)*d4],
                         [0,0,0,1]])
            T3_4=T3_4.subs(S)

            T4_5= Matrix([[cos(q5),-sin(q5),0,a4],
                         [sin(q5)*cos(alpha4),cos(q5)*cos(alpha4),-sin(alpha4),-sin(alpha4)*d5],
                         [sin(q5)*sin(alpha4),cos(q5)*sin(alpha4),cos(alpha4),cos(alpha4)*d5],
                         [0,0,0,1]])
            T4_5=T4_5.subs(S)

            T5_6= Matrix([[cos(q6),-sin(q6),0,a5],
                         [sin(q6)*cos(alpha5),cos(q6)*cos(alpha5),-sin(alpha5),-sin(alpha5)*d6],
                         [sin(q6)*sin(alpha5),cos(q6)*sin(alpha5),cos(alpha5),cos(alpha5)*d6],
                         [0,0,0,1]])
            T5_6=T5_6.subs(S)

            T6_7= Matrix([[cos(q7),-sin(q7),0,a6],
                         [sin(q7)*cos(alpha6),cos(q7)*cos(alpha6),-sin(alpha6),-sin(alpha6)*d7],
                         [sin(q7)*sin(alpha6),cos(q7)*sin(alpha6),cos(alpha6),cos(alpha6)*d7],
                         [0,0,0,1]])
            T6_7=T6_7.subs(S)

            T0_2 = simplify(T0_1 * T1_2)
            T0_3 = simplify(T0_2 * T2_3)
            T0_4 = simplify(T0_3 * T3_4)
            T0_5 = simplify(T0_4 * T4_5)
            T0_6 = simplify(T0_5 * T5_6)

            #Correcting gripper cordinate by 180 on Z-axis and 90 on y-axis
            R_z = Matrix([[           cos(np.pi),         -sin(np.pi),            0,              0],
                           [          sin(np.pi),          cos(np.pi),            0,              0],
                           [                   0,                   0,            1,              0],
                           [                   0,                   0,            0,              1]])

            # 90 degrees on the Y axis
            R_y = Matrix([[        cos(-np.pi/2),                   0,sin(-np.pi/2),              0],
                          [                   0,                   1,            0,              0],
                           [      -sin(-np.pi/2),                   0,cos(-np.pi/2),              0],
                           [                   0,                   0,            0,              1]])

            R_corr = simplify(R_z * R_y)

            T_final = simplify(T0_7 * R_corr)

            #Getting Transformation from base orgin

            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll_, pitch_, yaw_) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            P=Matrix([[px],
                     [py],
                    [pz],
                     [0]])
            D=Matrix([[d7],
                     [0],
                     [0],
                     [0]])

            Ryaw=Matrix([[cos(yaw),-sin(yaw),0,0],
                      [sin(yaw),cos(yaw),0,0],
                      [0,0,1,0],
                      [0,0,0,1]])

            Rpitch=Matrix([[cos(pitch),0,sin(pitch),0],
                      [0,1,0,0],
                      [-sin(pitch),0,cos(pitch),0],
                      [0,0,0,1]])

            Rroll=Matrix([[1,0,0,0],
                      [0,cos(roll),-sin(roll),0],
                      [0,sin(roll),cos(roll),0],
                      [0,0,0,1]])

            R0_6 = Ryaw * Rpitch * Rroll
            W = P - R0_6 * D
            W = W.subs(S)
            W_x = W[0]
            W_y = W[1]
            W_z = W[2]

            # Calculate joint angles using Geometric IK method
            theta1 = atan2(wy, wx) # Simple trig, using x,y coords

            # Find 'r' of origin(O) to wc
            r = sqrt(wx**2 + wy**2)

            # Calculate dist J2 - wc (c) and angle from horizontal to wc (C)
            a = r - a1
            b = wz - d1
            c = sqrt(a**2 + b**2)

            # Use cosine rule to find theta3 and allow for offset
            costheta3 = (a2**2 + d4**2 - c**2)/(2 * a2 * d4)
            theta3 = atan2(costheta3, sqrt(1 - costheta3**2))

            # Find elevation angle from J2 to J5
            theta21 = atan2(b, a)
            # Then angle between a2 and c (length J2 - J5)
            # And hence, total of that angle
            theta2 = theta21 + atan2(a2 + d4*cos(theta3), d4*sin(theta3))

            ## Find thetas for q4, q5 & q6
            ## Solve for T0_3
            T0_3mtrx = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            ## Use T0_3 with T0_6 from request data to find T3_6
            T3_6mtrx = inv(T0_3mtrx) * T0_6mtrx * R_corr

            ## Assign matrix elements to variables
            r11 = T3_6mtrx[0,0]
            r12 = T3_6mtrx[0,1]
            r13 = T3_6mtrx[0,2]
            r21 = T3_6mtrx[1,0]
            r31 = T3_6mtrx[2,0]
            r32 = T3_6mtrx[2,1]
            r33 = T3_6mtrx[2,2]

            theta5 = atan2(-r31, sqrt(r11*r11 + r21*r21))

            # Check for lock within the wrist,
            # pitch at -90
            if r31 == 1:
                theta4 = 0.0
                theta6 = atan2(-r12, -r13)

            # pitch at 90
            elif r31 == -1:
                theta4 = 0.0
                theta6 = atan2(r12, r13)
            else:
                theta4 = atan2(r21, r11)
                theta6 = atan2(r32, r33)

            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)

def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
