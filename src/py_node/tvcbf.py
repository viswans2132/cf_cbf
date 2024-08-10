 #!/usr/bin/env python
 # license removed for brevity
import rospy
import opengen as og
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
import time 
from quaternion_to_euler import quaternion_to_euler
from trajectory import trajectory
# from predict import predict
# from traj_msg.msg import OptimizationResult
from adapt_weights import adapt_weights
#from event_trigger import event_trigger
# from lin_pred import lin_pred
from proj_pred import proj_pred
import math
import time
import numpy as np
from qpsolvers import solve_qp
import csv
import sys
import os
#import subprocess
xpos = 0
ypos = 4
zpos = 0
xref = 0
yref = 4
zref = 0.6
yaw = 0
yawref = 0
d_yaw = 0
roll = 0
pitch = 0
qx = 0
qy = 0
qz = 0
qw = 0
vx = 0
vy = 0
vz = 0
vxref = 0
vyref = 0
vzref = 0
land_flag = 0
start_flag = 0
filter_flag = 0
ref_flag = 0

def callback_odom(data):
    global xpos, ypos, zpos, vx, vy, vz, roll, pitch, yaw, d_yaw
    xpos = data.pose.pose.position.x
    ypos = data.pose.pose.position.y
    zpos = data.pose.pose.position.z
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w
    d_yaw = data.twist.twist.angular.z
    [roll, pitch, yaw] = quaternion_to_euler(qx, qy, qz, qw)

    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    vz = data.twist.twist.linear.z
    # print(xpos)

def callback_ref(data):
    global xref, yref, zref, vxref, vyref, vzref, yawref, ref_flag
    xref = data.pose.position.x
    yref = data.pose.position.y
    zref = data.pose.position.z
    vxref = data.pose.orientation.x
    vyref = data.pose.orientation.y
    vzref = data.pose.orientation.z
    yawref = data.pose.orientation.z
    ref_flag = 1

def callback_safety(data):
    global land_flag
    land_flag = 1
    print('Landing')

def callback_start(data):
    global start_flag, land_flag, filter_flag, ref_flag
    if data.data == '1':
        time.sleep(0.1)
        print("UAV is ON")
        # land_flag = 0
        if land_flag == 0:
            start_flag = 1
            print("UAV is ON")
    if data.data == '2':
        time.sleep(0.1)
        filter_flag = 1
        ref_flag = 0
        print("Filter is ON")

def controller():
    rospy.init_node('tvcbf_controller', anonymous=True)
    pub = rospy.Publisher('/demo_crazyflie1/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/vicon/demo_crazyflie1/demo_crazyflie1/odom', Odometry, callback_odom)
    sub_start = rospy.Subscriber('/set_start', String, callback_start)
    sub_safety = rospy.Subscriber('/safety_land', String, callback_safety)
    sub_ref = rospy.Subscriber('/reference', PoseStamped, callback_ref)

    rate = rospy.Rate(30)
    global xref, yref, zref, vxref, vyref, vzref, integrator, start_flag, land_flag, yawref, ref_flag, filter_flag

    xref = 1.9
    yref = -0.250
    zref = 0.8
    yawref = 0

    to_thrust = 0.63

    t = 0
    integrator = 0

    k_px = 1.2
    k_py = 1.2
    k_pz = 0.8
    k_vx = 0.5
    k_vy = 0.5
    k_vz = 0.5

    k_y = 1
    k_dy = 0.2

    ezp = 0.5
    theta = 25

    kh_rate = 1/(ezp*np.tan(np.deg2rad(theta)))
    kh_scale = np.exp(1)*ezp

    dhdt_scale = kh_scale*kh_rate


    omega = 0.71
    print(kh_rate, kh_scale)
    # print(x)
    kh_offset = -0.03

    p_err_int = np.array([0.0, 0.0, 0.0])

    u_t_land = 0.5
    
    while not rospy.is_shutdown():
        if(start_flag == 1):
            # xpos_body = math.cos(yaw)*xpos + math.sin(yaw)*ypos
            # ypos_body = -math.sin(yaw)*xpos + math.cos(yaw)*ypos
            # xref_body = math.cos(yaw)*xref + math.sin(yaw)*yref
            # yref_body = -math.sin(yaw)*xref + math.cos(yaw)*yref
            if land_flag == 1:
                start_flag = 0
                u_p = 0
                u_r = 0
                u_y = 0
                # print(u_t)
                u_t = 0.5 - 0.08*t
                t = t+1
                if t > 20:
                    u_t = 0
                    cmd_vel = Twist()
                    cmd_vel.linear.x = u_p
                    cmd_vel.linear.y = u_r
                    cmd_vel.linear.z = u_t
                    cmd_vel.angular.z = u_y
                    pub.publish(cmd_vel)
                    rate.sleep()
                    sys.exit()


            else:
                x_err = xpos - xref
                y_err = ypos - yref
                z_err = zpos - zref

                # print("{:.3f} : {:.3f} : {:.3f}".format(x_err, y_err, z_err))

                

                psuedo_vx = -k_px*(x_err)
                psuedo_vy = -k_py*(y_err) 
                psuedo_vz = -k_pz*(z_err)

                if filter_flag == 1 & ref_flag == 1:
                    e_norm = np.linalg.norm(np.array([x_err, y_err]))
                    if e_norm < 0.25:
                        p_err_int =  p_err_int + 0.04*np.array([x_err, y_err, z_err])
                        psuedo_vx = psuedo_vx - 0.6*p_err_int[0] + vxref
                        psuedo_vy = psuedo_vy - 0.6*p_err_int[1] + vyref
                        psuedo_vz = psuedo_vz - 0.0*p_err_int[2]
                        # print("Integrator is ON")
                        # print([x_err,y_err,z_err])
                        # print(e_norm)

                    if  ((np.absolute(x_err) < 0.01) and (np.absolute(y_err) < 0.01) and (np.absolute(z_err) < 0.06)):
                        # print([x_err,y_err,z_err])
                        callback_safety(1)

                    x2y2_sqrt = np.sqrt(x_err*x_err + y_err*y_err)

                    h1 = z_err - dhdt_scale*x2y2_sqrt*np.exp(-kh_rate*x2y2_sqrt) - kh_offset
                    # print(z_err)
                    # if z_err>0.3:

                    P = np.array([[2.0, 0, 0], [0.0, 2, 0], [0, 0, 2]])
                    q = -2*(np.array([psuedo_vx, psuedo_vy, psuedo_vz]))
                    g1 = kh_scale*kh_rate*x_err*x2y2_sqrt*np.exp(-kh_rate*x2y2_sqrt)*(kh_rate*x2y2_sqrt - 1)/x2y2_sqrt
                    g2 = kh_scale*kh_rate*y_err*x2y2_sqrt*np.exp(-kh_rate*x2y2_sqrt)*(kh_rate*x2y2_sqrt - 1)/x2y2_sqrt
                    g3 = 1
                    G = -np.array([[g1, g2, g3]])
                    # A = 0.0*np.eye(3)
                    A = np.empty((3,3))
                    # b = np.array([0.0, 0.0, 0.0])
                    b = np.empty((3,1))
                    lb = np.array([-0.2, -0.2, -0.1])
                    ub = np.array([0.2, 0.2, 0.5])

                    dot_x2y2_sqrt = -dhdt_scale*(1-kh_rate*x2y2_sqrt)*np.exp(-kh_rate*x2y2_sqrt)*(vxref*x_err + vyref*y_err)/x2y2_sqrt
                    if z_err > 0.4:
                        print("{:.3f}".format(z_err))
                    if z_err>0.4 and omega > 0.7:
                        dhdt_scale = kh_scale
                        omega = 0.7
                        to_thrust = 0.6
                        print('switch')
                        print([x_err, y_err, z_err])
                    # print("{:.3f}".format(dot_x2y2_sqrt))

                    h = np.array([omega*h1]) + dot_x2y2_sqrt

                    try:
                        vel_input = solve_qp(P, q, G, h, None, None, None, None, solver='cvxopt')
                        psuedo_vx = vel_input[0]
                        psuedo_vy = vel_input[1]
                        psuedo_vz = vel_input[2]
                    except (TypeError, ValueError) as e:
                        print("Value Error")

                    if np.linalg.norm(vel_input) > 0.5:
                        vel_input = 0.5*vel_input/np.linalg.norm(vel_input)
                    vel_input = np.maximum(np.minimum(vel_input, np.array([0.2, 0.2, 0.5])), np.array([-0.2, -0.2, -0.5]))



                integrator = integrator + 0.0*(psuedo_vz)
                ang_diff = np.mod(yawref - yaw + math.pi, 2*math.pi) - math.pi

                # fx = k_vx*(k_px*(xref - xpos) - vx)
                fx = -k_vx*(vx - psuedo_vx)
                # fy = k_vy*(k_py*(yref - ypos) - vy)
                fy = -k_vy*(vy - psuedo_vy)
                u_p = math.cos(yaw)*fx + math.sin(yaw)*fy
                u_r = -math.sin(yaw)*fx + math.cos(yaw)*fy
                # u_t = k_vy*(k_pz*(zref - zpos) - vz) + to_thrust + integrator
                u_t = -k_vz*(vz-psuedo_vz) + to_thrust + integrator
                u_y = k_y*ang_diff - k_dy*d_yaw

                if u_p > 0.25:
                    u_p = 0.25
                if u_p < -0.25:
                    u_p = -0.25
                if u_r > 0.25:
                    u_r = 0.25
                if u_r < -0.25:
                    u_r = -0.25
                if u_t > 0.8:
                    u_t = 0.8
                if u_t < 0.0:
                    u_t = 0.0
                
                u_t_land = u_t - 0.1

            
        
        else:
            u_p = 0
            u_r = 0
            u_t = 0
            u_y = 0

        cmd_vel = Twist()
        cmd_vel.linear.x = u_p
        cmd_vel.linear.y = u_r
        cmd_vel.linear.z = u_t
        cmd_vel.angular.z = u_y
        pub.publish(cmd_vel)
        # print(u_p, u_r, u_t, u_y)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
