 #!/usr/bin/env python
 # license removed for brevity
import rospy
# import opengen as og
# import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
import time 
from tf.transformations import euler_from_quaternion, quaternion_matrix
# from quaternion_to_euler import quaternion_to_euler
# from trajectory import trajectory
# from predict import predict
# from traj_msg.msg import OptimizationResult
# from adapt_weights import adapt_weights
#from event_trigger import event_trigger
# from lin_pred import lin_pred
# from proj_pred import proj_pred
import math
import time
import numpy
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
land_flag = 0
start_flag = 0
odom_flag = False

def callback_odom(data):
    global xpos, ypos, zpos, vx, vy, vz, roll, pitch, yaw, d_yaw, odom_flag
    xpos = data.pose.pose.position.x
    ypos = data.pose.pose.position.y
    zpos = data.pose.pose.position.z
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w
    d_yaw = data.twist.twist.angular.z
    [roll, pitch, yaw] = euler_from_quaternion([qx, qy, qz, qw])

    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    vz = data.twist.twist.linear.z
    odom_flag = True
    # print('odom')

def callback_ref(data):
    global xref, yref, zref, yawref
    xref = data.pose.position.x
    yref = data.pose.position.y
    zref = data.pose.position.z
    yawref = data.pose.orientation.z

def callback_safety(data):
    global land_flag
    land_flag = 1

def callback_start(data):
    global start_flag, odom_flag
    if odom_flag == False:
        print("Sorry bro. I dont have a position feedback. Please fix it.")
    else:
        start_flag = 1

def controller():
    rospy.init_node('pid_controller', anonymous=True)
    pub = rospy.Publisher('/dcf6/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/vicon/dcf6/dcf6/odom', Odometry, callback_odom)
    sub_start = rospy.Subscriber('/set_start', String, callback_start)
    sub_safety = rospy.Subscriber('/safety_land', String, callback_safety)
    sub_ref = rospy.Subscriber('/reference', PoseStamped, callback_ref)

    rate = rospy.Rate(30)
    global xref, yref, zref, integrator, land_flag, yawref

    xref = -0.28
    yref = 4.00
    zref = 0.8
    yawref = 0

    to_thrust = 0.63

    t = 0
    integrator = 0

    k_px = 1.5
    k_py = 1.5
    k_pz = 0.5
    k_vx = 0.5
    k_vy = 0.5
    k_vz = 0.5

    k_y = 1
    k_dy = 0.2
    print("Values initialized")
    
    while not rospy.is_shutdown():
        if(start_flag == 1):
            integrator = integrator + 0.00*(zref-zpos)
            ang_diff = numpy.mod(yawref - yaw + math.pi, 2*math.pi) - math.pi

            vx_ref = k_px*(xref - xpos)

            fx = k_vx*(vx_ref - vx)
            fy = k_vy*(k_py*(yref - ypos) - vy)
            u_p = math.cos(yaw)*fx + math.sin(yaw)*fy - 0.0
            u_r = -math.sin(yaw)*fx + math.cos(yaw)*fy
            u_t = k_vy*(k_pz*(zref - zpos) - vz) + to_thrust + integrator
            u_y = k_y*ang_diff - k_dy*d_yaw

            print("Err: {:.3f}, {:.3f}, {:.3f},\n {:.3f}, {:.3f}, {:.3f}".format(xref - xpos, yref - ypos, zref - zpos, fx, fy, u_t))

            if u_p > 0.25:
                u_p = 0.25
            if u_p < -0.25:
                u_p = -0.25
            if u_r > 0.25:
                u_r = 0.25
            if u_r < -0.25:
                u_r = -0.25
            if u_t > 1.0:
                u_t = 1.0
            if u_t < 0.0:
                u_t = 0.0

            if land_flag == 1:
                u_p = 0
                u_r = 0
                u_y = 0
                u_t = 0.6 - 0.005*t
                t = t+1
                if t > 50:
                    u_t = 0
                if t > 52:
                    u_t = 0
                    sys.exit()

        
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
        # print("{:.3f}, {:.3f}, {:.3f}, {:.3f}".format(u_p, u_r, u_y, u_t))

        
        pub.publish(cmd_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
