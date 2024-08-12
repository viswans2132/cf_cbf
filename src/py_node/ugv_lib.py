 #!/usr/bin/env python
 # license removed for brevity
import rospy
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header
from trajectory import trajectory
import time
import numpy as np



class UGV:
    name = 'ugv'
    
    pos = np.array([0.0, 0, 0])
    quat = np.array([0.0, 0, 0, 1])

    vel = np.array([0, 0.0, 0])
    ang_vel = np.array([0.0, 0, 0])

    odom_status = False

    def __init__(self, name):
        self.name = name

        self.hz = 30.0
        self.dt = 1/self.hz
        self.control_input = np.array([0.0, 0.0, 0.0])
        self.cmdVel = Twist()
        self.ref = PoseStamped()

        self.rate = rospy.Rate(self.hz)

        # self.cmd_pub = rospy.Publisher('/{}/cmd_vel'.format(name), TwistStamped, queue_size=1)
        # self.ref_pub = rospy.Publisher('/{}/ref'.format(name), PoseStamped, queue_size=1)

        # self.odom_sub = rospy.Subscriber('/vicon/{}/{}/odom'.format(name, name), Odometry, self.odom_cb)
        # self.cmd_sub = rospy.Subscriber('/old_cmd_vel', TwistStamped, self.oldControl_cb)

        self.kRate = 4.0
        self.kScaleD = 1.2
        self.KOffset = 0.05
        self.omegaD = 1.0
        
        radius = 0.4
        self.kHeight = 1.0
        self.kScaleA = self.kHeight/(radius*radius)
        self.omegaA = 1.0





    def odom_cb(self, data):
        self.pos[0] = float(data.pose.pose.position.x)
        self.pos[1] = float(data.pose.pose.position.y)
        self.pos[2] = float(data.pose.pose.position.z)
        self.quat[0] = float(data.pose.pose.orientation.x)
        self.quat[1] = float(data.pose.pose.orientation.y)
        self.quat[2] = float(data.pose.pose.orientation.z)
        self.quat[3] = float(data.pose.pose.orientation.w)
        self.vel[0] = float(data.twist.twist.linear.x)
        self.vel[1] = float(data.twist.twist.linear.y)
        self.vel[2] = float(data.twist.twist.linear.z)
        self.odom_status = True

    def setStop(self, data):
        self.stop_flag = True

    def setStart(self, data):
        self.start_flag = True
    
    def setFollow(self, data):
        self.follow_flag = True

    def publishCmdVel(self, data):
        self.cmd_pub.publish(data)
    
    def odomStatus(self):
        return self.odom_status