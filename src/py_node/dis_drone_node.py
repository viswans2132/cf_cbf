 #!/usr/bin/env python
 # license removed for brevity
import rospy
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header, Int8
from cf_cbf.msg import PosVelMsg
import time
import numpy as np
import sys
from cf_cbf.drone_lib import Drone

def dist(x_):
    return np.linalg.norm(x_)

def sq_dist(x_, y_):
    return np.sum(np.square(x_) / np.square(y_))


class DroneController:
    def __init__(self, name):
        self.name = name

        self.drone = Drone(name)
        self.rate = rospy.Rate(1)
        self.modeSub = rospy.Subscriber('/{}/uav_mode'.format(self.drone.name), Int8, self.setMode)

        self.droneOdomSub = rospy.Subscriber('/vicon/{}/{}/odom'.format(self.drone.name, self.drone.name), Odometry, self.odom_cb)
        self.droneRefSub = rospy.Subscriber('/{}/ref'.format(self.drone.name), PosVelMsg, self.ref_cb)
        self.droneCmdPub = rospy.Publisher('/{}/cmd_vel'.format(self.drone.name), Twist, queue_size=10)

        self.cmdVelMsg = Twist()
        self.cmdArray = np.array([0,0,0,0.0])

        while not rospy.is_shutdown():
            self.loop()


    def loop(self):
        self.drone.generateControlInputs(self.cmdArray)
        self.cmdVelMsg.linear.x = self.cmdArray[0]
        self.cmdVelMsg.linear.y = self.cmdArray[1]
        self.cmdVelMsg.linear.z = self.cmdArray[3]
        self.cmdVelMsg.angular.z = self.cmdArray[2]
        self.droneCmdPub.publish(self.cmdVelMsg)
        self.rate.sleep()

    def setMode(self, msg):
        self.drone.setMode(msg.data)
        self.drone.setMode(msg.data)

    def land_cb(self, data):
        self.land_flag = True
        print('Safety Landing: Active')

    def ref_cb(self, msg):
        pose = np.array([msg.position[0], msg.position[1], msg.position[2], msg.yaw])
        vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.yawVelocity])
        self.drone.setRef(pose, vel)

    def odom_cb(self, msg):
        position = np.array([msg.pose.pose.positon.x, msg.pose.pose.positon.y, msg.pose.pose.positon.z])
        quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.yawVelocity])
        self.drone.setRef(pose, vel)



if __name__ == '__main__':
     try:
        rospy.init_node('crazyflie_controller', anonymous=True)
        uav_name = rospy.get_param('~uav_name')
        dc = DroneController(uav_name)
     except rospy.ROSInterruptException:
        pass
