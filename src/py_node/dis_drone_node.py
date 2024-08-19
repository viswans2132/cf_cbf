 #!/usr/bin/env python
 # license removed for brevity
import rospy
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header, Int8
from cf_cbf.msg import PosVelMsg, ConstraintMsg, DroneParamsMsg
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
        self.rate = rospy.Rate(30)
        self.followSub = rospy.Subscriber('/{}/uav_mode'.format(self.drone.name), Int8, self.setMode)
        # self.landingSub = rospy.Subscriber('/{}/land'.format(self.drone.name), Bool, self.land_cb)

        self.droneOdomSub = rospy.Subscriber('/vicon/{}/{}/odom'.format(self.drone.name, self.drone.name), Odometry, self.odom_cb)
        self.droneRefSub = rospy.Subscriber('/{}/ref'.format(self.drone.name), PosVelMsg, self.ref_cb)
        self.droneConsSub = rospy.Subscriber('/{}/cons'.format(self.drone.name), ConstraintsMsg, self.cons_cb)
        self.droneCmdPub = rospy.Publisher('/{}/cmd_vel'.format(self.drone.name), Twist, queue_size=10)
        self.droneParamPub = rospy.Publisher('/{}/param'.format(self.drone.name), DroneParamsMsg, queue_size=10)

        self.cmdVelMsg = Twist()
        self.cmdArray = np.array([0,0,0,0.0])

        time.sleep(1)
        print('Node {}: Awake'.format(self.name))

        paramMsg = DroneParamsMsg()
        paramMsg.kRad = self.drone.kRad
        paramMsg.omegaD = self.drone.omegaD
        self.droneParamPub.publish(paramMsg)
        self.rate.sleep()

        while not rospy.is_shutdown():
            self.loop()


    def loop(self):
        odomReceived = self.drone.generateControlInputs(self.cmdArray)
        if odomReceived:
            self.cmdVelMsg.linear.x = self.cmdArray[0]
            self.cmdVelMsg.linear.y = self.cmdArray[1]
            self.cmdVelMsg.linear.z = self.cmdArray[3]
            self.cmdVelMsg.angular.z = self.cmdArray[2]
            self.droneCmdPub.publish(self.cmdVelMsg)
            self.rate.sleep()

    def setMode(self, msg):
        self.drone.setMode(msg.data)

    def cons_cb(self, msg):
        matrix = np.array(msg.constraints).reshape((4,-1))
        self.drone.updateConstraintMatrices(matrix(:3,:), matrix(3,:))

    def land_cb(self, data):
        self.drone.landFlag = True
        print('Safety Landing: Active')

    def follow_cb(self, data):
        self.drone.followFlag = True
        print('Trajectory: Active')

    def start_cb(self, data):
        self.drone.startFlag = True
        print('Take off: Active')

    def ref_cb(self, msg):
        pose = np.array([msg.position[0], msg.position[1], msg.position[2], msg.yaw])
        vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.yawVelocity])
        self.drone.setRef(pose, vel)

    def odom_cb(self, msg):
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        velocity = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.z])
        self.drone.setOdom(position, quat, velocity)



if __name__ == '__main__':
     try:
        rospy.init_node('crazyflie_controller', anonymous=True)
        uav_name = rospy.get_param('~uav_name')
        dc = DroneController(uav_name)
     except rospy.ROSInterruptException:
        pass
