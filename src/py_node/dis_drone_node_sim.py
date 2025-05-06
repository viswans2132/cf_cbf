 #!/usr/bin/env python
 
 # Developed by Viswa Narayanan Sankaranarayanan

import rospy
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Header, Int8
from cf_cbf.msg import DronePosVelMsg, DroneConstraintMsg, DroneParamsMsg
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
        self.followSub = rospy.Subscriber('/{}/update_uav_mode'.format(self.drone.name), Int8, self.setMode)

        self.droneOdomSub = rospy.Subscriber('/{}/odometry_sensor1/odometry'.format(self.drone.name, self.drone.name), Odometry, self.odom_cb)
        self.droneRefSub = rospy.Subscriber('/{}/ref'.format(self.drone.name), DronePosVelMsg, self.ref_cb)
        self.droneConsSub = rospy.Subscriber('/{}/cons'.format(self.drone.name), DroneConstraintMsg, self.cons_cb)
        self.droneCmdPub = rospy.Publisher('/{}/vel_msg'.format(self.drone.name), TwistStamped, queue_size=10)
        self.droneParamSub = rospy.Subscriber('/{}/params'.format(self.drone.name), DroneParamsMsg, self.params_cb)
        self.droneUpdateParamPub = rospy.Publisher('/{}/update_params'.format(self.drone.name), DroneParamsMsg, queue_size=10)
        self.landSignal = rospy.Publisher('/{}/land_signal'.format(self.drone.name), Int8, queue_size=10)
        self.droneModePub = rospy.Publisher('/{}/uav_mode'.format(self.drone.name), Int8, queue_size=10)

        self.cmdVelMsg = TwistStamped()
        self.cmdArray = np.array([0,0,0,0.0])

        # time.sleep(1)
        print('Node {}: Awake'.format(self.name))

        self.drone.kRad = np.array([0.5, 0.5, 1.0])
        self.drone.kPos = np.array([-4.5, -4.5, -0.7])



        self.timer = rospy.get_time()

        while not rospy.is_shutdown():
            self.loop()


    def loop(self):
        odomReceived = self.drone.generateVelocityInputs(self.cmdArray)
        # rospy.loginfo(f"[self.name]_dis_node: Error Flag: {self.drone.errorFlag}")
        if odomReceived and not self.drone.errorFlag:
            # rospy.loginfo(f'[{self.name}_dis_node]: Command Velocity {self.cmdArray}')
            self.cmdVelMsg.twist.linear.x = self.cmdArray[0]
            self.cmdVelMsg.twist.linear.y = self.cmdArray[1]
            self.cmdVelMsg.twist.linear.z = self.cmdArray[2]

            if self.drone.landFlag:
                landMsg = Int8()
                landMsg.data = 1
                self.landSignal.publish(landMsg)
                # print('{}: land_signal'.format(self.drone.name))

            else:
                self.droneCmdPub.publish(self.cmdVelMsg)

            if not self.drone.paramFlag:
                rospy.loginfo(f'[{self.name}_dis_node]: Value of parameter flag: {self.drone.paramFlag}')
                paramMsg = DroneParamsMsg()
                paramMsg.kRad = self.drone.kRad
                paramMsg.omegaC = self.drone.omegaC
                self.droneUpdateParamPub.publish(paramMsg)

        if rospy.get_time() - self.timer > 2:
            # print('Odometry Not received for {}'.format(self.drone.name))
            rospy.loginfo(f"[{self.name}]_dis_node: Odometry not received for {self.drone.name}.")
            self.drone.errorFlag = True
        self.rate.sleep()

    def setMode(self, msg):
        self.drone.setMode(msg.data)
        modeMsg = Int8()
        modeMsg.data = self.drone.droneMode
        self.droneModePub.publish(modeMsg)

    def cons_cb(self, msg):
        matrix = np.array(msg.constraints).reshape((-1,4))
        # print('Matrix: {}'.format(matrix))
        self.drone.updateConstraintMatrices(matrix[:,:3], matrix[:,3])

    def land_cb(self, data):
        self.drone.landFlag = True
        print('Safety Landing: Active')

    def follow_cb(self, data):
        self.drone.followFlag = True
        print('Trajectory: Active')

    def start_cb(self, data):
        self.drone.startFlag = True
        print('Take off: Active')

    def params_cb(self, data):
        self.drone.paramFlag = True
        rospy.loginfo(f"[{self.name}_dis_node]: Parameter Update Acknowledged.")

    def ref_cb(self, msg):
        # print(msg.position)
        try:
            pose = np.array([msg.position[0], msg.position[1], msg.position[2], msg.yaw])
            vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.yawVelocity])
            self.drone.setRef(pose, vel)
        except IndexError:
            print('Ref msg empty: {}'.format(msg.position))

    def odom_cb(self, msg):
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        velocity = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.z])
        self.drone.setOdom(position, quat, velocity)
        self.timer = rospy.get_time()



if __name__ == '__main__':
     try:
        rospy.init_node('drone_controller', anonymous=True)
        uav_name = rospy.get_param('~uav_name')
        dc = DroneController(uav_name)
     except rospy.ROSInterruptException:
        pass
