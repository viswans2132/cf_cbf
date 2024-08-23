 #!/usr/bin/env python
 # license removed for brevity
import rospy
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header, Int8
from cf_cbf.msg import PosVelMsg, ConstraintMsg, DroneParamsMsg, Int8Array
import time
import numpy as np
import cvxpy
import sys
from cf_cbf.drone_parameters import DroneParameters
from cf_cbf.ugv_lib import UGV

def dist(x_):
    return np.linalg.norm(x_)

def sq_dist(x_, y_):
    return np.sum(np.square(x_) / np.square(y_))


class DroneController:
    droneOdomSub = []
    ugvOdomSub = []
    droneParamSub = []
    droneRefPub = []
    droneConsPub = []
    droneModePub = []
    ugvCmdPub = []
    def __init__(self, name):
        self.name = name
        self.filterFlag = False

        self.t = rospy.get_time()
        self.rate = rospy.Rate(1)

        self.drones = [DroneParameters('cf8'), DroneParameters('demo_crazyflie1')]
        # self.drones = [DroneParameters('demo_crazyflie1')]
        self.ugvs_1 = UGV('demo_turtle2') 
        self.ugvs_2 = UGV('demo_turtle1')

        # for ugv in self.ugvs:
        #     self.ugvOdomSub.append(rospy.Subscriber('/vicon/{}/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
        #     self.ugvCmdPub.append(rospy.Publisher('/{}/cmd_vel'.format(ugv.name), Twist, queue_size=10))

        self.ugvOdomSub_1 = rospy.Subscriber('/vicon/{}/{}/odom'.format(self.ugvs_1.name, self.ugvs_1.name), Odometry, self.ugvs_1.odom_cb)
        self.ugvOdomSub_2 = rospy.Subscriber('/vicon/{}/{}/odom'.format(self.ugvs_2.name, self.ugvs_2.name), Odometry, self.ugvs_2.odom_cb)


        print('Sleeping')
        time.sleep(1)

        while not rospy.is_shutdown():
            self.loop()


    def loop(self):
        i = 0

        print(self.ugvs_1.name)
        print("{}: {:.3f}, {:.3f}, {:.3f}".format(self.ugvs_1.name, self.ugvs_1.pos[0], self.ugvs_1.pos[1], self.ugvs_1.pos[2]))

        self.rate.sleep()
        print(self.ugvs_2.name)
        print("{}: {:.3f}, {:.3f}, {:.3f}".format(self.ugvs_2.name, self.ugvs_2.pos[0], self.ugvs_2.pos[1], self.ugvs_2.pos[2]))
        # for droneI, ugvI in zip(self.drones, self.ugvs):
        #     sqHorDist = sq_dist(ugvI.pos[:2] - droneI.pos[:2], np.ones((2,)))
        #     ugvErrPos = droneI.pos - ugvI.pos
        #     # print("{:.3f}, {:.3f}, {:.3f}".format(droneI.pos[0], droneI.pos[1], droneI.pos[2]))
        #     # print("{:.3f}, {:.3f}, {:.3f}".format(ugvErrPos[0], ugvErrPos[1], ugvErrPos[2]))
                    
        #     i = i + 1
        self.rate.sleep()



if __name__ == '__main__':
     try:
        rospy.init_node('crazyflie_controller', anonymous=True)
        dc = DroneController('controller')
     except rospy.ROSInterruptException:
        pass
