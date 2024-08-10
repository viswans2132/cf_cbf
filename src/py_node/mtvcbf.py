 #!/usr/bin/env python
 # license removed for brevity
import rospy
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header, Int8
from rrc_control.msg import PosVelMsg
from quaternion_to_euler import quaternion_to_euler
from trajectory import trajectory
import time
import numpy as np
import cvxpy
import sys
from drone_lib import Drone
from ugv_lib import UGV

def dist(x_):
    return np.linalg.norm(x_)

def sq_dist(x_, y_):
    return np.sum(np.square(x_) / np.square(y_))


class DroneController:
    droneOdomSub = []
    ugvOdomSub = []
    droneRefSub = []
    droneCmdPub = []
    ugvCmdPub = []
    def __init__(self, name):
        self.name = name

        self.drones = [Drone('cf8'), Drone('dcf2')]
        self.ugvs = [UGV('turtle1'), UGV('turtle2')]
        self.lenDrones = len(self.drones)
        self.rate = rospy.Rate(30)
        self.modeSub = rospy.Subscriber('/uav_mode', Int8, self.setMode)

        for drone in self.drones:
            self.droneOdomSub.append(rospy.Subscriber('/vicon/{}/{}/odom'.format(drone.name, drone.name), Odometry, drone.odom_cb))
            self.droneRefSub.append(rospy.Subscriber('/{}/ref'.format(drone.name), PosVelMsg, drone.ref_cb))
            self.droneCmdPub.append(rospy.Publisher('/{}/cmd_vel'.format(drone.name), Twist, queue_size=10))


        for ugv in self.ugvs:
            self.ugvOdomSub.append(rospy.Subscriber('/vicon/{}/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
            self.ugvCmdPub.append(rospy.Publisher('/{}/cmd_vel'.format(ugv.name), Twist, queue_size=10))


        self.filterFlag = False

        while not rospy.is_shutdown():
            self.loop()


    def loop(self):
        A_ = [np.zeros((1,3))]*self.lenDrones
        b_ = [0]*self.lenDrones
        i = 0
        for droneI, ugvI in zip(self.drones, self.ugvs):
            if self.filterFlag:
                sqHorDist = sq_dist(ugvI.pos[:2] - droneI.pos[:2], np.ones((3,)))
                ugvErrPos = droneI.pos - self.ugvs[i].pos
                h = ugvErrPos[2] - ugvI.kRate*ugvI.kScaleD*sqHorDist*(np.exp(-kRate*sqHorDist)) - droneI.kOff
                A_[i][0,0] = 2*ugvI.kRate*ugvI.kScaleD*(ugvI.kRate - 1)*np.exp(-ugvI.kRate*sqHorDist)*ugvErrPos[0]
                A_[i][0,1] = 2*ugvI.kRate*ugvI.kScaleD*(ugvI.kRate - 1)*np.exp(-ugvI.kRate*sqHorDist)*ugvErrPos[1]
                A_[i][0,2] = 1
                b[i] = - ugvI.omegaD*h - 2*ugvI.kRate*ugvI.kScaleD*(1 - ugvI.kRate)*np.exp(-ugvI.kRate*sqHorDist)*(ugvErrPos[0]*ugvI.vel[0] + ugvErrPos[1]*ugvI.vel[1])

                j = i+1

                for droneJ in self.drones[j:]:
                    droErrPos = droneI.pos - droneJ.pos
                    if dist(drErrPos) < 1.5:
                        h = sq_dist(drErrPos, droneI.kRad) - 0.25
                        scaled_disp = 2*droErrPos/droneI.kRad
                        A_[i] = np.vstack((A_[i], scaled_disp))
                        A_[j] = np.vstack((A_[j], -scaled_disp))
                        b_[i] = np.vstack((b[i], -droneI.omegaC*h - scaled_disp@droneJ.vel))
                        b_[j] = np.vstack((b[j], droneJ.omegaC*h + scaled_disp@droneI.vel))
                        j = j+1

                for ugvK in self.ugvs:
                    if ugvK != ugvI:
                        ugvErrPos = droneI.pos - droneJ.pos
                        if dist(ugvErrPos) < 1.5:
                            sqHorDist = sq_dist(ugvK.pos[:2] - droneI.pos[:2], ugvK.kRad[:2])
                            h = ugvErrPos[2] - ugvK.kHeight + ugvK.kScaleA*sqHorDist
                            A_[i] = np.vstack((A_[i], np.array([2*ugvK.kScaleA*ugvErrPos[0], 2*ugvK.kScaleA*ugvErrPos[1], 1])))
                            b_[i] = np.vstack((b_[i], -ugvK.omegaA*h + 2*ugvK.kScaleA*(ugvErrPos[0]*ugvK.vel[0] + ugvErrPos[1]*ugvK.vel[1])))

                droneI.updateConstraintMatrices(A_[i], b[i])

            cmdVel = Twist()
            droneI.generateControlInputs(cmdVel)
            self.droneCmdPub[i].publish(cmdVel)
            print('Publishing: {}'.format(droneI.name))
            self.rate.sleep()
            i = i + 1

    def setMode(self, msg):
        self.drones[0].setMode(msg.data)
        self.drones[1].setMode(msg.data)

        if msg.data == 1:
            self.filterFlag = True
            self.drones[0].desPos[1] = self.drones[0].desPos[1] - 1.5
            self.drones[1].desPos[1] = self.drones[1].desPos[1] + 1.5

    def land_cb(self, data):
        self.land_flag = True
        print('Safety Landing: Active')

    def path(self):
        pos_ref_ = self.pos_off + np.array([0.0, 0.0, 0.8])
        for i in range(0,self.N):
            self.pos_ref[0 + i*8] = pos_ref_[0] + 1.0 * math.cos(2 * math.pi * (self.t+i)/600)
            self.pos_ref[1 + i*8] = pos_ref_[1] + 1.0 * math.sin(2 * math.pi * (self.t+i)/600)
            self.pos_ref[2 + i*8] = 0.8    
        self.droneI.ref.pose.position.x = self.pos_ref[0]
        self.droneI.ref.pose.position.y = self.pos_ref[1]
        self.droneI.ref.pose.position.z = self.pos_ref[2]
        self.droneI.ref.header.stamp = rospy.Time.now()




    def obs_cb(self, data):
        self.obs_pos[0] = float(data.pose.pose.position.x)
        self.obs_pos[1] = float(data.pose.pose.position.y)
        self.obs_pos[2] = float(data.pose.pose.position.z)
        self.obs_pos[3] = float(data.twist.twist.linear.x)
        self.obs_pos[4] = float(data.twist.twist.linear.y)
        self.obs_pos[5] = float(data.twist.twist.linear.z)


    def ref_cb(self, data):
        if(self.follow_flag):
            self.pos_off[0] = float(data.pose.pose.position.x)
            self.pos_off[1] = float(data.pose.pose.position.y)
            self.pos_off[2] = float(data.pose.pose.position.z)



if __name__ == '__main__':
     try:
        rospy.init_node('crazyflie_controller', anonymous=True)
        dc = DroneController('controller')
     except rospy.ROSInterruptException:
        pass
