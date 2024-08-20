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

        self.t = rospy.get_time()

        self.drones = [DroneParameters('dcf2'), DroneParameters('dcf6')]
        self.ugvs = [UGV('turtle1'), UGV('turtle2')]
        self.lenDrones = len(self.drones)
        self.rate = rospy.Rate(120)
        self.modeSub = rospy.Subscriber('/uav_mode', Int8, self.setMode)

        for drone in self.drones:
            self.droneOdomSub.append(rospy.Subscriber('/vicon/{}/{}/odom'.format(drone.name, drone.name), Odometry, drone.odom_cb))
            self.droneParamSub.append(rospy.Subscriber('/{}/params'.format(drone.name), DroneParamsMsg, drone.params_cb))
            self.droneRefPub.append(rospy.Publisher('/{}/ref'.format(drone.name), PosVelMsg, queue_size=10))
            self.droneConsPub.append(rospy.Publisher('/{}/cons'.format(drone.name), ConstraintMsg, queue_size=10))
            self.droneModePub.append(rospy.Publisher('/{}/uav_mode'.format(drone.name), Int8, queue_size=10))


        for ugv in self.ugvs:
            self.ugvOdomSub.append(rospy.Subscriber('/vicon/{}/{}/odom'.format(ugv.name, ugv.name), Odometry, ugv.odom_cb))
            self.ugvCmdPub.append(rospy.Publisher('/{}/cmd_vel'.format(ugv.name), Twist, queue_size=10))


        self.filterFlag = False


        print('Sleeping')
        time.sleep(1)

        for i in range(self.lenDrones):
            modeMsg = Int8()
            modeMsg.data = 1
            self.droneModePub[i].publish(modeMsg)

        while not rospy.is_shutdown():
            self.loop()


    def loop(self):
        A_ = [np.zeros((1,3))]*self.lenDrones
        b_ = [0]*self.lenDrones
        i = 0
        for droneI, ugvI in zip(self.drones, self.ugvs):
            if droneI.odomFlag:
                if self.filterFlag: 
                    sqHorDist = sq_dist(ugvI.pos[:2] - droneI.pos[:2], np.ones((3,)))
                    ugvErrPos = droneI.pos - ugvI.pos
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
                    droneConsMsg = ConstraintMsg()
                    droneConsMsg.constraints = np.append(np.flatten(A_[i]), b_).tolist()
                    self.droneConsPub[i](droneConsMsg)

                    refMsg = PosVelMsg()
                    if droneI.followFlag:
                        self.getPosVelMsg(refMsg, droneI.offsetAngle, rospy.get_time())

                    elif droneI.returnFlag:
                        refMsg.position = [ugvI.pos[0], ugvI.pos[1], ugvI.pos[2]]
                        refMsg.velocity = [ugvI.vel[0], ugvI.vel[1], ugvI.vel[2]]
                        refMsg.yaw = 0.0
                        refMsg.yawVelocity = 0.0

                    self.droneRefPub[i].publish(refMsg)
                    print('Publishing: {}'.format(droneI.name))
                    self.rate.sleep()
            else:
                # print('Odometry Not received for {}'.format(droneI.name))
                pass
            i = i + 1


    def setMode(self, msg):
        if self.filterFlag == False:
            self.filterFlag = True
            self.t = rospy.get_time()
        for i in range(self.lenDrones):
            if msg.data[i] == 0 or msg.data[i] == 2:
                droneModeMsg = Int8()
                droneConsMsg.data = msg.data[i]
                self.droneModePub[i].publish(droneConsMsg)
                self.drones[i].followFlag = True
                self.drones[i].returnFlag = False

            elif msg.data[i] == 1:
                self.drones[i].returnFlag = True
                self.drones[i].followFlag = False

    def land_cb(self, data):
        self.landFlag = True
        print('Safety Landing: Active')

    def getPosVelMsg(self, msg, offset, time):
        time =  time - self.t
        msg.position = [np.cos(offset + time/10), np.sin(offset + time/10), 1.0]
        msg.velocity = [0.1*np.sin(offset + time/10), -0.1*np.cos(offset + time/10), 0.0]
        msg.yaw = 0.0
        msg.yawVelocity = 0.0





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
