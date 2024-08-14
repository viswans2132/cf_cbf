 #!/usr/bin/env python
 # license removed for brevity
import pkg_resources
from tf.transformations import euler_from_quaternion, quaternion_matrix
import time
import numpy as np
import cvxpy as cp
import sys



class Drone:
    name = 'crazyflie'

    def __init__(self, name):
        self.name = name
    
        self.pos = np.array([0.0, 0, 0])
        self.quat = np.array([0.0, 0, 0, 1])
        self.yaw = 0.0
        self.R = np.eye(3)

        self.vel = np.array([0, 0.0, 0])
        self.ang_vel = np.array([0.0, 0, 0])

        self.desPos = np.array([0.3, 4.0, 0.8])
        self.desVel = np.array([0.0, 0, 0])
        self.desYaw = 0.0
        self.desYawVel = 0.0
        self.errVel = np.array([0.0, 0, 0])
        self.errInt = np.array([0.0, 0, 0])
        self.errVelInt = np.array([0.0, 0, 0])
        self.errRel = np.array([[10.0, 10, 10], [10.0, 10, 10]]).T

        self.odomStatus = False
        self.hz = 30.0
        self.dt = 1/self.hz

        self.Kpos = np.array([-1.5, -1.5, -0.8])
        self.Kvel = np.array([-0.5, -0.5, -0.5])
        self.Kder = np.array([-0.05, -0.05, -0.05])
        self.KintP = np.array([-0.0, -0.0, -0.0])
        self.KintV = np.array([-0.3, -0.3, -0.2])
        self.Kyaw = 1

        self.kRad = np.array([0.16, 0.16, 0.64])
        self.omegaD = 1.0

        self.landCounter = 0.0
        self.landFlag = False
        self.startFlag = False
        self.filterFlag = False
        self.followFlag = False

        self.maxInt = np.array([0.5, 0.5, 0.0])
        self.maxVelInt = np.array([0.0, 0.0, 0.5])
        self.maxAcc = np.array([0.25, 0.25, 0.3])

        self.A = np.zeros((3,))
        self.b = 0.0
        self.P = np.eye(3)
        self.u = cp.Variable(3)

        print("crazyflie added: {}".format(self.name))

    def setOdom(self, position, quat, velocity):
        self.pos[0] = position[0]
        self.pos[1] = position[1]
        self.pos[2] = position[2]
        self.quat[0] = quat[0]
        self.quat[1] = quat[1]
        self.quat[2] = quat[2]
        self.quat[3] = quat[3]
        self.yaw = euler_from_quaternion(self.quat)[2]
        R_inv = quaternion_matrix(self.quat)[:-1, :-1]
        # self.R = np.linalg.inv(R_inv)
        self.R = np.array([[np.cos(self.yaw), np.sin(self.yaw), 0], [-np.sin(self.yaw), np.cos(self.yaw), 0], [0, 0, 1]])
        self.vel[0] = velocity[0]
        self.vel[1] = velocity[1]
        self.vel[2] = velocity[2]
        self.ang_vel[2] = velocity[3]

        if self.odomStatus == False:
            self.desPos[0] = self.pos[0]
            self.desPos[1] = self.pos[1]
        self.odomStatus = True
        # print('odom_received')

    def clearConstraintMatrices(self, A_, b_):
        self.A = np.empty()
        self.b = np.vstack((self.b, b_))

    def updateConstraintMatrices(self, A_, b_):
        self.A = A_
        self.b = b_

    def setMode(self, data):
        if data == 0:
            self.startFlag = True
            print('takeoff {}'.format(self.name))
        if data == 1:
            self.filterFlag = True
            print('filter: ON {}'.format(self.name))
        if data == 2:
            self.landFlag = True
            print('landing {}'.format(self.name))

    def filterValues(self, err, u_):
        constraints = [self.A@self.u >= -self.b]
        prob = cp.Problem(cp.Minimize(cp.quad_form(self.u-u_, self.P)), constraints)
        result = prob.solve()

        desVel = self.u.value

        if np.linalg.norm(desVel) > 0.3173:
            desVel = desVel*0.3173/np.linalg.norm(desVel)

        return desVel


    def setFollow(self, data):
        self.followFlag = True

    def setRef(self, pos, vel):
        self.desPos = np.array([pos[0], pos[1], pos[2]])
        self.desVel = np.array([vel[1], vel[2], vel[3]])
        self.desYaw = pos[3]
        self.desYawVel = vel[3]

        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.desYaw = euler_from_quaternion(q)[2]

    def publishCmdVel(self, data):
        self.cmd_pub.publish(data)
    
    def odomStatus(self):
        return self.odomStatus

    def generateControlInputs(self, velArray):
        uPitch = 0.0
        uRoll = 0.0
        uThrust = 0.0
        uYaw = 0.0
        if self.odomStatus:
            # print("Odometry status is: ".format(self.odomStatus))
            errPos = self.pos - self.desPos
            if self.startFlag:
                self.errInt = self.errInt + errPos*self.dt
            self.errInt = np.maximum(-self.maxInt, np.minimum(self.maxInt, self.errInt))

            self.desVel = self.Kpos * errPos + self.KintP * self.errInt

            if self.filterFlag:
                self.desVel = self.filterValues(errPos, self.desVel)


            derVel = ((self.vel - self.desVel) - self.errVel)/self.dt
            self.errVel = self.vel - self.desVel

            if self.startFlag:
                self.errVelInt = self.errVelInt + self.errVel*self.dt
            self.errVelInt = np.maximum(-self.maxVelInt, np.minimum(self.maxVelInt, self.errVelInt))
            # print(self.errVelInt)

            des_a = self.Kvel * self.errVel + self.Kder * derVel + self.KintV * self.errVelInt
            # des_a = self.Kvel * self.errVel
            des_a = self.R.dot(des_a)
            # print("Error: {0:.3f}: {1:.3f}: {2:.3f}: \n Acc: {3:.3f}: {4:.3f}: {5:.3f}".format(self.errVel[0], errPos[1], errPos[2], des_a[0], des_a[1], des_a[2]))
            # print(des_a)
            # print(des_a[3])
            des_a = np.maximum(-self.maxAcc, np.minimum(self.maxAcc, des_a))
            # print("{:.3f}: {:.3f}: {:.3f}".format(errPos[0], errPos[1], errPos[2]))

            # yaw_diff = np.minimum(0.2, np.maximum(self.desYaw - self.yaw, -0.2))

            yaw_des = -1.0*self.yaw -0.5*self.ang_vel[2]


            uPitch = des_a[0]
            uRoll = des_a[1]
            uYaw = yaw_des

            if self.landFlag:
                self.startFlag = False
                uThrust = 0.6 - self.landCounter*0.01
                if self.landCounter > 50:
                    uThrust = 0.0
                    uRoll = 0.0
                    uPitch = 0.0
                    uyaw = 0.0

                self.landCounter =  self.landCounter + 1

            elif self.startFlag:
                uThrust = des_a[2] + 0.62
        else:
            print("{}: Odometry not received".format(self.name))

        velArray[0] = uPitch
        velArray[1] = uRoll
        velArray[2] = uYaw
        velArray[3] = uThrust

        return self.odomStatus
        # print("{}: {:.3f}: {:.3f}: {:.3f}: {:.3f}: Ready to publish".format(self.name, uPitch, uRoll, uYaw, uThrust))


        # self.cmdPub.publish(velMsg)