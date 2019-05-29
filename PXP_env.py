import vrep
from coord_to_polar import *
import sys
import random as r
import math as m
from time import sleep
import numpy as np




class PXP_env():
    action_space = None
    observation_space = None

    def __init__(self):
        self.clientID = None
        self.handle_joint_1 = None
        self.handle_joint_2 = None
        self.handle_joint_3 = None
        self.handle_joint_4 = None
        self.handle_end_pincher = None
        self.handle_ball = None
        self.handle_end_tester = None
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.q4 = 0
        self.ball_coord = None
        self.ball_coord_x = None
        self.ball_coord_y = None
        self.ball_coord_z = None
        self.end_coord = None
        self.end_coord_x = None
        self.end_coord_y = None
        self.end_coord_z = None
        self.obs = None
        self.dist = None
        self.num_steps = 0
        self.render_steps = 0






    def step(self, action):

        self.distance()
        start_dist = self.dist

        if action == 0: self.q3 += m.radians(2)
        elif action == 1: self.q3 -= m.radians(2)
        elif action == 2: self.q4 += m.radians(2)
        elif action == 3: self.q4 -= m.radians(2)


        self.setJoint_PXP()
        self.get_end_coord()
        obs = self.get_obs()
        reward = (start_dist - self.dist) * 25

        done = False

        self.num_steps += 1

        if self.dist < 0.01:
            reward += 5
            done = True # ?
        elif self.num_steps > 400:
            done = True  # ?

        info = {}

        return obs, reward, done, info








    def reset(self):

        self.num_steps = 0

        self.set_randomJoint_PXP()

        self.create_random_ball()

        return self.get_obs()






        #raise NotImplementedError()

    def configurate(self):
        print('Program started')

        vrep.simxFinish(-1)

        self.clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

        if self.clientID != -1:
            print("Connected to remote server")
        else:
            print('Connection not successful')
            sys.exit('Could not connect')
        errorCode, self.handle_joint_1 = vrep.simxGetObjectHandle(self.clientID,
                                                                  'PhantomXPincher_joint1',
                                                                  vrep.simx_opmode_oneshot_wait)
        errorCode, self.handle_joint_2 = vrep.simxGetObjectHandle(self.clientID,
                                                                  'PhantomXPincher_joint2',
                                                                  vrep.simx_opmode_oneshot_wait)
        errorCode, self.handle_joint_3 = vrep.simxGetObjectHandle(self.clientID,
                                                                  'PhantomXPincher_joint3',
                                                                  vrep.simx_opmode_oneshot_wait)
        errorCode, self.handle_joint_4 = vrep.simxGetObjectHandle(self.clientID,
                                                                  'PhantomXPincher_joint4',
                                                                  vrep.simx_opmode_oneshot_wait)

        errorCode, self.handle_end_pincher = vrep.simxGetObjectHandle(self.clientID,
                                                                      'PhantomXPincher_gripperCenter_joint',
                                                                      vrep.simx_opmode_oneshot_wait)
        errorCode, self.handle_ball = vrep.simxGetObjectHandle(self.clientID,
                                                               'Dummy',
                                                               vrep.simx_opmode_oneshot_wait)
        errorCode, self.handle_end_tester = vrep.simxGetObjectHandle(self.clientID,
                                                               'Dummy0',
                                                               vrep.simx_opmode_oneshot_wait)

        print('Configuration ended')

    def render(self, mode='human', close=False):

        self.render_steps += 1

        print('num_render:', self.render_steps)

        #raise NotImplementedError()

    def close(self):

        print('close me')

        #raise NotImplementedError()


    def setJoint_PXP(self):
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_1, self.q1, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_2, self.q2, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_3, self.q3, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_4, self.q4, vrep.simx_opmode_oneshot_wait)


    def set_randomJoint_PXP(self):
        self.q3 = m.radians(r.randrange(-90, 90, 2))
        self.q4 = m.radians(r.randrange(0, 90, 2))
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_1, self.q1, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_2, self.q2, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_3, self.q3, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_4, self.q4, vrep.simx_opmode_oneshot_wait)
        self.get_end_coord()
        while self.end_coord[2] < 0.23561:

            self.q3 = m.radians(r.randrange(-90, 90, 2))
            self.q4 = m.radians(r.randrange(0, 90, 2))
            vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_1, 0, vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_2, 0, vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_3, self.q3, vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(self.clientID, self.handle_joint_4, self.q4, vrep.simx_opmode_oneshot_wait)
            self.get_end_coord()



    def create_random_ball(self):
        q3 = m.radians(r.randrange(-90, 90, 2))
        q4 = m.radians(r.randrange(0, 90, 2))

        self.ball_coord_x = 1.1411e-05
        self.ball_coord_y = -3.9454e-02 - (3.4224e-01 - 2.3561e-01) * m.cos(q3) - (4.3130e-01 - 3.4224e-01) * m.cos(
            q3 + q4)
        self.ball_coord_z = 2.3561e-01 + (3.4224e-01 - 2.3561e-01) * m.sin(q3) + (4.3130e-01 - 3.4224e-01) * m.sin(
            q3 + q4)

        while self.ball_coord_z < 2.3561e-01:
            q3 = m.radians(r.randrange(-90, 90, 2))
            q4 = m.radians(r.randrange(0, 90, 2))

            self.ball_coord_x = 1.1411e-05
            self.ball_coord_y = -3.9454e-02 - (3.4224e-01 - 2.3561e-01) * m.cos(q3) - (4.3130e-01 - 3.4224e-01) * m.cos(
                q3 + q4)
            self.ball_coord_z = 2.3561e-01 + (3.4224e-01 - 2.3561e-01) * m.sin(q3) + (4.3130e-01 - 3.4224e-01) * m.sin(
                q3 + q4)
        self.ball_coord = [self.ball_coord_x, self.ball_coord_y, self.ball_coord_z]



        vrep.simxSetObjectPosition(self.clientID, self.handle_ball, -1, self.ball_coord, vrep.simx_opmode_oneshot_wait)


    def get_end_coord(self):
        sleep(0.1)
        errorCode, end_coord = vrep.simxGetObjectPosition(self.clientID, self.handle_end_pincher, -1,
                                                               vrep.simx_opmode_oneshot_wait)
        self.end_coord = list(end_coord)
        self.end_coord_x = self.end_coord[0]
        self.end_coord_y = self.end_coord[1]
        self.end_coord_z = self.end_coord[2]
        vrep.simxSetObjectPosition(self.clientID, self.handle_end_tester, -1, self.end_coord,
                                   vrep.simx_opmode_oneshot_wait)


    def distance(self):

        x1 = self.end_coord_x
        y1 = self.end_coord_y
        z1 = self.end_coord_z
        x2 = self.ball_coord_x
        y2 = self.ball_coord_y
        z2 = self.ball_coord_z

        self.dist = m.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)





    def get_obs(self):
        self.distance()
        dist = self.dist
        r1, alf1, fi1 = coord_to_polar(self.end_coord_x, self.end_coord_y, self.end_coord_z)
        r2, alf2, fi2 = coord_to_polar(self.ball_coord_x, self.ball_coord_y, self.ball_coord_z)

        result = np.array(
            [dist / 0.38898719798032544, 2 * self.q1 / m.pi, 2 * self.q2 / m.pi, 2 * self.q3 / m.pi, 2 * self.q4 / m.pi,
             self.end_coord_x / 0.35, self.end_coord_y / 0.35, self.end_coord_z / 0.45, r1 / 0.45, 2 * alf1 / m.pi,
             2 * fi1 / m.pi, self.ball_coord_x / 0.35, self.ball_coord_y / 0.35, self.ball_coord_z / 0.45, r2 / 0.45,
             2 * alf2 / m.pi, 2 * fi2 / m.pi])

        return result











'''

env = PXP_env()
env.configurate()


print(env.reset())
print(env.step(0))
print(env.step(0))
print(env.num_steps)
'''