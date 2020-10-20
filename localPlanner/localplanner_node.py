#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 19 15:57:17 2020

@author: ckjensen
"""
# Add paths
import os
import sys
sys.path.append(os.path.join('..', 'ext', 'BeBOT'))

import json
import matplotlib.pyplot as plt
from numba import njit
import numpy as np
from scipy.optimize import Bounds, minimize
from scipy.spatial.transform import Rotation as R

from obstacle_detector.msg import Obstacles
from nav_msgs.msg import Odometry
import rospy

from f1tenth_iros2020.msg import Trajectory
from optimization.AngularRate import angularRate
from optimization.Speed import speed
from optimization.ObstacleAvoidance import obstacleAvoidance
from polynomial.bernstein import Bernstein


class LocalPlanner:
    def __init__(self, deg=3, elev=10, tf=3.0, vmax=10.0, wmax=np.pi/5, dsafe=0.3, debug=False):
        self.deg = deg
        self.elev = elev
        self.tf = tf
        self.vmax = vmax
        self.wmax = wmax
        self.dsafe = dsafe

        self.debug = debug

        self.x0 = np.array([0, 0], dtype=float)
        self.v0 = 1.0
        self.psi0 = np.pi/4
        self.xf = np.array([5, 5], dtype=float)
        self.obs = np.array([[3, 3], [4, 1], [1, 2]])

    def setInitState(self, x0, v0, psi0):
        self.x0 = x0
        self.v0 = v0
        self.psi0 = psi0

    def setObstacles(self, obstacleArray):
        self.obs = obstacleArray.astype(float)

    def setTerminal(self, xf):
        self.xf = xf

    def initGuess(self):
        lin = np.linspace(0, 1, self.deg-1)
        x = self.x0[0] + self.v0*np.cos(self.psi0)*self.tf/self.deg + lin*(self.xf[0] - self.x0[0])
        y = self.x0[1] + self.v0*np.sin(self.psi0)*self.tf/self.deg + lin*(self.xf[1] - self.x0[1])

        x0 = np.concatenate((x, y))

        return x0


    def nonlcon(self, x):
        y = _reshape(x, self.deg, self.tf, self.x0, self.v0, self.psi0)

        traj = Bernstein(y, tf=self.tf)

        maxSpeed = self.vmax**2 - speed(traj)
        maxAngRate = self.wmax**2 - angularRate(traj)
        separation = obstacleAvoidance([traj], self.obs, elev=self.elev) - self.dsafe**2

        return np.concatenate([maxSpeed, maxAngRate, separation])

    def cost(self, x):
        y = _reshape(x, self.deg, self.tf, self.x0, self.v0, self.psi0)

        euclid = 1/np.diff(y**2).sum()
        terminal = ((y[:, -1] - self.xf)**2).sum()

        return euclid + terminal


    def plan(self):
        x0 = self.initGuess()

        cons = [{'type': 'ineq',
                 'fun': lambda x: self.nonlcon(x)}]

        results = minimize(self.cost, x0,
                           constraints=cons,
                           # bounds=bounds,
                           method='SLSQP',
                           options={'maxiter': 250,
                                    'disp': self.debug,
                                    'iprint': 1})

        y = _reshape(results.x, self.deg, self.tf, self.x0, self.v0, self.psi0)
        traj = Bernstein(y, tf=self.tf)

        return traj


@njit(cache=True)
def _reshape(x, deg, tf, x0, v0, psi0):
    y = np.empty((2, deg+1))
    y[:, 0] = x0
    y[0, 1] = x0[0] + v0*np.cos(psi0)*tf/deg
    y[1, 1] = x0[1] + v0*np.sin(psi0)*tf/deg

    y[:, 2:] = x.reshape((2, -1))

    return y


def odomCB(data, lp):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    v = data.twist.twist.x

    wq = data.pose.pose.orientation.w
    xq = data.pose.pose.orientation.x
    yq = data.pose.pose.orientation.y
    zq = data.pose.pose.orientation.z
    r = R.from_quat([xq, yq, zq, wq])
    phi, gamma, psi = r.as_euler('xyz')

    lp.setInitState(np.array([x, y], dtype=float), v, psi)


def obsCB(data, lp):
    pass


if __name__ == '__main__':
    with open(os.path.join(os.path.dirname(__file__), '..', 'config.json'), 'r') as f:
        data = json.load(f)
    thor = data['local_planner']['time_horizon']

    lp = LocalPlanner()

    # Initialize ROS
    rospy.init_node('local_planner')
    planPub = rospy.Publisher('local_planner/trajectory', Trajectory, queue_size=10)

    rospy.Subscriber('/odom', Odometry, lambda x: odomCB(x, lp), queue_size=10)
    rospy.Subscriber('/processed_obstacles', Obstacles, lambda x: obsCB(x, lp), queue_size=10)

    tlast = rospy.get_time()
    while not rospy.is_shutdown():
        tnow = rospy.get_time()
        if tnow - tlast >= thor:
            tlast = tnow

