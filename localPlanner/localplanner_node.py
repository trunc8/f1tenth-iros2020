#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 19 15:57:17 2020

@author: ckjensen
"""
# Add paths
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'ext', 'BeBOT'))

import json
# import matplotlib.pyplot as plt
from numba import njit
import numpy as np
from scipy.optimize import Bounds, minimize
from scipy.spatial.transform import Rotation as R

# from obstacle_detector.msg import Obstacles
from nav_msgs.msg import Odometry, OccupancyGrid
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy

from f1tenth_iros2020.msg import Trajectory, Obstacles
from optimization.AngularRate import angularRate
from optimization.Speed import speed
from optimization.ObstacleAvoidance import obstacleAvoidance
from polynomial.bernstein import Bernstein


class LocalPlanner:
    def __init__(self, deg=3, elev=10, tf=3.0, vmax=7.0, wmax=np.pi/2, dsafe=0.03, debug=False):
        self.deg = deg
        self.elev = elev
        self.tf = tf
        self.vmax = vmax
        self.wmax = wmax
        self.dsafe = dsafe

        self.debug = debug

        fname = os.path.join(os.path.dirname(__file__), 'waypoints.npy')
        with open(fname, 'rb') as f:
            self.wpts = np.load(f)

        self.x0 = self.wpts[100, :2]
        self.v0 = 0
        self.psi0 = np.arctan2(self.wpts[101, 1] - self.wpts[100, 1], self.wpts[101, 0] - self.wpts[100, 0])
        self.obs = -np.array([[3, 3], [4, 1], [1, 2]]) + self.wpts[100, :2]
        self.obsRad = np.array([0.01, 0.02, 0.03])

    def setInitState(self, x0, v0, psi0):
        self.x0 = x0
        self.v0 = v0
        self.psi0 = psi0

    def setObstacles(self, obstacleArray):
        self.obs = obstacleArray.astype(float)

    def setTerminal(self, xf):
        self.xf = xf

    def initGuess(self):
        temp = np.append(self.x0, self.psi0)
        idx = np.argmin(np.abs(temp - self.wpts).sum(1))
        xf = self.wpts[idx+25, :2]

        x1 = self.x0[0] + self.v0*np.cos(self.psi0)*self.tf/self.deg
        y1 = self.x0[1] + self.v0*np.sin(self.psi0)*self.tf/self.deg
        lin = np.linspace(0, 1, self.deg)
        x = x1 + lin*(xf[0] - x1)
        y = y1 + lin*(xf[1] - y1)

        x0 = np.concatenate((x[1:], y[1:]))

        return x0


    def nonlcon(self, x):
        y = _reshape(x, self.deg, self.tf, self.x0, self.v0, self.psi0)

        traj = Bernstein(y, tf=self.tf)

        maxSpeed = self.vmax**2 - speed(traj)
        maxAngRate = self.wmax**2 - angularRate(traj)
        radii = np.repeat(self.obsRad**2, 2*self.deg+1+self.elev)
        separation = obstacleAvoidance([traj], self.obs, elev=self.elev) - radii - self.dsafe**2
        track = obstacleAvoidance([traj], self.track, elev=self.elev) - self.dsafe**2

        return np.concatenate([maxSpeed, maxAngRate, separation, track])

    def cost(self, x):
        y = _reshape(x, self.deg, self.tf, self.x0, self.v0, self.psi0)

        # euclid = np.diff(y**2).sum()
        dist = -np.linalg.norm(y[:, -1] - y[:, 0])

        # terminal = ((y[:, -1] - self.xf)**2).sum()
        # psif = np.arctan2(y[1, -1]-y[1, -2], y[0, -1]-y[0, -2])
        # xf = np.append(y[:, -1], psif)
        # terminal = np.abs(xf - self.wpts).sum(1).min()
        terminal = ((y[:, -1] - self.wpts[:, :-1])**2).sum(1).min()

        return 10*terminal + dist


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

        self.res = results
        # If we are not successful, just assume a straight line guess
        if results.success:
            y = _reshape(results.x, self.deg, self.tf, self.x0, self.v0, self.psi0)
        else:
            y = _reshape(x0, self.deg, self.tf, self.x0, self.v0, self.psi0)
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
    v = data.twist.twist.linear.x

    wq = data.pose.pose.orientation.w
    xq = data.pose.pose.orientation.x
    yq = data.pose.pose.orientation.y
    zq = data.pose.pose.orientation.z
    r = R.from_quat([xq, yq, zq, wq])
    phi, gamma, psi = r.as_euler('xyz')

    lp.setInitState(np.array([x, y], dtype=float), v, psi)


def obsCB(data, lp):
    obsList = []
    obsRadList = []
    for circ in data.circles:
        obsList.append((circ.center.x, circ.center.y))
        obsRadList.append(circ.radius)

    lp.obs = np.array(obsList, dtype=float)
    lp.obsRad = np.array(obsRadList, dtype=float)


def mapCB(data, lp):
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution

    grid = np.reshape(data.data, (width, height))

    centerMin = np.round((lp.x0-7)/resolution).astype(int)
    centerMax = np.round((lp.x0+7)/resolution).astype(int)

    locGrid = grid[centerMin[0]:centerMax[0], centerMin[1]:centerMax[1]]
    x, y = np.where(locGrid > 50)

    lp.track = np.vstack((x, y))


def buildTrajMsg(traj, thor):
    T = np.linspace(0, thor, 100)
    trajMsg = JointTrajectory()

    for t in T:
        pt = JointTrajectoryPoint()
        pt.time_from_start = rospy.Duration(t)
        pt.positions = list(traj(t))
        pt.velocities = list(traj.diff()(t))
        pt.accelerations = list(traj.diff().diff()(t))

        trajMsg.points.append(pt)

    trajMsg.header.frame_id = 'map'
    trajMsg.header.stamp = rospy.get_rostime()

    return trajMsg


if __name__ == '__main__':
    with open(os.path.join(os.path.dirname(__file__), '..', 'config.json'), 'r') as f:
        data = json.load(f)
    thor = data['local_planner']['time_horizon']

    lp = LocalPlanner()

    # # Initialize ROS
    rospy.init_node('local_planner')
    planPub = rospy.Publisher('/waypoints', Trajectory, queue_size=10)

    rospy.Subscriber('/odom', Odometry, lambda x: odomCB(x, lp), queue_size=10)
    rospy.Subscriber('/processed_obstacles', Obstacles, lambda x: obsCB(x, lp), queue_size=10)
    rospy.Subscriber('/map', OccupancyGrid, lambda x: mapCB(x, lp), queue_size=10)

    rospy.loginfo('Waiting for odometry message...')
    rospy.wait_for_message('/odom', Odometry)
    rospy.loginfo('---> Odometry message found!')
    rospy.loginfo('Waiting for obstacle message...')
    rospy.wait_for_message('/processed_obstacles', Obstacles)
    rospy.loginfo('---> Obstacle message found!')

    traj = lp.plan()

    #=========================================================================
    # Testing Code

    # print(lp.res)
    # print(lp.nonlcon(lp.initGuess()))
    # y = _reshape(lp.initGuess(), lp.deg, lp.tf, lp.x0, lp.v0, lp.psi0)
    # trajInit = Bernstein(y, tf=lp.tf)
    # plt.close('all')
    # ax = traj.plot()
    # ax.plot(lp.wpts[100:150, 0], lp.wpts[100:150, 1])
    # for i, obs in enumerate(lp.obs):
    #     ax.add_artist(plt.Circle(obs, radius=lp.obsRad[i], ec='k'))
    # trajInit.plot(ax)
    #=========================================================================

    trajMsg = buildTrajMsg(traj, thor)
    planPub.publish(trajMsg)
    tlast = rospy.get_time()
    while not rospy.is_shutdown():
        tnow = rospy.get_time()
        if tnow - tlast >= thor:
            tlast = tnow
            traj = lp.plan()
            trajMsg = buildTrajMsg(traj, thor)
            planPub.publish(trajMsg)

