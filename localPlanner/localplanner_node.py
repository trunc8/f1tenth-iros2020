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

import matplotlib.pyplot as plt
from numba import njit
import numpy as np
from scipy.optimize import Bounds, minimize

from optimization.AngularRate import angularRate
from optimization.Speed import speed
from optimization.ObstacleAvoidance import obstacleAvoidance
from polynomial.bernstein import Bernstein


class LocalPlanner:
    def __init__(self, debug=False):
        self.deg = 3
        self.elev = 10
        self.tf = 3.0
        self.vmax = 10.0
        self.wmax = np.pi/5
        self.dsafe = 0.3

        self.debug = debug

        self.x0 = np.array([0, 0], dtype=float)
        self.v0 = 1.0
        self.psi0 = np.pi/4
        self.xf = np.array([5, 5], dtype=float)
        self.obs = np.array([[3, 3], [4, 1], [1, 2]])

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


if __name__ == '__main__':

    lp = LocalPlanner()
    traj = lp.plan()

    plt.close('all')
    ax = traj.plot()
    for obs in lp.obs:
        ax.add_artist(plt.Circle(obs, radius=lp.dsafe, ec='k'))