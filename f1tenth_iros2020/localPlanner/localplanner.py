#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
"""
# Add paths
import os, sys
sys.path.append(os.path.join('..', 'ext', 'BeBOT'))

import matplotlib.pyplot as plt
from numba import njit
import numpy as np
from scipy.optimize import Bounds, minimize

from optimization.AngularRate import angularRate
from optimization.Speed import speed
from optimization.ObstacleAvoidance import obstacleAvoidance
from polynomial.bernstein import Bernstein


def initGuess(params):
    lin = np.linspace(0, 1, params.deg-1)
    x = params.x0[0] + params.v0*np.cos(params.psi0)*params.tf/params.deg + lin*(params.xf[0] - params.x0[0])
    y = params.x0[1] + params.v0*np.sin(params.psi0)*params.tf/params.deg + lin*(params.xf[1] - params.x0[1])

    x0 = np.concatenate((x, y))

    return x0


@njit(cache=True)
def reshape(x, deg, tf, x0, v0, psi0):
    y = np.empty((2, deg+1))
    y[:, 0] = x0
    y[0, 1] = x0[0] + v0*np.cos(psi0)*tf/deg
    y[1, 1] = x0[1] + v0*np.sin(psi0)*tf/deg

    y[:, 2:] = x.reshape((2, -1))

    return y


def nonlcon(x, params):
    y = reshape(x, params.deg, params.tf, params.x0, params.v0, params.psi0)

    traj = Bernstein(y, tf=params.tf)

    maxSpeed = params.vmax**2 - speed(traj)
    maxAngRate = params.wmax**2 - angularRate(traj)
    separation = obstacleAvoidance([traj], params.obs, elev=params.elev) - params.dsafe**2

    return np.concatenate([maxSpeed, maxAngRate, separation])

def cost(x, params):
    y = reshape(x, params.deg, params.tf, params.x0, params.v0, params.psi0)

    euclid = 1/np.diff(y**2).sum()
    terminal = ((y[:, -1] - params.xf)**2).sum()

    return euclid + terminal


class Parameters:
    def __init__(self):
        self.deg = 3
        self.elev = 10
        self.tf = 3.0
        self.vmax = 10.0
        self.wmax = np.pi/5
        self.dsafe = 0.3

        self.x0 = np.array([0, 0], dtype=float)
        self.v0 = 1.0
        self.psi0 = np.pi/4
        self.xf = np.array([5, 5], dtype=float)
        self.obs = np.array([[3, 3], [4, 1], [1, 2]])


if __name__ == '__main__':
    params = Parameters()
    x0 = initGuess(params)

    cons = [{'type': 'ineq',
             'fun': lambda x: nonlcon(x, params)}]

    results = minimize(lambda x: cost(x, params), x0,
                       constraints=cons,
                       # bounds=bounds,
                       method='SLSQP',
                       options={'maxiter': 250,
                                'disp': True,
                                'iprint': 1})

    y = reshape(results.x, params.deg, params.tf, params.x0, params.v0, params.psi0)
    print(y)
    traj = Bernstein(y, tf=params.tf)
    plt.close('all')
    ax = traj.plot()
    for obs in params.obs:
        ax.add_artist(plt.Circle(obs, radius=params.dsafe, ec='k'))