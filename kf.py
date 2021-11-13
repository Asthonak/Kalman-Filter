# kf.py updated file for robot estimation assignment
# pset[3]
# (C) 2017 David Feil-Seifer
# Additional modifications done by Ethan Park

import numpy as np
import math
from scipy import stats
import scipy.stats

# kf_update: update state estimate [u, sigma] with new control [xdot] and measurement [z]
# 	parameters:
#			u : 2x1 vector with state estimate (x) at time t-1 and control (xdot) at time t
#			sigma: 2x2 matrix with covariance at time t-1
#			z (int): observed (uncertain) measurement of state (x) at time t
#	returns: [u sigma] updated state with estimate at time t

def kf_update(u, sigma, z):

	dt = 1.0
	F = np.matrix([[1, dt], [0, 1]])
	H = np.matrix([1,0])
	G = np.matrix([(dt*dt), (dt/2)]).T
	I = np.identity(2)

	sigma_x = np.linalg.matrix_power(G * G.T, 4)
	sigma_z = 5*5

	K_tp1 = ((F * sigma * F.T) + sigma_x) * H.T * np.linalg.inv(H * ((F * sigma * F.T) + sigma_x) * H.T + sigma_z)

	u_tp1 = F * u + K_tp1 * (z - H * F * u)

	sigma_tp1 = (I - (K_tp1 * H)) * ((F * sigma * F.T) + sigma_x)

	return [u_tp1, sigma_tp1]


# door_update: update estimate of door locations
# 	parameters:
#			u : 2x1 vector with state estimate (x) at time t-1 and control (xdot) at time t-1
#			sigma: 2x2 matrix with covariance at time t-1
#			d (binary): door sensor at time t-1 
#			door_dist (array of size 10): probability (0..1) that a door exists at each location (0..9)
#	returns: [door_dist] updated door distribution

true = [1,1,1,1,1,1,1,1,1,1]
false = [2,2,2,2,2,2,2,2,2,2]

def door_update(u, sigma, d, door_dist):
	current_door = int(u[0] / 100)

	if d:
		true[current_door] = true[current_door]+1
	else:
		false[current_door] = false[current_door]+1

	P_d = float(true[current_door]) / float(false[current_door])

	"""
	D = Hopper is at a door
	S = Sensor is correct
	P(S|D) = 0.6
	P(S|~D) = 0.8
	P(S) = P(S|D) * P(S|~D)
	P(D|S) = (P(S|D) * P(D)) / P(S)
	P(D|S) = (P(S|D) * P(D)) / (P(S|D) * P(S|~D))
	"""

	door_dist[current_door] = (0.6 * float(P_d)) / (0.6 * 0.8)

	# make sure probability is between 0 and 1
	if door_dist[current_door] > 1.0:
		door_dist[current_door] = 1.0
	if door_dist[current_door] < 0.0:
		door_dist[current_door] = 0.0

	return door_dist

