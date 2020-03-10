import numpy as np

def diff_flatness(trajectory,inputs,angle_current,mass=2.856,g=9.81):
	pos = trajectory[0]
	yaw = trajectory[1]
	vel = trajectory[2]
	yaw_d = inputs[1]
	accel = inputs[0]
	yaw = angle_current[2]

	# Calculate total force
	F = mass*np.linalg.norm(accel-np.array([0,0,g]))

	# Calculate roll and pitch from desired acceleration
	roll = -1/g*(accel[0]*np.sin(yaw)-accel[1]*np.cos(yaw))
	pitch = -1/g*(accel[0]*np.cos(yaw)+accel[1]*np.sin(yaw))

	states = np.array([roll, pitch, F])
	return states