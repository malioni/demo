import numpy as np

class controller:
    def __init__(self, param):
        # self.kp_yaw = np.array(param['controller']['Kp_yaw'])
        self.kp_accel = np.diag([param['controller']['kp_accel_xy'],param['controller']['kp_accel_xy'],param['controller']['kp_accel_z']])
        self.kd_accel = np.diag([param['controller']['kd_accel_xy'],param['controller']['kd_accel_xy'],param['controller']['kd_accel_z']])
        self.vel_last = np.array([0.0,0.0,0.0])

    def update(self, reference, state):

        acc_des = reference[2]
        vel_des = reference[1]
        pos_des = reference[0]
        # yaw_des = reference[3]

        vel = state[1]
        pos = state[0]
        # yaw = state[2]

        acc_com = acc_des+np.matmul(self.kd_accel,vel_des-vel)+np.matmul(self.kp_accel,pos_des-pos)
        yaw_rate = 0.0#self.kp_yaw*(yaw_des-yaw)

        return np.array([acc_com,yaw_rate])

    def saturate(self, force):
        if abs(force) > 1.0:
            force = 1.0 * np.sign(force)
        return force

