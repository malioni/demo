#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from rosflight_msgs.msg import Command
from diff_flatness import diff_flatness
from traj_planner import trajectory_planner
from controller import controller
import yaml
# import matplotlib.pyplot as plt


class simTester:

    def __init__(self):
        self.pub = rospy.Publisher('/command',Command, queue_size=10, latch=True)
        self.command_msg = Command()
        self.command_msg.mode = self.command_msg.MODE_ROLL_PITCH_YAWRATE_THROTTLE
        self.count = 1 # used for initiating time

        rospy.Subscriber('/odom',Odometry,self.callback)

        # with open('/home/matiss/demo_ws/src/demo/scripts/demo.yaml','r') as f:
        #   param = yaml.safe_load(f)

        param = rospy.get_param('~')

        self.mass = param['dynamics']['mass']
        self.g = param['dynamics']['g']

        self.controller = controller(param) # initiate controller
        # calculate the equilibrium force
        self.force_adjust = param['dynamics']['mass']*param['dynamics']['g']/param['controller']['equilibrium_throttle']
        # plotting variables
        # self.time = []
        # self.pos_x_des = []
        # self.pos_x_actual = []
        # self.pos_y_des = []
        # self.pos_y_actual = []
        # self.pos_z_des = []
        # self.pos_z_actual = []

    def callback(self,msg):

        # for some reason time did not initiate correctly if done
        # in the __init__ function
        if self.count == 1:
            self.start_time = rospy.Time.now()
            self.count = 0
        
        time = rospy.Time.now()
        time_from_start = time.to_sec() - self.start_time.to_sec() 
        pose = trajectory_planner(time_from_start)

        pos = msg.pose.pose.position
        # The mekf gives unusual odometry message, the coordinates are different than NED
        pos = np.array([-1.*pos.y,-1.*pos.z,pos.x])
        attitude = msg.pose.pose.orientation
        vel = msg.twist.twist.linear
        vel = np.array([-1.*vel.y,-1.*vel.z,vel.x])
        ang_curr = self.euler(attitude)

        # plotting variables
        # self.time.append(time_from_start)
        # self.pos_x_actual.append(pos[0])
        # self.pos_x_des.append(pose[0][0])

        # self.pos_y_actual.append(pos[1])
        # self.pos_y_des.append(pose[0][1])
        # self.pos_z_actual.append(pos[2])
        # self.pos_z_des.append(pose[0][2])

        reference = np.array([pose[0],pose[2],pose[4],pose[1]])
        state = np.array([pos,vel,ang_curr[2]])

        control_inputs = self.controller.update(reference,state)

        accel_input = np.array([control_inputs[0],pose[3]+control_inputs[1]])

        states = diff_flatness(pose, accel_input, ang_curr,mass=self.mass,g=self.g)
        # R = states[1]
        # angle = self.angles(R)

        force = states[2] / self.force_adjust
        force = self.controller.saturate(force)
        roll = states[0]
        pitch = states[1]
        yaw_rate = accel_input[1]

        self.command_msg.x = roll
        self.command_msg.y = pitch
        self.command_msg.z = 0.0 # yaw_rate if variable desired yaw
        self.command_msg.F = force
        self.command_msg.ignore = Command.IGNORE_X | Command.IGNORE_Y | Command.IGNORE_Z
        self.pub.publish(self.command_msg)


    def update(self):
        rospy.spin()

    def angles(self,R): # ends up unused for now
        yaw = np.arctan2(-R[0][1],R[1][1])
        pitch = np.arctan2(-R[2][0],R[2][2])
        roll = np.arctan2(R[2][1]*np.cos(pitch),R[2][2])
        return np.array([roll,pitch,yaw])

    def euler(self, quat):
        w = quat.w
        x = -1.*quat.y
        y = -1.*quat.z
        z = quat.x

        roll = np.arctan2(2.0 * (w * x + y * z), 1. - 2. * (x * x + y * y))
        pitch = np.arcsin(2.0 * (w * y - z * x))
        yaw = np.arctan2(2.0 * (w * z + x * y), 1. - 2. * (y * y + z * z))
        return np.array([roll,pitch,yaw])


if __name__ == '__main__':
    rospy.init_node('sim_tester', anonymous=True)
    sim_tester = simTester()

    while not rospy.is_shutdown():
        try:
            sim_tester.update()
        except rospy.ROSInterruptException:
            print("exiting....")

    # Plot the states over time and relative to each other
    # plt.figure(1)
    # plt.subplot(311)
    # plt.plot(sim_tester.time,sim_tester.pos_x_actual)
    # plt.plot(sim_tester.time,sim_tester.pos_x_des)

    # plt.subplot(312)
    # plt.plot(sim_tester.time,sim_tester.pos_y_actual)
    # plt.plot(sim_tester.time,sim_tester.pos_y_des)

    # plt.subplot(313)
    # plt.plot(sim_tester.time,sim_tester.pos_z_actual)
    # plt.plot(sim_tester.time,sim_tester.pos_z_des)

    # plt.figure(2)
    # plt.subplot(311)
    # plt.plot(sim_tester.pos_x_actual,sim_tester.pos_y_actual)
    # plt.plot(sim_tester.pos_x_des,sim_tester.pos_y_des)

    # plt.subplot(312)
    # plt.plot(sim_tester.pos_x_actual,sim_tester.pos_z_actual)
    # plt.plot(sim_tester.pos_x_des,sim_tester.pos_z_des)

    # plt.subplot(313)
    # plt.plot(sim_tester.pos_y_actual,sim_tester.pos_z_actual)
    # plt.plot(sim_tester.pos_y_des,sim_tester.pos_z_des)

    # plt.show()