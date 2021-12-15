#!/usr/bin/env python
from __future__ import print_function
import sys
import numpy as np
import rospy
from std_msgs.msg import String, Bool, Int32
import time
from geometry_msgs.msg import WrenchStamped
import pickle


class Vaga:

    def __init__(self):


        rate_hz = 100.
        self.rate = rospy.Rate(rate_hz)

        self.opto_sub = rospy.Subscriber("/force_sensor/force_torque_output", WrenchStamped, self.callbackForce, queue_size = 1)

        self.reset_zero = rospy.Subscriber("/zero_now", Bool, self.zeroCallback)
        self.reset_weight = rospy.Subscriber("/weight_now", Bool, self.weightCallback)

        self.win_size = rospy.Subscriber("/window_size", Int32, self.winSizeCallback)

        self.pub = rospy.Publisher("/vaga_puna", Bool, queue_size = 1)

        self.win_size = 100

        self.zero = 0.
        self.weight = 0.5
        self.threshold = 0.1

        self.state = False # false = empty, true = full

        self.data = np.zeros((1,self.win_size))
        self.new_recording = False

        self.zero_data = []
        self.weight_data = []



    def callbackForce(self, data):

        self.data = np.insert(self.data, self.data.size, data.wrench.force.z, 1)
        self.data = np.delete(self.data, 0, axis=1)
        self.new_recording = True

        # print(self.data)

        if len(self.zero_data):
            self.zero_data.append(self.data[-1])
        if len(self.weight_data):
            self.weight_data.append(self.data[-1])

    def zeroCallback(self, data):
        self.zero_data = self.data[-1]

    def weightCallback(self, data):
        self.weight_data = self.data[-1]

    def winSizeCallback(self, data):
        self.win_size = data.data


def main():
    rospy.init_node('vaga_node', anonymous=True)

    vaga = Vaga()

    vaga_msg = Bool()

    while not rospy.is_shutdown():
        if vaga.new_recording:
            filtered = np.mean(vaga.data)
            # print(filtered)
            if filtered < vaga.zero + vaga.threshold:
                vaga_msg.data = False
                vaga.pub.publish(vaga_msg)
            elif (filtered > vaga.weight - vaga.threshold):
                vaga_msg.data = True
                vaga.pub.publish(vaga_msg)
            vaga.new_recording = False

        if len(vaga.zero_data) > vaga.win_size:
            vaga.zero = np.mean(vaga.zero_data)
            vaga.zero_data = []
        if len(vaga.weight_data) > vaga.win_size:
            vaga.weight = np.mean(vaga.weight_data)
            vaga.weight_data = []

        vaga.rate.sleep()


if __name__ == '__main__':
    main()
