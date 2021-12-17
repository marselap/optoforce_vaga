#!/usr/bin/env python
from __future__ import print_function
import sys
import numpy as np
import rospy
from std_msgs.msg import String, Bool, Int32, Float32
import time
from geometry_msgs.msg import WrenchStamped
import pickle


class Vaga:

    def __init__(self):


        rate_hz = 100.
        self.rate = rospy.Rate(rate_hz)
        
        self.win_size = 5
        self.zero = 0.
        self.weight = 0.5
        self.threshold = 0.1

        self.state = False # false = empty, true = full

        self.data = {}
        for keys in ['0', '1', '2']:
            self.data[keys] = np.zeros((1,self.win_size))
        # self.data = np.zeros((1,self.win_size))

        self.zero_data = []
        self.weight_data = []

        self.opto_sub_0 = rospy.Subscriber("/f0", WrenchStamped, self.callbackForce_0, queue_size = 1)
        self.opto_sub_1 = rospy.Subscriber("/f1", WrenchStamped, self.callbackForce_1, queue_size = 1)
        self.opto_sub_2 = rospy.Subscriber("/f2", WrenchStamped, self.callbackForce_2, queue_size = 1)

        self.reset_zero = rospy.Subscriber("/zero_now", Bool, self.zeroCallback)

        #set weight by measuring
        self.reset_weight = rospy.Subscriber("/weight_now", Bool, self.weightCallback)

        self.win_size = rospy.Subscriber("/window_size", Int32, self.winSizeCallback)

        self.pub = rospy.Publisher("/vaga_puna", Bool, queue_size = 1)

        #set weight manually
        self.cookieweight = rospy.Subscriber("/cookie_weight", Float32, self.cookieCallback)



    def callbackForce_0(self, data):
        self.callbackForce('0', data.wrench.force.z)
    def callbackForce_1(self, data):
        self.callbackForce('1', data.wrench.force.z)
    def callbackForce_2(self, data):
        self.callbackForce('2', data.wrench.force.z)


    def callbackForce(self, sensorkey, data):

        self.data[sensorkey] = np.insert(self.data[sensorkey], self.data[sensorkey].size, data, 1)
        self.data[sensorkey] = np.delete(self.data[sensorkey], 0, axis=1)

        if len(self.zero_data):
            self.zero_data.append(data)
        if len(self.weight_data):
            self.weight_data.append(data)

    def zeroCallback(self, data):
        self.zero_data = [self.data[keys][-1] for keys in self.data.keys()]
        # self.zero_data = self.data[-1]

    def weightCallback(self, data):
        self.weight_data = [self.data[keys][-1] for keys in self.data.keys()]

    def cookieCallback(self, data):
        self.weight = data.data

    def winSizeCallback(self, data):
        print(data.data)
        self.win_size = data.data
        for keys in self.data.keys():
            self.data[keys] = self.data[keys][0:self.win_size]


def main():
    rospy.init_node('vaga_node', anonymous=True)

    vaga = Vaga()

    vaga_msg = Bool()

    avg_i = {}

    while not rospy.is_shutdown():
    

        filtered = 0.
        for keys in vaga.data.keys():
            avg_i[keys] = np.mean(vaga.data[keys])
            filtered += avg_i[keys]

        # filtered /= len(vaga.data.keys())
        # print(filtered)
        if filtered < vaga.zero + vaga.threshold:
            vaga_msg.data = False
            vaga.pub.publish(vaga_msg)
        elif (filtered > vaga.weight - vaga.threshold):
            vaga_msg.data = True
            vaga.pub.publish(vaga_msg)


        if len(vaga.zero_data) > vaga.win_size:
            vaga.zero = np.mean(vaga.zero_data)
            vaga.zero_data = []
        if len(vaga.weight_data) > vaga.win_size:
            vaga.weight = np.mean(vaga.weight_data)
            vaga.weight_data = []

        vaga.rate.sleep()


if __name__ == '__main__':
    main()
