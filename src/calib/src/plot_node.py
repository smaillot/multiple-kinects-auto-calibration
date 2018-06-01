#!/usr/bin/env python
import rospy
from calib.msg import TF
import sys
import numpy as np
from matplotlib import pyplot as plt
from time import time, sleep
from multiprocessing import Lock
import scipy.stats as st

class TF_msg():

    def __init__(self):
        self.tx = 0
        self.ty = 0
        self.tz = 0
        self.rx = 0
        self.ry = 0
        self.rz = 0

    def update_translation(self, x, y, z):
        self.tx = x
        self.ty = y
        self.tz = z

    def update_rotation(self, x, y, z):
        self.rx = x
        self.ry = y
        self.rz = z

    def update(self, tx, ty, tz, rx, ry, rz):
        self.update_translation(tx, ty, tz)
        self.update_rotation(rx, ry, rz)

    def __repr__(self):
        return "TF message\n\ttranslation: {}, {}, {} (in m)\n\trotation: {}, {}, {} (in deg)".format(self.tx, self.ty, self.tz, self.rx, self.ry, self.rz)

class Plotter():

    def __init__(self):
        self.mutex = Lock()

        self.data_time = []
        self.data_filt_time = []

        plt.ion()
        plt.show()

    def display_time(self):
        try:
            with self.mutex:
                plt.clf()
                if len(self.data_time) > 0:
                    mat = np.array(self.data_time)
                    plt.plot(mat[:, 0], mat[:, 1])
                if len(self.data_filt_time) > 0:
                    mat_filt = np.array(self.data_filt_time)
                    plt.plot(mat_filt[:, 0], mat_filt[:, 1], "-r", label="filtered")
                    x = 0.68
                    col = (1, x**8, 0)
                    plt.plot(mat_filt[:, 0], self.interval(mat_filt[:, 1], x)[:,0], "--", c=col, label=r'{:.1f}\%'.format(100*x))
                    plt.plot(mat_filt[:, 0], self.interval(mat_filt[:, 1], x)[:,1], "--", c=col, label='_nolegend_')
                plt.draw()
                plt.pause(1e-9)
        except RuntimeError:
            pass
        else:
            sleep(0.01)

    def get_stat(self, serie):
        return np.mean(serie), np.std(serie)
    
    def interval(self, data, conf):
        output = np.zeros([len(data), 2])
        for i in range(len(data)):
            m, s = self.get_stat(data[:i+1])
            output[i,:] = st.t.interval(conf, i+1, m, s)
        return output

def callback_tf(data):
    global tf
    tf.update(data.tx, data.ty, data.tz, data.rx, data.ry, data.rz)
    # rospy.loginfo(tf)
    
    global Plot
    Plot.data_time.append([time(), tf.ty])
    Plot.display_time()
        
def callback_tf_filt(data):
    global tf
    tf.update(data.tx, data.ty, data.tz, data.rx, data.ry, data.rz)
    # rospy.loginfo(tf)

    global Plot
    Plot.data_filt_time.append([time(), tf.ty])
    Plot.display_time()
        
def listener(topic):

    rospy.init_node('plot_node', anonymous=True)

    rospy.Subscriber("/calib/tf/" + topic , TF, callback_tf)
    rospy.Subscriber("/calib/tf/" + topic + "_filtered", TF, callback_tf_filt)

    rospy.spin()

if __name__ == '__main__':
    Plot = Plotter()
    tf = TF_msg()
    listener(sys.argv[1])