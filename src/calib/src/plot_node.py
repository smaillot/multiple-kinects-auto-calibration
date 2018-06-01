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
        self.tx = x * 1000
        self.ty = y * 1000
        self.tz = z * 1000

    def update_rotation(self, x, y, z):
        self.rx = x
        self.ry = y
        self.rz = z

    def update(self, tx, ty, tz, rx, ry, rz):
        self.update_translation(tx, ty, tz)
        self.update_rotation(rx, ry, rz)

    def get(self, var):
        return (self.tx, self.ty, self.tz, self.rx, self.ry, self.rz)[var]

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
                global var
                if len(self.data_time) > 0:
                    mat = np.array(self.data_time)
                    plt.plot(mat[:, 0], mat[:, 1], label="raw")
                if len(self.data_filt_time) > 0:
                    mat_filt = np.array(self.data_filt_time)
                    plt.plot(mat_filt[:, 0], mat_filt[:, 1], "-r", label="filtered")   

                    for x in [0.68, 0.9, 0.95]:
                        col = (1, x**6, 0)
                        plt.plot(mat_filt[:, 0], self.interval(mat[:, :2], mat_filt[:, :2], mat_filt[:, 2 + (var > 2)], x)[:,0], "--", c=col, label=r'{:.0f}% conf'.format(100*x))
                        plt.plot(mat_filt[:, 0], self.interval(mat[:, :2], mat_filt[:, :2], mat_filt[:, 2 + (var > 2)], x)[:,1], "--", c=col, label='_nolegend_')

                plt.title("Evolution of " + ('tx','ty','tz','rx','ry','rz')[var])
                plt.xlabel("time (in s)")
                if var < 3:
                    plt.ylabel("Distance error (in mm)")
                else:
                    plt.ylabel("Angle error (in deg)")
                plt.legend()
                plt.draw()
                plt.pause(1e-9)
        except RuntimeError:
            pass
        else:
            # sleep(0.01)
            pass

    def get_stat(self, serie):
        return np.mean(serie), np.std(serie)
    
    def interval(self, data, mean, it, conf):
        output = np.zeros([len(mean[:, 1]), 2])
        for i in range(len(mean[:, 1])):
            t = mean[i, 0]
            s = np.std([data[k, 1] for k in range(len(data[:,1])) if data[k, 0] <= t])
            m = mean[i, 1]
            if it[i] > 0:
                output[i,:] = st.t.interval(conf, it[i], m, s)
            else:
                output[i, :] = [m, m]
        return output

def callback_tf(data):
    global tf, start
    tf.update(data.tx, data.ty, data.tz, data.rx, data.ry, data.rz)
    # rospy.loginfo(tf)
    
    global Plot, var
    x = tf.get(var)
    if var < 3:
        it = data.it_transl
    else:
        it = data.it_rot
    Plot.data_time.append([time() - start, x, it])
    Plot.display_time()
        
def callback_tf_filt(data):
    global tf, start
    tf.update(data.tx, data.ty, data.tz, data.rx, data.ry, data.rz)
    # rospy.loginfo(tf)

    global Plot, var
    x = tf.get(var)
    if var < 3:
        it = data.it_transl
    else:
        it = data.it_rot
    Plot.data_filt_time.append([time() - start, x, it])
    Plot.display_time()
        
def listener(topic):

    rospy.init_node('plot_node', anonymous=True)

    rospy.Subscriber("/calib/tf/" + topic , TF, callback_tf, queue_size=1)
    rospy.Subscriber("/calib/tf/" + topic + "_filtered", TF, callback_tf_filt, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    Plot = Plotter()
    tf = TF_msg()
    start = time()
    var = int(sys.argv[2])
    listener(sys.argv[1])