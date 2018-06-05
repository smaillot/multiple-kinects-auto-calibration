#!/usr/bin/env python
import rospy
from calib.msg import TF
import sys
import numpy as np
from matplotlib import pyplot as plt
from time import time, sleep
from multiprocessing import Lock
import scipy.stats as st
from statsmodels import robust

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

    def display(self):
        try:
            with self.mutex:
                plt.clf()
                global var, plot_type
                if plot_type == 0:
                    if len(self.data_time) > 0:
                        mat = np.array(self.data_time)
                        plt.plot(mat[:, 0], mat[:, 1 + (var % 3)], label="raw")
                        if len(self.data_filt_time) > 0:
                            mat_filt = np.array(self.data_filt_time)
                            plt.plot(mat_filt[:, 0], mat_filt[:, 1 + (var % 3)], "-r", label="filtered")   

                            for x in [0.68, 0.9, 0.95]:
                                col = (1, x**6, 0)
                                plt.plot(mat_filt[:, 0], self.interval(mat_filt[:, [0, 1 + (var%3)]], mat_filt[:, [0, 1 + (var%3)]], mat_filt[:, 4], x)[:,0], "--", c=col, label=r'{:.0f}% conf'.format(100*x))
                                plt.plot(mat_filt[:, 0], self.interval(mat_filt[:, [0, 1 + (var%3)]], mat_filt[:, [0, 1 + (var%3)]], mat_filt[:, 4], x)[:,1], "--", c=col, label='_nolegend_')

                    plt.title("Evolution of " + ('tx','ty','tz','rx','ry','rz')[var])
                    plt.xlabel("time (in s)")
                    if var < 3:
                        plt.ylabel("Distance error (in mm)")
                    else:
                        plt.ylabel("Angle error (in deg)")
                elif plot_type == 1:
                    mat = []
                    mat_filt = []
                    if len(self.data_time) > 0:
                        for i in range(3):
                            vari = 3 * (var > 2) + i
                            mat = np.array(self.data_time)[:,1 + i]
                            bins = 5+int(len(mat)/10)
                            m, s = Plot.get_stat(mat)
                            _ = plt.hist(mat, bins=bins, normed=1, color=[0.5 + 0.3 * (i == 0),0.5 + 0.3 * (i == 1),0.5 + 0.3 * (i == 2),0.6], linewidth=0, label='raw ' + ('tx','ty','tz','rx','ry','rz')[vari] + ' (mean: {:.2f}, std: {:.2f})'.format(m, s))
                        
                    if len(self.data_filt_time) > 0:
                        for i in range(3):
                            vari = 3 * (var > 2) + i
                            mat_filt = np.array(self.data_filt_time)[:,1 + i]
                            bins = 5+int(len(mat_filt)/10)
                            m, s = Plot.get_stat(mat_filt)
                            _ = plt.hist(mat_filt, bins=bins, normed=1, color=[0.3 * (i == 0),0.3 * (i == 1),0.3 * (i == 2),0.8], linewidth=0, label='filtered ' + ('tx','ty','tz','rx','ry','rz')[vari] + ' (mean: {:.2f}, std: {:.2f})'.format(m, s))
                    plt.xlim(([-20, 20], [-10, 10])[var > 2])
                    plt.title(("Translation", "Rotation")[var > 2] + ' error, it: ' + str(len(mat)) + ', ' + str(len(mat_filt)))
                    plt.xlabel(("Distance (in mm)", "Angle (in deg)")[var > 2])
                    plt.ylabel("Samples (normed)")

                elif plot_type == 2:
                    mat = []
                    mat_filt = []
                    ind = range(3)
                    if len(self.data_filt_time) > 0:
                        for i in ind:
                            vari = 3 * (var > 2) + i
                            mat = np.array(self.data_filt_time)[:,1 + i]
                            bins = 5+int(len(mat)/10)
                            m, s = Plot.get_stat(mat)
                            mad = robust.mad(mat)
                            _ = plt.bar(i, s, color=[0.3 * (i == 0),0.3 * (i == 1),0.3 * (i == 2),1], label='filtered ' + ('tx','ty','tz','rx','ry','rz')[vari] + ' (mean: {:.2f}, std: {:.2f})'.format(m, s))
                            # _ = plt.bar(3*i+1, mad, color=[0.25+0.3 * (i == 0),0.25+0.3 * (i == 1),0.25+0.3 * (i == 2),1], label='filtered ' + ('tx','ty','tz','rx','ry','rz')[vari] + ' (mean: {:.2f}, std: {:.2f})'.format(m, s))
                            # _ = plt.bar(3*i+2, m, color=[0.5+0.3 * (i == 0),0.5+0.3 * (i == 1),0.5+0.3 * (i == 2),1], label='filtered ' + ('tx','ty','tz','rx','ry','rz')[vari] + ' (mean: {:.2f}, std: {:.2f})'.format(m, s))
                    
                    # plt.yscale("log")
                    plt.title(("Translation", "Rotation")[var > 2] + ' standard deviation, it: ' + str(len(mat)) + ', ' + str(len(mat_filt)))
                    plt.ylabel("mean absolute deviation (in mm)")
                    plt.ylim([0, 30])
                    
                
                # plt.legend()
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
    
    global Plot, var, plot_type
    x = tf.get(0 + 3 * (var > 2))
    y = tf.get(1 + 3 * (var > 2))
    z = tf.get(2 + 3 * (var > 2))
    if var < 3:
        it = data.it_transl
    else:
        it = data.it_rot
    Plot.data_time.append([time() - start, x, y, z, it])
    Plot.display()
        
def callback_tf_filt(data):
    global tf, start
    tf.update(data.tx, data.ty, data.tz, data.rx, data.ry, data.rz)
    # rospy.loginfo(tf)

    global Plot, var
    x = tf.get(0 + 3 * (var > 2))
    y = tf.get(1 + 3 * (var > 2))
    z = tf.get(2 + 3 * (var > 2))
    if var < 3:
        it = data.it_transl
    else:
        it = data.it_rot
    Plot.data_filt_time.append([time() - start, x, y, z, it])
    Plot.display()

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
    plot_type = int(sys.argv[3])
    listener(sys.argv[1])
