#!/usr/bin/env python
import rospy
from calib.msg import TF
import sys
import numpy as np
from matplotlib import pyplot as plt
import tf.transformations as t
import geometry_msgs.msg

def qv_mult(q1, v1):
    v1 = tf.transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2), 
        tf.transformations.quaternion_conjugate(q1)
    )[:3]

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

    def get(self, var):
        return (self.tx, self.ty, self.tz, self.rx, self.ry, self.rz)[var]

    def __repr__(self):
        return "TF message\n\ttranslation: {}, {}, {} (in m)\n\trotation: {}, {}, {} (in deg)".format(self.tx, self.ty, self.tz, self.rx, self.ry, self.rz)

    def valid_translation(self):
        valid = True
        if (np.abs(self.tx) < 1e-5):
            valid = False
        if (np.abs(self.ty) < 1e-5):
            valid = False
        if (np.abs(self.tz) < 1e-5):
            valid = False
        return valid

    def valid_rotation(self):
        valid = True
        if np.mod((np.abs(self.rx, np.pi)) < 1e-5):
            valid = False
        if (np.mod(np.abs(self.ry), np.pi) < 1e-5):
            valid = False
        if (np.mod(np.abs(self.rz), np.pi) < 1e-5):
            valid = False
        return valid

    def valid(self):
        return valid_rotation() and valid_translation()

def callback_topic(data):
    global gt
    tf.update(data.tx, data.ty, data.tz, data.rx, data.ry, data.rz)
    q_topic = t.quaternion_from_euler(data.rx, data.ry, data.rz)
    q_gt = t.quaternion_from_euler(gt[3], gt[4], gt[5])
    q = t.quaternion_multiply(q_topic, t.quaternion_inverse(q_gt))
    t = np.array([data.tx, data.ty, data.tz] - qv_mult(t.quaternion_inverse(q_gt), [gt[0], gt[1], gt[2]]))

def callback_icp1(data):
    global tf
    tf.update(data.tx, data.ty, data.tz, data.rx, data.ry, data.rz)

def callback_icp2(data):
    

def listener(topic):

    rospy.init_node('plot_node', anonymous=True)

    if topic == "icp":
        rospy.Subscriber("/calib/tf/cam1_icp", TF, callback_icp1, queue_size=1)
        rospy.Subscriber("/calib/tf/cam2_icp", TF, callback_icp2, queue_size=1)
    else:
        rospy.Subscriber("/calib/tf/" + topic, TF, callback_topic, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    tf = TF_msg()
    gt = [0 0 0 0 0 0]
    listener(argv[1])
