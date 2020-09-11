import numpy as np
import matplotlib.pylab as plt
import sys
import os
import rosbag
import cv2
sys.path.append('/home/koji/dvs/vlc/src')
from detector import get_event_list_from_bag, estimate_led_positions_kmeans
from encoder import encoder
import argparse


def load_calibration(filepath='/home/koji/dvs/calibration/data'):
    matrix = np.loadtxt(os.path.join(filepath, 'matrix.txt'))
    dist = np.loadtxt(os.path.join(filepath, 'dist.txt'))
    return matrix, dist
K, dist = load_calibration()

def localize(ps, Pws, K, dist, method='xyz'):
    """
    Parameters:
    ------
    ps: numpy.array (shape: [n, 2])
        2D positions of markers in pixel
    Pws: 
        3D positions of markers in world frame
    K:
        calibration matrix
    dist:
        distortion parameters
    method:
        [x, xyz, xyyaw]
    """
    R_test = np.array([
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0]
    ])
    R = np.zeros((3, 3))
    T = np.zeros((3, 1))
    ps_undistort = cv2.undistortPoints(np.array([ps]).astype('float32'), K, dist, P=K)[0]
    ps_undistort
    Kinv = np.linalg.inv(K)
    if method=='xyz':
        u1 = np.hstack([ps_undistort[0], 1]).reshape([-1, 1])
        u2 = np.hstack([ps_undistort[1], 1]).reshape([-1, 1])
        A = np.block([[np.eye(3), -np.dot(Kinv, u1), np.zeros((3, 1))], [np.eye(3), np.zeros((3, 1)), -np.dot(Kinv, u2)]])
        b = - Pws.reshape([-1, 1])
        return np.eye(3), np.dot(np.linalg.pinv(A), b)[:3]
    elif method=='xy':
        u = np.hstack([ps_undistort[0], 1]).reshape([-1, 1])
        A = np.block([np.array([[1, 0], [0, 1], [0, 0]]), -np.dot(Kinv, u)])
        b = - Pws[0].reshape([-1, 1])
        
#         b = np.dot(R_test, b)
        
        ans = np.dot(np.linalg.inv(A), b)[:3]
        T[0, 0] = ans[0, 0]
        T[1, 0] = ans[1, 0]
        return np.eye(3), T
    elif method=='x':
        u = np.hstack([ps_undistort[0], 1]).reshape([-1, 1])
        A = np.block([np.array([1, 0, 0]).reshape((-1, 1)), -np.dot(Kinv, u)])
        b = - Pws[0].reshape([-1, 1])
        ans = np.dot(np.linalg.pinv(A), b)[:3]
        T[0, 0] = ans[0, 0]
        return np.eye(3), T
    elif method=='y':
        u = np.hstack([ps_undistort[0], 1]).reshape([-1, 1])
        A = np.block([np.array([0, 1, 0]).reshape((-1, 1)), -np.dot(Kinv, u)])
        b = - Pws[0].reshape([-1, 1])
        ans = np.dot(np.linalg.pinv(A), b)[:3]
        T[1, 0] = ans[0, 0]
        return np.eye(3), T
    elif method=='zx':
        u = np.hstack([ps_undistort[0], 1]).reshape([-1, 1])
        A = np.block([np.array([[0, 1], [0, 0], [1, 0]]), -np.dot(Kinv, u)])
        b = - Pws[0].reshape([-1, 1])        
        ans = np.dot(np.linalg.inv(A), b)[:3]
        T[2, 0] = ans[0, 0]
        T[0, 0] = ans[1, 0]
        return np.eye(3), T
    
if __name__=='__main__':
