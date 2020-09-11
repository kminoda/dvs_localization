import numpy as np
import matplotlib.pylab as plt
import sys
import os
import subprocess
import rosbag
import time
import json
import cv2
WORKDIR_PATH = '/home/koji/git/dvs_localziation/'
sys.path.append(WORKDIR_PATH+'/vlc/src')
from detector import get_event_list_from_bag, estimate_led_positions_kmeans
from encoder import encoder
from recorder import record
import argparse


def load_calibration(filepath=WORKDIR_PATH+'/calibration/data'):
    matrix = np.loadtxt(os.path.join(filepath, 'matrix.txt'))
    dist = np.loadtxt(os.path.join(filepath, 'dist.txt'))
    return matrix, dist
K, dist = load_calibration()

def solve_pnp(ps, Pws, K, dist, method='xyz'):
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
    Kinv = np.linalg.inv(K)
    if method=='xyzyaw':
        u1 = np.hstack([ps_undistort[0], 1]).reshape([-1, 1])
        u2 = np.hstack([ps_undistort[1], 1]).reshape([-1, 1])
        d_theta = np.array([
            [0, -1, 0], 
            [1, 0, 0],
            [0, 0, 0]
        ])
        A = np.block([
            [np.eye(3), np.dot(d_theta, Pws[0].reshape([-1, 1])), -np.dot(Kinv, u1), np.zeros((3, 1))], 
            [np.eye(3), np.dot(d_theta, Pws[1].reshape([-1, 1])), np.zeros((3, 1)), -np.dot(Kinv, u2)]
        ])
        b = - Pws.reshape([-1, 1])
        ans = np.dot(np.linalg.pinv(A), b)
        R = np.array([
            [np.cos(ans[3]), -np.sin(ans[3]), 0],
            [np.sin(ans[3]), np.cos(ans[3]), 0],
            [0, 0, 1]
        ])
        print("A:", A)
        print("b:", b)
        print("ans:", ans)
        T = ans[:3]
        return R, T, ans
    elif method=='xyz':
        u1 = np.hstack([ps_undistort[0], 1]).reshape([-1, 1])
        u2 = np.hstack([ps_undistort[1], 1]).reshape([-1, 1])
        A = np.block([
            [np.eye(3), -np.dot(Kinv, u1), np.zeros((3, 1))], 
            [np.eye(3), np.zeros((3, 1)), -np.dot(Kinv, u2)]
        ])
        b = - Pws.reshape([-1, 1])
        ans = np.dot(np.linalg.pinv(A), b)
        return np.eye(3), ans[:3], ans
    elif method=='xy':
        u = np.hstack([ps_undistort[0], 1]).reshape([-1, 1])
        A = np.block([np.array([[1, 0], [0, 1], [0, 0]]), -np.dot(Kinv, u)])
        b = - Pws[0].reshape([-1, 1])
                
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
    
def localize_from_rosbag(rosbagpath, config):
    event_list, event_time_list = get_event_list_from_bag(rosbag.Bag(rosbagpath), config['t_at'])
    start = time.time()
    R, T = localize(event_list, event_time_list, config)
    end = time.time()
    print('time: {}[s]'.format(end - start))
    return R, T

def localize(event_list, event_time_list, config):
    estimated_pos_dict, evidence_map_dict = estimate_led_positions_kmeans(
        event_list, 
        config['freq'], 
        n_led=config['n_led'],
        t_at=config['t_at'],
        event_time_list=event_time_list, 
        n_tol=1, 
        sigma=30
    )
    
    encoded_msgs = encoder(
        event_list, 
        estimated_pos_dict, 
        config['freq'], 
        config['thres_detect_led'], 
        t_begin=0.0, 
        t_end=config['t_at']
    )
    ps = np.zeros((2, 2))
    if config['debug']:
        plt.imshow(np.zeros((720, 1280)))

    for key in encoded_msgs.keys():
        pos = estimated_pos_dict[key]
        msg = encoded_msgs[key]
        print('pos:{0}, msg:{1}'.format([int(pos[0]), int(pos[1])], msg))
        if msg == [True, True, False, False, True, True]:
            clr = 'yellow'
            label = 'yellow LED, Origin'
            ps[1] = pos
        elif msg == [True, True, True, True, False, False]:
            clr = 'red'
            label = 'red LED'
            ps[0] = pos
        else:
            clr = 'white'
            label = 'invalid'
        if config['debug']:
            plt.scatter(pos[0], pos[1], color=clr, label=label)
    if config['debug']:
        plt.legend()
        plt.show()

    K, dist = load_calibration()

    Pws = np.array(config['led_pos']) # [m?]
    R, T, ans = solve_pnp(ps, Pws, K, dist, method=config['method'])
    if config['method'] in ['xyz', 'xyzyaw']:
        update_ceres_input(ps, Pws, K, dist, ans)
    return R, T

def update_ceres_input(ps, Pws, K, dist, ans):
    ps_undistort = cv2.undistortPoints(np.array([ps]).astype('float32'), K, dist, P=K)[0]
    with open(WORKDIR_PATH+'localization/data/ceres_input/markers.txt', 'w') as f:
        for i in range(len(ps)):
            f.write('{0} {1} {2} {3} {4}\n'.format(ps_undistort[i][0], ps_undistort[i][1], Pws[i][0], Pws[i][1], Pws[i][2]))
    with open(WORKDIR_PATH+'localization/data/ceres_input/initial.txt', 'w') as f:
        if len(ans)==5:
            # x, y, z, l1, l2
            f.write('{0}\n{1} {2} {3} {4} {5}\n'.format(0, ans[0][0], ans[1][0], ans[2][0], ans[3][0], ans[4][0]))
        elif len(ans)==6:
            # x, y, z, yaw, l1, l2
            f.write('{0}\n{1} {2} {3} {4} {5} {6}\n'.format(1, ans[0][0], ans[1][0], ans[2][0], ans[3][0], ans[4][0], ans[5][0]))
    

if __name__=='__main__':
    parser = argparse.ArgumentParser() 
    parser.add_argument('--rosbag', help='rosbag', type=str)
    parser.add_argument('--config', help='config', type=str, default=WORKDIR_PATH+'localization/cfg/config.json')
    parser.add_argument('--exec_ceres', action='store_true', help='execute nonlinear optimization with ceres in c++')
    args = parser.parse_args()

    with open('../cfg/config.json') as f:
        config = json.load(f)
    record(args.rosbag, config['t_at']*2)
    R, T = localize_from_rosbag(args.rosbag, config)
    print('Translation: {}'.format(T))
    # print('Rotation: {}'.format(R))
    print('Yaw: {} [deg]'.format(np.rad2deg(np.arcsin(R[0][1]))[0]))

    if args.exec_ceres:
        subprocess.call(WORKDIR_PATH+'localization/src/ceres_optim/build/localize')

