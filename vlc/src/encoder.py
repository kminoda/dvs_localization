import numpy as np
import os
import rosbag
import matplotlib.pylab as plt
import copy
import sys
import tqdm
import time

sys.path.append('~/git/dvs_localization/vlc/src')
from encoder import get_event_list_from_bag, estimate_led_positions, estimate_led_positions_kmeans

import argparse

def get_hist_and_time(event_list, led_pos, t_begin=0., t_end=0.5, led_r = 40, debug=False):
    x_min_cond = led_pos[0]-led_r < event_list[:, 0]
    x_max_cond = led_pos[0]+led_r > event_list[:, 0]
    y_min_cond = led_pos[1]-led_r < event_list[:, 1]
    y_max_cond = led_pos[1]+led_r > event_list[:, 1]
    event_list_freq = event_list[np.where(x_min_cond & x_max_cond & y_min_cond & y_max_cond)[0]].tolist()
    
    time_list = []
    polarity_list = []
    start_t = -1
    for event in event_list_freq:
        if start_t < 0:
            start_t = event[2]
        if event[2] - start_t < t_begin:
            continue
        time_list.append(event[2] - start_t)
        polarity_list.append(event[3])
        if event[2] - start_t > t_end:
            break
    time_list = np.array(time_list)
    polarity_list = np.array(polarity_list)
    
    # Very heuristic!!!!
    n_bins = int((t_end - t_begin) * 2000)
    
    range_hist = (np.min(time_list), np.max(time_list))
    pos = np.histogram(time_list[polarity_list==1], bins=n_bins, range=range_hist)
    neg = np.histogram(time_list[polarity_list==0], bins=n_bins, range=range_hist)
    time_bins = pos[1]
    print(time_bins)
    hist_datas = [pos[0], neg[0]]

    if debug:
        plt.figure(figsize=(20, 4))
        hoge = plt.hist([time_list[polarity_list==1], time_list[polarity_list==0]], bins=n_bins, label=['positive', 'negative'])
        # time_bins = hoge[1]
        # hist_datas = hoge[0]
        plt.xlabel('time [s]', fontsize=12)
        plt.xticks(fontsize=12)
        plt.ylabel('# of events', fontsize=12)
        plt.yticks(fontsize=12)
        plt.legend()

    return time_bins, hist_datas

def get_edges(hist_data, time_bins, thres=50):
    edges = []
    flag = False
    for time, val in zip(time_bins, hist_data):
        if val > thres and not flag:
            edges.append(time)
            flag = True
        elif val < thres and flag:
            flag = False
    return edges

def get_bits_from_edges(edges, freq):
    """
    edges: list of detected edge timestamps
    """
    tol_rate = 0.9
    bits = [True]
    cur_time = edges[0]
    cur_idx = 1
    while True:
        cur_time += 1.0/freq
        if edges[cur_idx] > cur_time - 0.5*tol_rate/freq and edges[cur_idx] < cur_time + 0.5*tol_rate/freq:
            bits.append(True)
            cur_idx += 1
        else:
            bits.append(False)
        if cur_idx >= len(edges) or cur_time > edges[-1]:
            break
    return bits

def get_sync_idx(bits, sync):
    n_sync = len(sync)
    n_bits = len(bits)
    sync_idx = []
    for i in range(n_bits - n_sync):
        is_sync = True
        for j in range(n_sync):
            is_sync = is_sync and (bits[i+j]*1==sync[j])
        if is_sync:
            sync_idx.append(i)
    return sync_idx

def encode_msg(bits, sync=[0, 0, 1, 0, 1, 1, 0], len_msg=6):
    sync_idx = get_sync_idx(bits, sync)
    n_sync = len(sync)
    for idx in sync_idx:
        msg = bits[idx+n_sync : idx+n_sync+len_msg]
        parity = bits[idx + n_sync + len_msg]
        if (sum(msg)+sum(sync))%2==parity:
            print('Detected the correct msg')
            return msg
        else:
            print('Error detected!!')

def encoder(event_list, led_pos_dict, freq, thres_detect_led, t_begin=0, t_end=0.5, debug=False):
    encoded_msgs = {}
    for key, val in led_pos_dict.items():
        time_bins, hist_datas = get_hist_and_time(event_list, val, t_begin=t_begin, t_end=t_end, debug=debug)
        edges = get_edges(hist_datas[0], time_bins, thres=thres_detect_led)
        bits = get_bits_from_edges(edges, freq)
        msg = encode_msg(bits)
        encoded_msgs[key] = msg
    return encoded_msgs

if __name__=='__main__':
    parser = argparse.ArgumentParser() 
    parser.add_argument('--rosbag', help='rosbag', type=str, default='/home/koji/dvs/vlc/data/vlc_slow_100_100hz_00101101111001_00101101101110_noise.bag')
    parser.add_argument('--freq', help='frequency', type=float, default=100)
    parser.add_argument('--n_led', help='number of led', type=int, default=2)
    parser.add_argument('--t_at', help='time to extract', type=float, default=0.5)
    parser.add_argument('--thres_detect_led', help='tthreshold for led', type=float, default=100)
    parser.add_argument('--debug', help='time to extract', action='store_true')
    args = parser.parse_args()

    if args.debug:
        print('Debug mode')

    event_list, event_time_list = get_event_list_from_bag(rosbag.Bag(args.rosbag), args.t_at)
    estimated_pos_dict, evidence_map_dict = estimate_led_positions_kmeans(
        event_list, 
        args.freq, 
        n_led=args.n_led,
        t_at=args.t_at,
        event_time_list=event_time_list, 
        n_tol=1, 
        sigma=30
    )
    
    encoded_msgs = encoder(event_list, estimated_pos_dict, args.freq, args.thres_detect_led, t_begin=0.0, t_end=args.t_at, debug=args.debug)
    for key in encoded_msgs.keys():
        pos = estimated_pos_dict[key]
        msg = encoded_msgs[key]
        print('pos:{0}, msg:{1}'.format([int(pos[0]), int(pos[1])], msg))