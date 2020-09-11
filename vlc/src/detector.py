import numpy as np
import os
import rosbag
import matplotlib.pylab as plt
import copy
import sys
import tqdm
import time
import math
import bisect
from sklearn.cluster import KMeans


def get_estimated_pos_from_evidence_map(evidence_map, thres_rate=0.8):
    xs, ys = np.where(np.array(evidence_map) > np.max(evidence_map)*thres_rate)
    est_pos = [np.mean(xs), np.mean(ys)]
    return est_pos

def prob(dt, freq, sigma=30, n_tol=1):
    gauss = lambda x : math.exp(-x**2/2/sigma**2) / math.sqrt(2*math.pi*sigma*sigma)
    res = 0
    for i in range(n_tol):
        res += gauss(1.0*i/dt - freq)
    return res

def calc_evidence_maps(event_buffer, hz, t_at, img_size=[1280, 720], event_time_list=None, n_tol=1, sigma=30):
    start = time.time()
    last_event_polarity = [[-1 for _ in range(img_size[1])] for _ in range(img_size[0])]
    last_trans_time = [[0 for _ in range(img_size[1])] for _ in range(img_size[0])]
    evidence_maps = {}
    for freq in hz:
        evidence_maps[freq] = [[0 for _ in range(img_size[1])] for _ in range(img_size[0])]
        
    n = 5
    # gauss = lambda x : math.exp(-x**2/2/sigma**2) / math.sqrt(2*math.pi*sigma*sigma)
    
    if event_time_list is not None:
        min_index = bisect.bisect_left(event_time_list, event_time_list[-1] - min([n * 1.0/freq for freq in hz]))

    for event in event_buffer[min_index:]:
        x, y, t, p = int(event[0]), int(event[1]), event[2], int(event[3])
        if last_event_polarity[x][y] == -1:
            last_event_polarity[x][y] = p*1
        elif last_event_polarity[x][y] != p*1:
            # Update evidence_map
            dt = t - last_trans_time[x][y] 
            for freq in hz:
                if t_at - n * 1.0/freq < t and t < t_at and dt!=0:
                    evidence_maps[freq][x][y] += prob(dt, freq, n_tol=n_tol, sigma=sigma)
            last_trans_time[x][y] = t
        last_event_polarity[x][y] = p*1
    return evidence_maps

def estimate_led_positions(event_buffer, hz, t_at, img_size=[1280, 720], event_time_list=None, n_tol=1, sigma=30, evd_tol_rate=0.2):
    evidence_maps = calc_evidence_maps(event_buffer, hz, t_at, img_size=img_size, event_time_list=event_time_list, n_tol=n_tol, sigma=sigma)
    
    estimated_poss = {}
    for freq in hz:
        estimated_poss[freq] = get_estimated_pos_from_evidence_map(evidence_maps[freq], thres_rate=0.9)
    return estimated_poss, evidence_maps

def estimate_led_positions_kmeans(event_buffer, fi, n_led, t_at, img_size=[1280, 720], event_time_list=None, n_tol=1, sigma=30, evd_tol_rate=0.2):
    evidence_maps = calc_evidence_maps(event_buffer, [fi], t_at, img_size=img_size, event_time_list=event_time_list, n_tol=n_tol, sigma=sigma)
    
    high_evidence_points = np.where(np.array(evidence_maps[fi]).T > np.max(evidence_maps[fi])*evd_tol_rate)
    high_evidence_points = np.array([high_evidence_points[0], high_evidence_points[1]]).T
    
    kmeans_model = KMeans(n_clusters=n_led, random_state=10).fit(high_evidence_points)
    labels = kmeans_model.labels_
    
    estimated_poss = {}
    for i in range(n_led):
        pos = high_evidence_points[labels==i].mean(axis=0)
        estimated_poss[i] = [pos[1], pos[0]]

    return estimated_poss, evidence_maps

def get_event_list_from_bag(bag, extract_time):
    event_list = [] # x, y, t, p
    event_time_list = [] # t

    timestamp_list = []
    start_time = -1
    for topic, msg, t in tqdm.tqdm(bag):
        if topic=='/prophesee/camera/cd_events_buffer':
            if start_time<0:
                start_time = msg.events[0].ts.to_time()
            if msg.events[0].ts.to_time() - start_time > extract_time:
                break
            for event in msg.events:
                if msg.events[0].ts.to_time() - start_time > extract_time:
                    break
                event_list.append([int(event.x), int(event.y), event.ts.to_time() - start_time, event.polarity])
                event_time_list.append(event.ts.to_time() - start_time)
    return np.array(event_list), event_time_list

if __name__=='__main__':
    bagpath = '/home/koji/dvs/vlc/data/led_500_700hz_light.bag'
    bag = rosbag.Bag(bagpath)
    event_list, event_time_list = get_event_list_from_bag(bag)
    estimated_poss, evidence_maps = estimate_led_positions(event_list, [500, 700], 0.5, time_list=event_time_list)
    
    fi = 500
    plt.imshow(np.array(evidence_maps[fi]).T)
    plt.scatter(estimated_poss[fi][0], estimated_poss[fi][1], color='white')
    plt.show()