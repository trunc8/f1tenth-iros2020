#!/usr/bin/env python

# trunc8 did this

import json

import numpy as np

if __name__ == '__main__':
    list_of_waypoints = []
    json_file_path1 = 'json_waypoints1.json'
    with open(json_file_path1, 'r') as read_file:
        list_of_waypoints1 = json.load(read_file)
    json_file_path2 = 'json_waypoints2.json'
    with open(json_file_path2, 'r') as read_file:
        list_of_waypoints2 = json.load(read_file)
    list_of_waypoints.extend(list_of_waypoints1)
    print(len(list_of_waypoints))
    list_of_waypoints.extend(list_of_waypoints2)
    print(len(list_of_waypoints))

    x = [i['x'] for i in list_of_waypoints]
    y = [i['y'] for i in list_of_waypoints]
    psi = [i['z'] for i in list_of_waypoints]

    waypoints = np.concatenate((x, y, psi)).reshape((3, -1)).T

    print('Saving waypoints...')
    with open('localPlanner/waypoints.npy', 'wb') as f:
        np.save(f, waypoints)

    print('Testing save...')
    with open('localPlanner/waypoints.npy', 'rb') as f:
        test = np.load(f)

    if np.all(test==waypoints):
        print('Save successful!')
    else:
        print('[!] Save unsuccessful')
