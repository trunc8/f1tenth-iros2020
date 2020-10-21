#!/usr/bin/env python

# trunc8 did this

import yaml
import json

def get_global_waypoints():
  with open(r'waypoints2.yaml') as file:
    list_of_waypoints = yaml.load(file)
  print(list_of_waypoints["markers"][5]["ns"])
  return list_of_waypoints["markers"][5]["points"]

if __name__ == '__main__':
  #print(get_global_waypoints())
  list_of_waypoints = get_global_waypoints()
  print(len(list_of_waypoints))
  json_file_path = 'json_waypoints2.json'
  with open(json_file_path, 'w') as fout:
    json.dump(list_of_waypoints, fout)

  with open(json_file_path, 'r') as read_file:
    data = json.load(read_file)
  
  print(len(data))
