import os
import csv
import math
import json
import itertools
import numpy as np

from client import config

def to_SE3(pose):
  x, y, theta = pose
  return np.array([[math.cos(theta), -math.sin(theta), 0,                    x],
                   [math.sin(theta),  math.cos(theta), 0,                    y],
                   [              0,                0, 1, -0.06447185171756116],
                   [              0,                0, 0,                    1]])


def calculate_plan_score(limits, plan):
  lower_bound_proximity = plan - limits[:, 0]
  upper_bound_proximity = limits[:, 1] - plan
  joint_angle_proximity = np.min(np.dstack((lower_bound_proximity, upper_bound_proximity)), axis=2)
  plan_score = np.min(np.min(joint_angle_proximity, axis=1))
  return plan_score


def distance_of_demo(x, y, demo):
  demo['object_poses_file']
  f = open(demo['object_poses_file'], newline='')
  demo_x, demo_y = [float(row[-1]) for row in csv.reader(f, delimiter=',')][:2]
  f.close()
  return math.sqrt((x - demo_x)**2 + (y - demo_y)**2)


def process_demos(demo_path):
  config_file = open(f'{demo_path}/config.json', 'r')
  config_dict = json.loads(config_file.read())
  config_file.close()

  dimensions = config_dict['dimensions']
  '''
  dimensions = [
    {'name': 'x',   'min': 0.6602,    'max': 1.2602,      'n_segments': 4},
    {'name': 'y',   'min': -0.185,    'max': 0.7560,      'n_segments': 4},
    {'name': 'θ',   'min':  0,        'max': 2 * math.pi, 'n_segments': 3}
  ]
  
  ∀ p ∈ (x, y, θ) i.e. dimensions ∃ p' ∈ SE(3) s.t.
  
  p: (x, y, θ) implies p': | cos(θ)  -sin(θ)  0  x |  for any arbitrary z ∈ R
                           | sin(θ)   cos(θ)  0  y |
                           |      0        0  1  z |
                           |      0        0  0  1 |
  
  which rotates points in the xy plane counter-clockwise through an angle θ w.r.t. the positive x axis
  '''

  initial_joint_config = config_dict['initial_joint_config']
  n_objects = config_dict['n_objects']
  demonstrations = [
    {
      'recorded_demo_file': f'{f}/joint_angles.csv',
      'object_poses_file': f'{f}/object_poses.csv',
      'score': calculate_plan_score(
        config.JOINT_LIMITS,
        np.array([row[1:8] for row in csv.reader(open(f'{f}/joint_angles.csv', newline=''), delimiter=',')][1:], dtype=float)
      ) if os.path.isfile(f'{f}/joint_angles.csv') else -1,
      'region_of_interest': float(open(f'{f}/region_of_interest.txt', 'r').read())
                            if os.path.isfile(f'{f}/region_of_interest.txt')
                            else 1.5  # sphere of radius equals to 1.5 times the minimum distance between object and guiding pose
    } for f in sorted([
      os.path.join(demo_path, f) for f in os.listdir(demo_path)
      if not os.path.isfile(os.path.join(demo_path, f))
    ])
  ]

  intervals = (
    [(interval[i], interval[i + 1]) for i in range(dimension['n_segments'])]
    for interval, dimension in zip(
      (np.linspace(d['min'], d['max'], d['n_segments'] + 1) for d in dimensions), dimensions
    )
  )

  segments = itertools.product(*intervals, repeat=n_objects)

  bandit_arms = {(i + 1): {'segment': segment, 'demos': []} for i, segment in enumerate(segments)}

  # arm ∈ [1, (d1 * d2 * d3)**n_objects]

  for demo in demonstrations:
    f = open(demo['object_poses_file'], newline='')
    x, y = [float(row[-1]) for row in csv.reader(f, delimiter=',')][:2]
    f.close()
    for arm in bandit_arms.values():
      (x_min, x_max), (y_min, y_max), _ = arm['segment']
      if (x_min <= x <= x_max) and (y_min <= y <= y_max):
        arm['demos'].append(demo)
        break

  return dimensions, initial_joint_config, n_objects, demonstrations, bandit_arms