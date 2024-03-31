import math
import copy
import numpy as np

from client import plot, utils, config, planner




def evaluate_arm(arm_id, arms, demontrations, n_objects, n_dimensions, initial_joint_config, n_task_instances, reuse_tasks_instances):
  arm = arms[arm_id]

  metadata = {}

  if reuse_tasks_instances:
    if 'task_instances' not in metadata:
      low, high = zip(*arm['segment'])
      metadata['success'] = np.empty((0, n_objects, n_dimensions))
      metadata['task_instances'] = task_instances = np.random.uniform(
        low=low, high=high,
        size=(n_task_instances, n_dimensions)
      ).reshape(n_task_instances, n_objects, n_dimensions)
    elif len(metadata['failure_score']) == 0:
      return
    else:
      task_instances = metadata['failure']
  else:
    low, high = zip(*arm['segment'])
    task_instances = np.random.uniform(
      low=low, high=high,
      size=(n_task_instances, n_dimensions)
    ).reshape(n_task_instances, n_objects, n_dimensions)

  motion_plans = planner.remote_planner(
    config.REMOTE_SERVER,
    [demontrations[-1]] if reuse_tasks_instances else sorted(demontrations, key=lambda d: d['score'], reverse=True),
    initial_joint_config,
    np.apply_along_axis(utils.to_SE3, 2, task_instances)
  )

  successful_indices, failed_indices, scores = [], [], []
  for i, plans in enumerate(motion_plans):
    if plans[-1]['is_successful']:
      successful_indices.append(i)
    else:
      failed_indices.append(i)
      scores.append(np.mean([p['failed_screw_segment'] / p['n_screw_segments'] for p in plans]))

  print(f'Failed samples: {len(failed_indices)}/{n_task_instances}')

  metadata['success'] = np.vstack(
    (metadata['success'], task_instances[successful_indices])
  ) if reuse_tasks_instances else task_instances[successful_indices]
  metadata['failure'] = task_instances[failed_indices]
  metadata['failure_score'] = scores

  return metadata



# this algorithm gives ε-optimal arm with probability 1 − δ
# each arm must be sampled at least 1/2ε² * ln(2K/δ) times
# both ε and δ should be small numbers
def naive_pac(arms, demontrations, n_objects, n_dimensions, initial_joint_config, epsilon=0.25, delta=0.1, reuse_tasks_instances=False):
  n_arms = len(arms)
  n_samples_per_arm = math.floor(math.log(2 * n_arms / delta) / (2 * epsilon**2))

  samples_metadata = {}
  for arm_id, arm in arms.items():
    print(f'---- Sampling {n_samples_per_arm} task instances from arm #{arm_id:2d} ----')
    samples_metadata[arm_id] = evaluate_arm(arm_id, arms, demontrations, n_objects, n_dimensions, initial_joint_config, n_samples_per_arm, reuse_tasks_instances)

  arm_probabilities = [
    (arm_id, len(data['failure']) / (len(data['success']) + len(data['failure'])))
    for arm_id, data in samples_metadata.items()
  ]
  worst_arm_probability = max([probability for _, probability in arm_probabilities])
  worst_arm_index = np.random.choice([
    arm_id for arm_id, probability in arm_probabilities if probability == worst_arm_probability
  ])

  min_score = min(samples_metadata[worst_arm_index]['failure_score'], default=0)  # pick the lowest score
  failed_task_instances = [
    task_instances for task_instances, score
    in zip(samples_metadata[worst_arm_index]['failure'], samples_metadata[worst_arm_index]['failure_score'])
    if score == min_score
  ]

  next_demonstration = None
  if len(failed_task_instances):
    next_demonstration_index = np.random.choice(range(len(failed_task_instances)))
    next_demonstration = failed_task_instances[next_demonstration_index]

  return worst_arm_index, worst_arm_probability, next_demonstration, samples_metadata




def self_evaluation(dimensions, n_objects, initial_joint_config, arms, epsilon=0.25, delta=0.1, beta=0.9, plot_data=False, reuse_tasks_instances=False):
  demontrations = []
  arm_index = np.random.choice(list(arms.keys()))
  next_demonstration = None
  while True:
    print(f'======== Round #{len(demontrations) + 1} ========')
    arm = arms[arm_index]
    (x_min, x_max), (y_min, y_max), _ = arm['segment']
    x, y, _ = ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0,0) if next_demonstration is None else next_demonstration[0]
    if len(arm['demos']) == 0:
      print(f'No demonstration found in arm #{arm_index}')
      nearby_demos = [(ai, *(sorted(
        [(di, utils.distance_of_demo(x, y, d)) for di, d in enumerate(a['demos'])],
        key=lambda x: x[-1],
        reverse=False)[0]
      )) for ai, a in arms.items() if len(a['demos']) != 0]
      if len(nearby_demos) == 0:
        print('No more demonstration available, exiting prematurely.')
        return []
      arm_index, demo_index, _ = sorted(nearby_demos, key=lambda x: x[-1], reverse=False)[0]
      print(f'Obtained the nearest demonstration from arm #{arm_index}')
      arm = arms[arm_index]
      demo = arm['demos'][demo_index]
    else:
      print(f'Obtained a demonstration from arm #{arm_index}')
      demo_index, _ = sorted(
        [(di, utils.distance_of_demo(x, y, d)) for di, d in enumerate(arm['demos'])],
        key=lambda x: x[-1],
        reverse=False
      )[0]
      demo = arm['demos'][demo_index]
    del arm['demos'][demo_index]
    demontrations.append(demo)
    arms_copy = arms if reuse_tasks_instances else copy.deepcopy(arms)    
    arm_index, worst_arm_probability, next_demonstration, samples = naive_pac(
      arms_copy,
      demontrations,
      n_objects,
      len(dimensions),
      initial_joint_config,
      epsilon=epsilon,
      delta=delta,
      reuse_tasks_instances=reuse_tasks_instances
    )


  
    if plot_data:
      plot.plot_heatmap(dimensions, n_objects, arms_copy, demontrations, samples, output_directory='images')
    if worst_arm_probability < 1 + epsilon - beta:
      return demontrations
