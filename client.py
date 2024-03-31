import csv
import numpy as np
from client import bandit, plot, utils, config, planner




def k_successful_task_instances_in_each_region(k, arms, initial_joint_config, demontrations, n_objects, dimensions, output_directory):
  task_plans = {}
  for id, val in arms.items():
    low, high = zip(*val['segment'])
    while True:
      task_instance = np.random.uniform(low=low, high=high, size=(1, len(dimensions))).reshape(1, n_objects, len(dimensions))
      se3_task_instance = np.apply_along_axis(utils.to_SE3, 2, task_instance)  
      for j, demo in enumerate(demontrations):
        [[plan]] = planner.remote_planner(config.REMOTE_SERVER, [demo], initial_joint_config, se3_task_instance)
        if plan['is_successful']:
          task_plans.setdefault(id, []).append((task_instance, plan['plan']))
          break
      if id in task_plans and len(task_plans[id]) == k:
        break
  print(f'---- {k * len(arms)} plans generated ----')
  for id, plans in task_plans.items():
    print(f'---- saving {k} plans in region #{id} ----')
    for i, (task, plan) in enumerate(plans):
      file_name = f'{i+1}_in_region_{id}'
      plot.plot_object_for_video(dimensions, task[0][0], output_directory, f'object_{file_name}', transparent = True)
      with open(f'{output_directory}/plan_{file_name}.csv', 'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'])
        csvwriter.writerows(plan)
      with open(f'{output_directory}/object_pose_{file_name}.csv', 'w') as file:
        file.write(str(task[0][0]))




if __name__ == '__main__':

  DEMO_PATH = 'demonstrations/Scoop_Interactive'

  dimensions, initial_joint_config, n_objects, demonstrations, bandit_arms = utils.process_demos(DEMO_PATH)


  # interactive simulation
  _, _, [next_demo], samples = bandit.naive_pac(bandit_arms, demonstrations, n_objects, len(dimensions), initial_joint_config, epsilon=0.2, delta=0.05)
  plot.plot_heatmap_for_video(dimensions, n_objects, bandit_arms, demonstrations, samples, next_demo, draw_next_demo = False, output_directory='images', transparent=True)
  print('----- provide the next demonstration here -----')
  print(next_demo)


  
  # # simulation with at least 1 pre-collected demo in each arm
  # demontrations = bandit.self_evaluation(dimensions, n_objects, initial_joint_config, bandit_arms, epsilon = 0.2, delta = 0.05, beta = 0.95, plot_data = True, reuse_tasks_instances = False)
  # print(demontrations)



  # k_successful_task_instances_in_each_region(2, bandit_arms, initial_joint_config, demonstrations, n_objects, dimensions, 'plans')
  
  pass
