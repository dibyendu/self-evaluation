Open `http://192.168.1.5` in chrome and unloack the joints

## Run on Franka's server

Address:  `irsl@192.168.1.100`<br/>
Password: `roboticsHE133`

#### Tab 1
  1. `roscore`

#### Tab 2
  1. `source ~/Dasharadhan/franka_ws/ros_ws/devel/setup.bash`
  2. `roslaunch franka_control franka_control.launch robot_ip:=192.168.1.5` (To move the arm manually)

#### Tab 3
  1. `source ~/Dasharadhan/franka_ws/ros_ws/devel/setup.bash`
  2. `roslaunch user_controllers user_joint_position_controller.launch` (To move the arm programmatically)
  3. `echo "0.07598477351140998
-1.5053082774145556
-0.4201544715354317
-1.2670914326216045
-0.756117510623402
0.8283229323798543
-0.085027550574806" > ~/Desktop/franka_joint_config.csv`
  3. `echo <N> | rosrun franka_motion_planning goto_joint_config`
     - N = 1 to save robot joint configuration
     - N = 2 to move to saved joint configuration
     - N = 3 to move to home configuration

#### Tab 4
  1. `source ~/Dasharadhan/franka_ws/ros_ws/devel/setup.bash` (To get the camera poses)
  3. `rosrun franka_motion_planning publish_camera_pose`


## Run on Baxter's server

Address:  `shubham@192.168.1.107`<br/>
Password: `CSE523`

#### Tab 4
  1. `cd ~/self-evaluation/web_assembly`
  2. `source ~/miniconda3/bin/activate py37`
     - `npm i`
     - `pip install -r backend/requirements.txt`
     - `npm run build`
  3. `python backend/app.py`
  4. `source ~/miniconda3/bin/deactivate`
