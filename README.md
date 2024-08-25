Open `http://192.168.1.5` in chrome and unloack the joints

## Run on Franka's server

Address:  `irsl@192.168.1.100`<br/>
Password: `roboticsHE133`

#### Tab 1
  1. `roscore`

#### Tab 2
  1. `source ~/Dasharadhan/franka_ws/ros_ws/devel/setup.bash`
  2. `roslaunch franka_control franka_control.launch robot_ip:=192.168.1.5`
  
#### Tab 3
  1. `source ~/Dasharadhan/franka_ws/ros_ws/devel/setup.bash`
  2. `rosrun franka_motion_planning publish_camera_pose`


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