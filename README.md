[![Open in StackBlitz](https://developer.stackblitz.com/img/open_in_stackblitz_small.svg)](https://stackblitz.com/github/dibyendu/self-evaluation?configPath=web_assembly)

## Run in client-server mode
### Compile server code

`g++ -std=c++11 -O2 kinlib/*.cpp server.cpp -o server -lpthread`

### Start server
`./server`

### Start client
`python client.py`




## Run in client mode (web-assembly)

`cd web_assembly`

### Compile server code

`emcc -std=c++11 -O2 ../kinlib/*.cpp server.cpp -o src/server.mjs -s ALLOW_MEMORY_GROWTH -lembind`

### Start application
`npm start`




## On the baxter machine, follow these steps to perform experiments

#### To connect to robot
`cd baxter_ws && . baxter.sh`

#### To enable robot
`rosrun baxter_tools enable_robot.py -e`

#### To get joint angles
`rosrun baxter_examples get_joint_angles.py`

### To get the object pose from the end-effector top position
`rosrun baxter_examples ee_tip_tf.py`

#### To store demo in a file
`rosrun baxter_examples joint_recorder.py --file <FILE_NAME>.csv`

#### To reposition the left arm back to initial configuration
`rosrun baxter_examples left_home_config.py`

#### To execute the joint angles from a file

Option 1 (preferred):

  ```
  nano src/baxter_examples/src/js_motion_plan_execution.cpp 
  catkin_make
  rosrun baxter_examples js_motion_plan_execution <FILE_NAME>.csv
  ```

Option 2:

`rosrun baxter_examples left_joint_position_file_playback.py --file <FILE_NAME>.csv`
