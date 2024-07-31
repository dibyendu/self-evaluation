// g++ -std=c++11 -O2 kinlib/*.cpp planner.cpp -o server -lpthread

#ifdef __APPLE__
  #include <libkern/OSByteOrder.h>
  #define htobe64(x) OSSwapHostToBigInt64(x)
  #define be64toh(x) OSSwapBigToHostInt64(x)
#endif

#include <array>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <cstdbool>
#include <iostream>
#include <unistd.h>
#include <sys/un.h>
#include <pthread.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "kinlib/kinlib_kinematics.h"
#include "kinlib/motion_planning.h"


int N_THREADS;
kinlib::KinematicsSolver kin_solver;


template<typename M>
M loadCSV (const std::string &file, bool ignore_header=false) {
  std::ifstream indata;
  indata.open(file);
  std::string line;
  std::vector<double> values;
  uint rows = 0;
  if (ignore_header) std::getline(indata, line);
  while (std::getline(indata, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (getline(lineStream, cell, ','))
      values.push_back(stod(cell));
    ++rows;
  }
  indata.close();
  return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
}


void init_solver() {
  kinlib::Manipulator baxter_manipulator;

  std::array<std::string, 7> joint_names{"S0", "S1", "E0", "E1", "W0", "W1", "W2"};

  Eigen::MatrixXd joint_axes = loadCSV<Eigen::MatrixXd>("baxter_config/baxter_joint_axes.csv");
  Eigen::MatrixXd joint_q = loadCSV<Eigen::MatrixXd>("baxter_config/baxter_joint_q.csv");
  Eigen::MatrixXd joint_limits = loadCSV<Eigen::MatrixXd>("baxter_config/baxter_joint_limits.csv");
  Eigen::MatrixXd gst_0 = loadCSV<Eigen::MatrixXd>("baxter_config/baxter_gst0.csv");

  for (int i = 0; i < 7; i++) {
    Eigen::Vector4d jnt_axis;
    jnt_axis.head<3>() = joint_axes.block<3, 1>(0, i);
    jnt_axis(3) = 0;

    Eigen::Vector4d jnt_q;
    jnt_q.head<3>() = joint_q.block<3, 1>(0, i);
    jnt_q(3) = 0;

    kinlib::JointLimits jnt_limits;
    jnt_limits.lower_limit_ = joint_limits(i, 0);
    jnt_limits.upper_limit_ = joint_limits(i, 1);

    baxter_manipulator.addJoint(kinlib::JointType::Revolute, joint_names[i], jnt_axis, jnt_q, jnt_limits, gst_0);
  }

  kin_solver.setManipulator(baxter_manipulator);
}


union {
  double f;                                                   // float
  uint64_t i;                                                 // uint32_t for float;
} float_buffer;

void send_float(int socket_fd, double data) {                 // float      
  float_buffer.f = data;
  float_buffer.i = htobe64(float_buffer.i);                   // htonl for float
  send(socket_fd, &float_buffer.i, sizeof(double), 0);        // float
}

double recv_float(int socket_fd) {                            // float
  recv(socket_fd, &float_buffer.i, sizeof(double), 0);        // float
  float_buffer.i = be64toh(float_buffer.i);                   // ntohl for float
  return float_buffer.f;
}

void send_int(int socket_fd, int data) {
  send(socket_fd, &data, sizeof(int), 0);
}

int recv_int(int socket_fd) {
  int data;
  recv(socket_fd, &data, 4, 0);
  return ntohl(data);
}

void send_bool(int socket_fd, bool data) {
  send(socket_fd, &data, sizeof(bool), 0);
}

char *recv_str(int socket_fd) {
  int length = recv_int(socket_fd);
  char *data = (char *) calloc(length + 1, sizeof(char));
  recv(socket_fd, data, length, 0);
  return data;
}

void send_plan(int socket_fd, std::vector<Eigen::VectorXd> path) {
  send_int(socket_fd, path.size());
  for(auto joint_angles : path) {
    send_int(socket_fd, joint_angles.size());
    for (int i = 0; i < joint_angles.size(); i++)
      send_float(socket_fd, joint_angles(i));
  }
}


typedef struct {
  char *recorded_demo_file, *object_poses_file;
  double roi, score;
  kinlib::Demonstration demo;
} Demontration;

typedef struct {
  int plan_length;
  bool is_successful;
  std::vector<Eigen::VectorXd> joint_angles;
  int failed_screw_segment, failed_joint_id;
} PlanInfo;

typedef struct {
  Demontration *demontrations;
  kinlib::TaskInstance *task_instances;
  Eigen::VectorXd *init_jnt_val;
  int n_demontrations, n_task_instances;
  int start_index, end_index;
  PlanInfo **plans;
} ThreadArg;


void *planner_thread(void *args) {
  ThreadArg *arg = (ThreadArg *) args;
  Demontration *demontrations = arg->demontrations;
  kinlib::TaskInstance *task_instances = arg->task_instances;
  Eigen::VectorXd init_jnt_val = *(arg->init_jnt_val);
  PlanInfo **plans = arg->plans;
  int n_demontrations = arg->n_demontrations,
      n_task_instances = arg->n_task_instances;
  int start_index = arg->start_index,
      end_index = arg->end_index;

  for (int i = start_index; i <= end_index; i++) {
    for (int j = 0; j < n_demontrations; j++) {
      Eigen::VectorXd jnt_val = init_jnt_val;
      std::vector<Eigen::Matrix4d> guiding_poses;
      kinlib::UserGuidedMotionPlanner::planMotionForNewTaskInstance(demontrations[j].demo, task_instances[i], guiding_poses);

      int screw_segment = 0;
      bool plan_successful = true;
      std::vector<Eigen::VectorXd> joint_angles;
      for (auto guiding_pose : guiding_poses) {
        Eigen::Matrix4d init_ee_g;
        kin_solver.getFK(jnt_val, init_ee_g);

        // Get motion plan using ScLERP Planner
        kinlib::MotionPlanResult plan_info;
        std::vector<Eigen::VectorXd> plan_result;
        kinlib::ErrorCodes code = kin_solver.getMotionPlan(jnt_val, init_ee_g, guiding_pose, plan_result, plan_info);

        joint_angles.reserve(joint_angles.size() + distance(plan_result.begin(), plan_result.end()));
        joint_angles.insert(joint_angles.end(), plan_result.begin(), plan_result.end());
        
        if (code == kinlib::ErrorCodes::OPERATION_SUCCESS) {
          jnt_val = plan_result.back();
          screw_segment += 1;
        } else {
          int joint_id = plan_info.result == kinlib::MotionPlanReturnCodes::JOINT_LIMITS_VIOLATED ? plan_info.joint_id : -1;
          plan_successful = false;
          plans[i][j].plan_length = guiding_poses.size();
          plans[i][j].is_successful = false;
          plans[i][j].joint_angles = joint_angles;
          plans[i][j].failed_screw_segment = screw_segment + 1;
          plans[i][j].failed_joint_id = joint_id;
          break;
        }
      }

      if (plan_successful) {
        plans[i][j].plan_length = guiding_poses.size();
        plans[i][j].is_successful = true;
        plans[i][j].joint_angles = joint_angles;
        break;
      }
    }
  }

  return NULL;
}


void *handle_request(void *arg) {
  
  int client_socket_fd = ((int *)arg)[0];
  int thread_index = ((int *)arg)[1];
  
  int n_demontrations = recv_int(client_socket_fd);
  Demontration *demontrations = (Demontration *) calloc(n_demontrations, sizeof(Demontration));
  for (int i = 0; i < n_demontrations; i++) {
    demontrations[i].recorded_demo_file = recv_str(client_socket_fd);
    demontrations[i].object_poses_file = recv_str(client_socket_fd);
    demontrations[i].roi = recv_float(client_socket_fd);
    demontrations[i].score = recv_float(client_socket_fd);
  }

  int n_task_instances = recv_int(client_socket_fd);
  kinlib::TaskInstance *task_instances = (kinlib::TaskInstance *) calloc(n_task_instances, sizeof(kinlib::TaskInstance));
  for (int i = 0; i < n_task_instances; i++) {
    int n_poses_in_task_instance = recv_int(client_socket_fd);
    for (int j = 0; j < n_poses_in_task_instance; j++) {
      int length = recv_int(client_socket_fd);
      Eigen::Matrix4d object_pose;
      for (int k = 0; k < length; k++) object_pose(k) = recv_float(client_socket_fd);
      task_instances[i].object_poses.push_back(object_pose.transpose());
    }
  }

  int joint_config_length = recv_int(client_socket_fd);
  Eigen::VectorXd init_jnt_val(joint_config_length);
  for (int i = 0; i < joint_config_length; i++)
    init_jnt_val(i) = recv_float(client_socket_fd);

  int n_threads = N_THREADS > n_task_instances ? n_task_instances : N_THREADS;

  std::cout << "===================== Invocation # " << thread_index << " =====================\n";
  std::cout << "Task instances to evaluate : " << n_task_instances << "\n";
  std::cout << "Threads used for evaluation: " << n_threads << "\n";
  for (int i = 0; i < n_demontrations; i++)
    std::cout << "Demontration #" << i+1 << ": {\n\tdemo: " << demontrations[i].recorded_demo_file << ",\n\tobject pose: " << demontrations[i].object_poses_file << ",\n\troi: " << demontrations[i].roi << ",\n\tscore: " << demontrations[i].score << "\n}\n";
  std::cout << "Task instance #1:\n";
  for(auto pose : task_instances[0].object_poses)
    std::cout << pose << "\n\n";
  std::cout << "Initial joint config:\n";
  std::cout << init_jnt_val << "\n";
  std::cout << "==========================================================\n";
  
  for (int i = 0; i < n_demontrations; i++) {
    Eigen::MatrixXd recorded_demo = loadCSV<Eigen::MatrixXd>(demontrations[i].recorded_demo_file, true),
                    object_poses = loadCSV<Eigen::MatrixXd>(demontrations[i].object_poses_file);
    
    std::vector<Eigen::Matrix4d> recorded_ee_traj, obj_poses;
    
    for(int j = 0; j < recorded_demo.rows(); j++) {
      Eigen::VectorXd jnt_cfg = recorded_demo.block<1,7>(j,1).transpose();
      Eigen::Matrix4d ee_pose;
      kin_solver.getFK(jnt_cfg, ee_pose);
      if (j != 0 && kinlib::positionDistance(recorded_ee_traj.back(), ee_pose) < 0.005)
        continue;
      recorded_ee_traj.push_back(ee_pose);
    }
    
    for(int j = 0; j < object_poses.rows() / 4; j++) {
      Eigen::Matrix4d g = object_poses.block<4,4>((j*4),0);
      obj_poses.push_back(g);
    }

    demontrations[i].demo = kinlib::saveDemonstration(recorded_ee_traj, obj_poses, demontrations[i].roi);  
  }


  pthread_t *threads = (pthread_t *) calloc(n_threads, sizeof(pthread_t));
  ThreadArg *args = (ThreadArg *) calloc(n_threads, sizeof(ThreadArg));
  PlanInfo **plans = (PlanInfo **) calloc(n_task_instances, sizeof(PlanInfo *));
  for (int i = 0; i < n_task_instances; i++)
    plans[i] = (PlanInfo *) calloc(n_demontrations, sizeof(PlanInfo));

  int n_task_instances_per_thread = n_task_instances / n_threads;
  for (int t = 0; t < n_threads; t++) {
    args[t].demontrations = demontrations;
    args[t].task_instances = task_instances;
    args[t].init_jnt_val = &(init_jnt_val);
    args[t].n_demontrations = n_demontrations;
    args[t].n_task_instances = n_task_instances;
    args[t].plans = plans;
    args[t].start_index = t * n_task_instances_per_thread;
    args[t].end_index = t == n_threads - 1 ? n_task_instances - 1 : (t + 1) * n_task_instances_per_thread - 1;    
    pthread_create(&(threads[t]), NULL, planner_thread, (void *) &(args[t]));
  }

  for (int t = 0; t < n_threads; t++)
    pthread_join(threads[t], NULL);

  for (int i = 0; i < n_task_instances; i++) {
    for (int j = 0; j < n_demontrations; j++) {
      send_int(client_socket_fd, plans[i][j].plan_length);
      send_plan(client_socket_fd, plans[i][j].joint_angles);
      send_bool(client_socket_fd, plans[i][j].is_successful);
      if (plans[i][j].is_successful) break;
      send_int(client_socket_fd, plans[i][j].failed_screw_segment);
      send_int(client_socket_fd, plans[i][j].failed_joint_id);
    }    
  }

  close(client_socket_fd);

  free(demontrations);
  free(task_instances);
  free(threads);
  free(args);
  for (int i = 0; i < n_task_instances; i++)
    free(plans[i]);
  free(plans);

  return NULL;
}


int main(int argc, char *argv[]) {

  // int port = 8888;
  // int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  // struct sockaddr_in socket_info;
  // memset(&socket_info, 0, sizeof(socket_info));
  // socket_info.sin_family = AF_INET;
  // socket_info.sin_port = htons(port);
  // socket_info.sin_addr.s_addr = INADDR_ANY;

  const char *port = "./socket_file";
  remove(port);
  int socket_fd = socket(AF_UNIX, SOCK_STREAM, 0);
  struct sockaddr_un socket_info;
  memset(&socket_info, 0, sizeof(socket_info));
  socket_info.sun_family = AF_UNIX;
  strcpy(socket_info.sun_path, port);


  bind(socket_fd, (struct sockaddr *) &socket_info, sizeof(socket_info));

  listen(socket_fd, 10);

  int thread_count = 0;
  pthread_t thread;

  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  init_solver();

  N_THREADS = std::thread::hardware_concurrency();
  N_THREADS = N_THREADS > 0 ? N_THREADS : 1;

  while (true) {
    int client_socket_fd = accept(socket_fd, NULL, NULL);
    int args[2] = { client_socket_fd, thread_count++ };
    pthread_create(&thread, &attr, handle_request, (void *)args);
  }

  pthread_attr_destroy(&attr);

  close(socket_fd);
  return 0;
}