// emcc -std=c++11 -O2 ../kinlib/*.cpp planner.cpp -o src/planner.mjs -s ALLOW_MEMORY_GROWTH -lembind

#include <array>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <cstdbool>
#include <emscripten/bind.h>

#include "../kinlib/motion_planning.h"
#include "../kinlib/kinlib_kinematics.h"




typedef struct {
  std::string joint_axes, joint_q, joint_limits, gst0;
} RobotConfiguration;

typedef struct {
  int id;
  double roi, score;
  std::string joint_angles, object_poses;
  kinlib::Demonstration demo;
} Demonstration;

typedef struct {
  int demo_id;
  int plan_length;
  std::vector<int> screw_segments;
  bool is_successful;
  std::vector<std::vector<double>> joint_angles;
  int failed_screw_segment, failed_joint_id;
} PlanInfo;




template<typename M>
M loadCSV (const std::string &data, bool ignore_header=false) {
  std::stringstream indata(data);
  std::string line;
  std::vector<double> values;
  int rows = 0;
  if (ignore_header) std::getline(indata, line);
  while (std::getline(indata, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (getline(lineStream, cell, ','))
      values.push_back(stod(cell));
    ++rows;
  }
  return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
}




kinlib::KinematicsSolver initSolver(RobotConfiguration &config) {
  kinlib::KinematicsSolver kinSolver;
  kinlib::Manipulator baxter_manipulator;

  std::array<std::string, 7> joint_names{"S0", "S1", "E0", "E1", "W0", "W1", "W2"};

  Eigen::MatrixXd joint_axes = loadCSV<Eigen::MatrixXd>(config.joint_axes);
  Eigen::MatrixXd joint_q = loadCSV<Eigen::MatrixXd>(config.joint_q);
  Eigen::MatrixXd joint_limits = loadCSV<Eigen::MatrixXd>(config.joint_limits);
  Eigen::MatrixXd gst_0 = loadCSV<Eigen::MatrixXd>(config.gst0);

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

  kinSolver.setManipulator(baxter_manipulator);
  return kinSolver;
}




std::vector<std::vector<PlanInfo>> motionPlan(
  RobotConfiguration &robotConfig,
  std::vector<double> &initial_joint_values,
  std::vector<Demonstration> &demonstrations,
  std::vector<std::vector<std::vector<double>>> &taskInstances
) {

  kinlib::KinematicsSolver kinSolver = initSolver(robotConfig);


  int n_task_instances = taskInstances.size();
  kinlib::TaskInstance *task_instances = (kinlib::TaskInstance *) calloc(n_task_instances, sizeof(kinlib::TaskInstance));
  for (int i = 0; i < n_task_instances; i++) {
    int n_poses_in_task_instance = taskInstances[i].size();
    for (int j = 0; j < n_poses_in_task_instance; j++) {
      int length = taskInstances[i][j].size();
      Eigen::Matrix4d object_pose;
      for (int k = 0; k < length; k++) object_pose(k) = taskInstances[i][j][k];
      task_instances[i].object_poses.push_back(object_pose.transpose());
    }
  }

  int joint_config_length = initial_joint_values.size();
  Eigen::VectorXd init_jnt_val(joint_config_length);
  for (int i = 0; i < joint_config_length; i++)
    init_jnt_val(i) = initial_joint_values[i];

  int n_demontrations = demonstrations.size();
  for (int i = 0; i < n_demontrations; i++) {
    Eigen::MatrixXd object_poses = loadCSV<Eigen::MatrixXd>(demonstrations[i].object_poses),
                    joint_angles = loadCSV<Eigen::MatrixXd>(demonstrations[i].joint_angles, true);
    
    std::vector<Eigen::Matrix4d> recorded_ee_traj, obj_poses;
    for(int j = 0; j < joint_angles.rows(); j++) {
      Eigen::VectorXd jnt_cfg = joint_angles.block<1,7>(j, 1).transpose();
      Eigen::Matrix4d ee_pose;
      kinSolver.getFK(jnt_cfg, ee_pose);
      if (j != 0 && kinlib::positionDistance(recorded_ee_traj.back(), ee_pose) < 0.005)
        continue;
      recorded_ee_traj.push_back(ee_pose);
    }
    
    for(int j = 0; j < object_poses.rows() / 4; j++) {
      Eigen::Matrix4d g = object_poses.block<4,4>((j * 4), 0);
      obj_poses.push_back(g);
    }

    demonstrations[i].demo = kinlib::saveDemonstration(recorded_ee_traj, obj_poses, demonstrations[i].roi);  
  }


  std::vector<std::vector<PlanInfo>> plans;
  for (int i = 0; i < n_task_instances; i++) {
    std::vector<PlanInfo> task_plans;
    for (int j = 0; j < n_demontrations; j++) {
      PlanInfo plan;
      Eigen::VectorXd jnt_val = init_jnt_val;
      std::vector<Eigen::Matrix4d> guiding_poses;
      kinlib::UserGuidedMotionPlanner::planMotionForNewTaskInstance(demonstrations[j].demo, task_instances[i], guiding_poses);

      int screw_segment = 0;
      plan.is_successful = true;
      plan.demo_id = demonstrations[j].id;
      std::vector<Eigen::VectorXd> joint_angles;
      std::vector<std::vector<double>> plan_joint_angles;
      for (auto guiding_pose : guiding_poses) {
        Eigen::Matrix4d init_ee_g;
        kinSolver.getFK(jnt_val, init_ee_g);

        // Get motion plan using ScLERP Planner
        kinlib::MotionPlanResult plan_info;
        std::vector<Eigen::VectorXd> plan_result;
        kinlib::ErrorCodes code = kinSolver.getMotionPlan(jnt_val, init_ee_g, guiding_pose, plan_result, plan_info);

        joint_angles.reserve(joint_angles.size() + distance(plan_result.begin(), plan_result.end()));
        joint_angles.insert(joint_angles.end(), plan_result.begin(), plan_result.end());
        for (auto joint_angle : plan_result) {
          std::vector<double> joint_angle_data(joint_angle.data(), joint_angle.data() + joint_angle.size());
          plan_joint_angles.push_back(joint_angle_data);
        }

        if (code == kinlib::ErrorCodes::OPERATION_SUCCESS) {
          screw_segment += 1;
          jnt_val = plan_result.back();
          plan.screw_segments.push_back(plan_joint_angles.size() - 1);
        } else {
          int joint_id = plan_info.result == kinlib::MotionPlanReturnCodes::JOINT_LIMITS_VIOLATED ? plan_info.joint_id : -1;
          plan.is_successful = false;
          plan.plan_length = guiding_poses.size();
          plan.failed_screw_segment = screw_segment + 1;
          plan.failed_joint_id = joint_id;
          plan.joint_angles = plan_joint_angles;
          break;
        }
      }

      if (plan.is_successful) {
        plan.plan_length = guiding_poses.size();
        plan.joint_angles = plan_joint_angles;
        task_plans.push_back(plan);
        break;
      }
      task_plans.push_back(plan);
    }
    plans.push_back(task_plans);
  }

  free(task_instances);

  return plans;
}


EMSCRIPTEN_BINDINGS(motion_planner) {
  emscripten::value_object<RobotConfiguration>("RobotConfiguration")
  .field("joint_axes", &RobotConfiguration::joint_axes)
  .field("joint_q", &RobotConfiguration::joint_q)
  .field("joint_limits", &RobotConfiguration::joint_limits)
  .field("gst0", &RobotConfiguration::gst0);

  emscripten::value_object<Demonstration>("Demonstration")
  .field("id", &Demonstration::id)
  .field("score", &Demonstration::score)
  .field("region_of_interest", &Demonstration::roi)
  .field("joint_angles", &Demonstration::joint_angles)
  .field("object_poses", &Demonstration::object_poses);

  emscripten::value_object<PlanInfo>("PlanInfo")
  .field("demo_id", &PlanInfo::demo_id)
  .field("n_screw_segments", &PlanInfo::plan_length)
  .field("screw_segments", &PlanInfo::screw_segments)
  .field("is_successful", &PlanInfo::is_successful)
  .field("joint_angles", &PlanInfo::joint_angles)
  .field("failed_screw_segment", &PlanInfo::failed_screw_segment)
  .field("failed_joint_id", &PlanInfo::failed_joint_id);

  emscripten::register_vector<int>("VectorInt");
  emscripten::register_vector<double>("VectorDouble");
  emscripten::register_vector<PlanInfo>("VectorPlan");
  emscripten::register_vector<std::vector<PlanInfo>>("VectorPlans");
  emscripten::register_vector<std::vector<double>>("VectorVectorDouble");
  emscripten::register_vector<Demonstration>("DemonstrationVector");
  emscripten::register_vector<std::vector<std::vector<double>>>("TaskInstanceVector");

  emscripten::function("motionPlan", &motionPlan);
}