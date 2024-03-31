/*
 * Kinemtic Solver definitions for kinlib
 */

/* Author: Dasharadhan Mahalingam */

#include <iostream>
#include "kinlib_kinematics.h"

Eigen::IOFormat PrintFormat(4,0,", ","\n");

#if DEBUG

#include <signal.h>
#include <fstream>
#include <ctime>
#include <sys/stat.h>
#include <errno.h>

namespace kinlib
{

Eigen::IOFormat CSVFormat(Eigen::FullPrecision,0,",","\n","","","","");

std::string log_folder_name = "kinlib_kinematics";
std::string home_dir(getenv("HOME"));
std::string log_dir = home_dir + "/logs/" + log_folder_name;

std::string timestamp_str = "Y%YM%mD%d_T%H%M%S";

bool log_folder_error = false;
bool log_file_error = false;
bool log_plan_request_error = false;

}

#endif

namespace kinlib
{
double positionDistance(const Eigen::Matrix4d &t1, const Eigen::Matrix4d &t2)
{
  Eigen::Vector4d p_delta = t1.col(3) - t2.col(3);
  return p_delta.norm();
}

double positionDistance(const Eigen::VectorXd &p1, const Eigen::VectorXd &p2)
{
  Eigen::Vector3d p_delta = p1.head<3>() - p2.head<3>();
  return p_delta.norm();
}

double rotationDistance(const Eigen::Matrix4d &t1, const Eigen::Matrix4d &t2)
{
  Eigen::Quaterniond q1(t1.block<3,3>(0,0));
  Eigen::Quaterniond q2(t2.block<3,3>(0,0));

  return rotationDistance(q1, q2);
}

double rotationDistance(const Eigen::Quaterniond &q1, 
                        const Eigen::Quaterniond &q2)
{
  Eigen::Vector4d q1_coeff = q1.coeffs();
  Eigen::Vector4d q2_coeff = q2.coeffs();

  double d1 = (q1_coeff - q2_coeff).norm();
  double d2 = (q1_coeff + q2_coeff).norm();

  return (d1 > d2 ? d2 : d1);
}

double rotationDistance(eigen_ext::DualQuat &dq_1,
                        eigen_ext::DualQuat &dq_2)
{
  Eigen::Vector4d q1_coeff = dq_1.getRealPartVec();
  Eigen::Vector4d q2_coeff = dq_2.getRealPartVec();

  double d1 = (q1_coeff - q2_coeff).norm();
  double d2 = (q1_coeff + q2_coeff).norm();

  return (d1 > d2 ? d2 : d1);
}

Eigen::Matrix3d getSkewMatrix(const Eigen::VectorXd &vec)
{
  Eigen::Matrix3d skew_mat = Eigen::Matrix3d::Zero();

  skew_mat(0,1) = -vec(2);
  skew_mat(0,2) = vec(1);
  skew_mat(1,0) = vec(2);
  skew_mat(1,2) = -vec(0);
  skew_mat(2,0) = -vec(1);
  skew_mat(2,1) = vec(0);

  return skew_mat;
}

Eigen::Matrix4d getTransformationInv(const Eigen::Matrix4d &g)
{
  Eigen::Matrix4d g_inv = Eigen::Matrix4d::Identity();

  g_inv.block<3,3>(0,0) = g.block<3,3>(0,0).transpose();
  g_inv.block<3,1>(0,3) = -g_inv.block<3,3>(0,0) * g.block<3,1>(0,3);

  return g_inv;
}

Eigen::Matrix<double,6,6> getAdjoint(const Eigen::Matrix4d &g)
{
  Eigen::Matrix<double,6,6> adj = Eigen::Matrix<double,6,6>::Zero();

  //std::cout << "adj :\n" << adj << "\n\n";

  Eigen::Vector4d p = g.col(3);
  adj.block<3,3>(0,0) = g.block<3,3>(0,0);
  adj.block<3,3>(3,3) = g.block<3,3>(0,0);
  adj.block<3,3>(0,3) = getSkewMatrix(p) * g.block<3,3>(0,0);

  //std::cout << "adj :\n" << adj << "\n\n";

  return adj;
}

ErrorCodes getScrewParameters(  const Eigen::Matrix4d &g_i,
                                const Eigen::Matrix4d &g_f,
                                Eigen::Vector3d &omega,
                                double &theta,
                                double &h,
                                Eigen::Vector3d &l,
                                ScrewMotionType &screw_motion_type)
{
  eigen_ext::DualQuat dq_i(g_i);
  eigen_ext::DualQuat dq_f(g_f);

  Eigen::Matrix4d g = g_f * getTransformationInv(g_i);

  Eigen::Matrix3d R = g.block<3,3>(0,0);
  Eigen::Vector3d p = g.block<3,1>(0,3);

  Eigen::AngleAxisd angle_axis(R);

  Eigen::Vector3d v = Eigen::Vector3d::Zero();

  // Assume that the motion is a general screw motion
  // Check for special cases is included later
  screw_motion_type = ScrewMotionType::GENERAL_SCREW;

  // Handle special case (Pure Translation)
  if(fabs(angle_axis.angle()) <= PURE_TRANSLATION_ROT_ANGLE_THRESHOLD)
  {
    // Pitch is infinity for pure translation
    // Set as 0 because infinity cannot be represented
    // Ignore pitch if screw_motion_type = ScrewMotionType::PURE_TRANSLATION
    h = 0;

    // Set screw_motion_type as ScrewMotionType::PURE_TRANSLATION
    screw_motion_type = ScrewMotionType::PURE_TRANSLATION;

    // Magnitude of screw
    theta = p.norm();

    // Handle special case where there is no motion
    // (i.e) g_i = g_f
    if(theta <= NO_MOTION_MAGNITUDE_THRESHOLD)
    {
      screw_motion_type = ScrewMotionType::NO_MOTION;

      omega(0) = 0; omega(1) = 0; omega(2) = 0;
      l(0) = 0; l(1) = 0; l(2) = 0;

      return ErrorCodes::OPERATION_SUCCESS;
    }

    // Screw axis
    omega = p.normalized();

    // Point on the screw axis (origin)
    l(0) = 0; l(1) = 0; l(2) = 0;

    return ErrorCodes::OPERATION_SUCCESS;
  }

  // Magnitude of screw
  theta = angle_axis.angle();

  // Screw axis
  omega = angle_axis.axis();

  Eigen::Matrix3d A = ((Eigen::Matrix3d::Identity() - R) * getSkewMatrix(omega))
      + (theta * (omega * omega.transpose()));

  v = A.inverse() * p;

  // Pitch of screw
  h = omega.transpose() * v;

  // Check if motion is pure rotation
  if(fabs(h) < PURE_ROTATION_PITCH_THRESHOLD)
  {
    screw_motion_type = ScrewMotionType::PURE_ROTATION;
  }

  // Point on the screw axis
  l = omega.cross(v);

  return ErrorCodes::OPERATION_SUCCESS;
}

ErrorCodes getNearestPoseOnScrew( const Eigen::Matrix4d &g_i,
                                  const Eigen::Matrix4d &g_f,
                                  const Eigen::Matrix4d &g_t,
                                  double &t,
                                  double &d_pos,
                                  double &d_rot)
{
  eigen_ext::DualQuat dq_i(g_i);
  eigen_ext::DualQuat dq_f(g_f);

  std::vector<double> d_pos_array(3,0);
  std::vector<double> t_inter(3,0);

  std::vector<Eigen::Matrix4d> g_inter;
  g_inter.resize(3);

  double t_i = 0;
  double t_f = 1;
  double L = t_f - t_i;

  std::vector<eigen_ext::DualQuat> dq_t_inter;
  dq_t_inter.resize(3);

  int min_d_idx = 0;
  double min_d = 10000000;

  bool init_flag = false;

  std::vector<double> coarse_d(10,0);

  // Do a initial coarse search
  for(int i = 0; i <= 10; i++)
  {
    double t_coarse = i * 0.1;
  
    dq_t_inter[0] = eigen_ext::DualQuat::dualQuatInterpolation(
        dq_i, dq_f, t_coarse);
    g_inter[0] = dq_t_inter[0].getTransform();
    d_pos_array[0] = positionDistance(g_inter[0], g_t);

    if(d_pos_array[0] < min_d)
    {
      min_d = d_pos_array[0];
      min_d_idx = i;
    }
  }

  if(min_d < 1.0e-3)
  {
    t = min_d_idx * 0.1;
    goto compute_distances;
  }

  if(min_d_idx == 0)
  {
    t = 0;
    t_i = 0;
    t_f = 0.1;
    L = 0.1;
  }
  else if(min_d_idx == 10)
  {
    t = 1;
    t_i = 0.9;
    t_f = 1;
    L = 0.1;
  }
  else
  {
    t = min_d_idx * 0.1;
    t_i = (min_d_idx * 0.1) - 0.1;
    t_f = (min_d_idx * 0.1) + 0.1;
    L = 0.1;
  }

  while(min_d > 0.001)
  {
    L = t_f - t_i;

    if(L < 0.0001)
    {
      t = (t_i + t_f) / 2.0;
      break;
    }

    t_inter[0] = t_i + (L/4.0);
    t_inter[1] = t_i + (L/2.0);
    t_inter[2] = t_f - (L/4.0);
    
    for(int i = 0; i < 3; i++)
    {
      dq_t_inter[i] = eigen_ext::DualQuat::dualQuatInterpolation(
          dq_i, dq_f, t_inter[i]);
      g_inter[i] = dq_t_inter[i].getTransform();
      d_pos_array[i] = positionDistance(g_inter[i], g_t);
    }

    if(d_pos_array[0] < d_pos_array[1])
    {
      t_f = t_inter[1];
      t = t_inter[0];
      min_d = d_pos_array[0];
    }
    else if(d_pos_array[2] < d_pos_array[1])
    {
      t_i = t_inter[1];
      t = t_inter[2];
      min_d = d_pos_array[2];
    }
    else
    {
      t_i = t_inter[0];
      t_f = t_inter[2];
      t = t_inter[1];
      min_d = d_pos_array[1];
    }
  }

compute_distances:
  dq_t_inter[0] = eigen_ext::DualQuat::dualQuatInterpolation(dq_i, dq_f, t);
  g_inter[0] = dq_t_inter[0].getTransform();

  d_pos = positionDistance(g_inter[0], g_t);
  d_rot = rotationDistance(g_inter[0], g_t);

  return ErrorCodes::OPERATION_SUCCESS;
}

ErrorCodes getScrewSegments(const std::vector<Eigen::Matrix4d> &g_seq,
                            std::vector<unsigned int> &segs,
                            double max_pos_d,
                            double max_rot_d)
{
  segs.clear();

  unsigned int start_idx = 0;

  unsigned int i,j,k;

  unsigned int end_idx;
  unsigned int itr;

  double pos_d_diff = 0;
  double rot_d_diff = 0;
  double nearest_t;

  while(start_idx < g_seq.size())
  {
    for(end_idx = start_idx + 1; end_idx < g_seq.size(); end_idx++)
    {
      double prev_nearest_t = -1;
      for(itr = start_idx + 1; itr <= end_idx; itr++)
      {
        getNearestPoseOnScrew(g_seq[start_idx], g_seq[end_idx], g_seq[itr],
                              nearest_t, pos_d_diff, rot_d_diff);
        if((pos_d_diff > max_pos_d) || (rot_d_diff > max_rot_d) || (std::abs(nearest_t - prev_nearest_t) < 1e-3))
        {
          segs.push_back(end_idx);
          start_idx = end_idx;

          if(end_idx == (g_seq.size() - 1))
          {
            return ErrorCodes::OPERATION_SUCCESS;
          }
          break;
        }
        prev_nearest_t = nearest_t;
      }

      if(end_idx == (g_seq.size() - 1))
      {
        segs.push_back(end_idx);
        return ErrorCodes::OPERATION_SUCCESS;
      }
    }
  }

  return ErrorCodes::OPERATION_SUCCESS;
}

KinematicsSolver::KinematicsSolver()
{

}

KinematicsSolver::KinematicsSolver(Manipulator manip) :
  manipulator_(manip)
{

}

Manipulator KinematicsSolver::getManipulator(void)
{
  return manipulator_;
}

void KinematicsSolver::setManipulator(Manipulator manip) {
  manipulator_ = manip;
}

ErrorCodes KinematicsSolver::getFK(const Eigen::VectorXd &jnt_values, 
                                  Eigen::Matrix4d &g_base_tool)
{
  Eigen::Matrix4d g;

  g_base_tool.setIdentity();

  for(unsigned int itr = 0; itr < manipulator_.joint_count_; itr++)
  {
    g.setIdentity();

    if(manipulator_.joint_types_[itr] == JointType::Revolute)
    {
      // Determine exponential for revolute joint
      Eigen::Matrix3d rot_mat;
      Eigen::Vector3d rot_axis(manipulator_.joint_axes_[itr].head<3>());
      rot_mat = Eigen::AngleAxisd(jnt_values(itr), rot_axis);
      g.block<3,3>(0,0) = rot_mat;

      g.block<3,1>(0,3) = (Eigen::Matrix3d::Identity() - rot_mat) *
                          manipulator_.joint_q_[itr].head<3>();
    }
    else if(manipulator_.joint_types_[itr] == JointType::Prismatic)
    {
      // Determine exponential for prismatic joint
      Eigen::Vector3d transl_axis(manipulator_.joint_axes_[itr].head<3>());
      g.block<3,1>(0,3) = jnt_values(itr) * transl_axis;
    }

    // End-effector configuration
    g_base_tool = g_base_tool * g;
  }

  g_base_tool = g_base_tool * manipulator_.gst0_;

  return ErrorCodes::OPERATION_SUCCESS;
}

ErrorCodes KinematicsSolver::getSpatialJacobian(
    const Eigen::VectorXd &jnt_values,
    Eigen::MatrixXd &manip_jac)
{
  //const unsigned int jnt_count = manipulator_.joint_count_;
  //manip_jac = Eigen::Matrix<double,6,jnt_count>::Zero();
  manip_jac.resize(6, manipulator_.joint_count_);
  manip_jac.Zero(6, manipulator_.joint_count_);
  
  Eigen::MatrixXd joint_twists;
  //joint_twists = Eigen::Matrix<double,6,jnt_count>::Zero();
  joint_twists.resize(6, manipulator_.joint_count_);
  joint_twists = Eigen::MatrixXd::Zero(6, manipulator_.joint_count_);

  //std::cout << joint_twists << "\n\n";

  std::vector<Eigen::Matrix4d> g;

  for(unsigned int itr = 0; itr < manipulator_.joint_count_ - 1; itr++)
  {
    g.push_back(Eigen::Matrix<double,4,4>::Identity());
  }

  // Determine joint twists
  for(unsigned int itr = 0; itr < manipulator_.joint_count_; itr++)
  {
    Eigen::Vector3d jnt_axis = manipulator_.joint_axes_[itr].head<3>();
    Eigen::Vector3d jnt_q = manipulator_.joint_q_[itr].head<3>();

    if(manipulator_.joint_types_[itr] == JointType::Revolute)
    {
      //std::cout << "jnt_axis\n" << jnt_axis << "\n\n";
      //std::cout << "jnt_q\n" << jnt_q << "\n\n";
      //std::cout << "jnt_axis_cross\n" << jnt_axis.cross(jnt_q) << "\n\n";
      joint_twists.block<3,1>(0,itr) = -jnt_axis.cross(jnt_q);
      joint_twists.block<3,1>(3,itr) = jnt_axis;
    }
    else if(manipulator_.joint_types_[itr] == JointType::Prismatic)
    {
      joint_twists.block<3,1>(0,itr) = jnt_axis;
    }

    //std::cout << joint_twists << "\n\n";
  }

  // Determine joint twists exponential
  for(unsigned int i = 1; i < manipulator_.joint_count_; i++)
  {
    for(unsigned int j = 0; j < i; j++)
    {
      Eigen::Matrix4d temp_g = Eigen::Matrix4d::Identity();

      if(manipulator_.joint_types_[j] == JointType::Revolute)
      {
        // Determine exponential for revolute joint
        Eigen::Matrix3d rot_mat;
        Eigen::Vector3d rot_axis(manipulator_.joint_axes_[j].head<3>());
        rot_mat = Eigen::AngleAxisd(jnt_values(j), rot_axis);
        temp_g.block<3,3>(0,0) = rot_mat;

        temp_g.block<3,1>(0,3) = (Eigen::Matrix3d::Identity() - rot_mat) *
                            manipulator_.joint_q_[j].head<3>();
      }
      else if(manipulator_.joint_types_[j] == JointType::Prismatic)
      {
        // Determine exponential for prismatic joint
        Eigen::Vector3d transl_axis(manipulator_.joint_axes_[j].head<3>());
        temp_g.block<3,1>(0,3) = jnt_values(j) * transl_axis;
      }

      g[i-1] = g[i-1] * temp_g;
    }
  }

  manip_jac.col(0) = joint_twists.col(0);

  for(unsigned int itr = 1; itr < manipulator_.joint_count_; itr++)
  {
    //std::cout << "g[itr-1] :\n" << g[itr-1] << "\n\n";
    //std::cout << "Adjoint :\n" << getAdjoint(g[itr-1]) << "\n\n";
    //std::cout << "joint_twists.col(itr) :\n" << joint_twists.col(itr) << "\n\n";
    manip_jac.col(itr) = getAdjoint(g[itr-1]) * joint_twists.col(itr);
  }

  return ErrorCodes::OPERATION_SUCCESS;
}

ErrorCodes KinematicsSolver::getResolvedMotionRateControlStep(
    eigen_ext::DualQuat &dq_i,
    eigen_ext::DualQuat &dq_f,
    const Eigen::VectorXd &jnt_values,
    Eigen::VectorXd &jnt_values_increment)
{
  Eigen::Matrix4d g(Eigen::Matrix4d::Identity());
  Eigen::Vector3d p;
  Eigen::Matrix<double,7,1> gamma_i, gamma_f;

  Eigen::Matrix3d I3(Eigen::Matrix3d::Identity());

  Eigen::Vector3d p_i;
  Eigen::Vector4d q_i;

  g = dq_i.getTransform();
  p = g.block<3,1>(0,3);
  gamma_i.head<3>() = p;
  gamma_i.tail<4>() = dq_i.getRealPartVec();

  p_i = gamma_i.head<3>();
  q_i = gamma_i.tail<4>();

  g = dq_f.getTransform();
  p = g.block<3,1>(0,3);
  gamma_f.head<3>() = p;
  gamma_f.tail<4>() = dq_f.getRealPartVec();

  Eigen::MatrixXd s_jac;

  getSpatialJacobian(jnt_values, s_jac);

  Eigen::Matrix<double,3,4> J_1(Eigen::Matrix<double,3,4>::Zero());

  J_1.block<3,1>(0,0) = -q_i.tail<3>();
  J_1(0,1) =  q_i(0);
  J_1(0,2) = -q_i(3);
  J_1(0,3) =  q_i(2);
  J_1(1,1) =  q_i(3);
  J_1(1,2) =  q_i(0);
  J_1(1,3) = -q_i(1);
  J_1(2,1) = -q_i(2);
  J_1(2,2) =  q_i(1);
  J_1(2,3) =  q_i(0);

  Eigen::Matrix<double,6,7> J_2(Eigen::Matrix<double,6,7>::Zero());

  J_2.block<3,3>(0,0) = I3;
  J_2.block<3,4>(0,3) = 2 * getSkewMatrix(p_i) * J_1;
  J_2.block<3,4>(3,3) = 2 * J_1;

  Eigen::MatrixXd jac_pseudo_inv;

  Eigen::MatrixXd temp_mat;
  temp_mat = s_jac * s_jac.transpose();

  // TODO Include invertible check

  jac_pseudo_inv = s_jac.transpose() * temp_mat.inverse();

  Eigen::MatrixXd B;
  B = jac_pseudo_inv * J_2;

  jnt_values_increment = B * (gamma_f - gamma_i);

  return ErrorCodes::OPERATION_SUCCESS;
}

MotionPlanResult::MotionPlanResult() :
  result(MotionPlanReturnCodes::UNKNOWN),
  joint_id(0)
{

}

ErrorCodes KinematicsSolver::getMotionPlan(
    const Eigen::VectorXd &init_jnt_values,
    const Eigen::Matrix4d &g_i,
    const Eigen::Matrix4d &g_f,
    //trajectory_msgs::JointTrajectory &jnt_trajectory);
    std::vector<Eigen::VectorXd> &jnt_values_seq,
    MotionPlanResult &plan_result)
{
  Eigen::VectorXd joint_values_inc;
  Eigen::VectorXd current_joint_values;
  Eigen::VectorXd next_joint_values;

#if DEBUG

  std::time_t now = std::time(0);
  std::tm* timestamp = std::localtime(&now);

  char timestamp_char[100];
  strftime(timestamp_char, 100, timestamp_str.c_str(), timestamp);

  std::string current_log_folder(timestamp_char);
  std::string log_dir_path = log_dir + "/" + current_log_folder;

  std::string timestamp_str_val(timestamp_char);
  std::string log_file_path = log_dir_path + "/" + timestamp_str_val 
                            + "_motion_plan.csv";

  std::ofstream log_file;

  if(mkdir(log_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
  {
    if(errno != EEXIST)
    {
      std::cout << "Error : Cannot create log folders\n";
      std::cout << "Error " << errno << ": " << strerror(errno);
      std::cout.flush();
      log_folder_error = true;
    }
  }

  if(!log_folder_error)
  {
    if(mkdir(log_dir_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
    {
      if(errno != EEXIST)
      {
        std::cout << "Error : Cannot create log folders\n";
        std::cout << "Error " << errno << ": " << strerror(errno);
        std::cout.flush();
        log_folder_error = true;
      }
    }
    
    if(!log_folder_error)
    {
      log_file.open(log_file_path, std::ofstream::out | std::ofstream::app);

      if(!log_file.is_open())
      {
        log_file_error = true;

        std::cout << "Error : Cannot open log file\n";
        std::cout.flush();
      }
    }
  }

#endif

  double pos_dist, rot_dist;

  current_joint_values = init_jnt_values;

  unsigned long int itr_cnt = 0;

  eigen_ext::DualQuat dq_i(g_i);
  eigen_ext::DualQuat dq_f(g_f);

  Eigen::Matrix4d g_current;

  eigen_ext::DualQuat dq_current = dq_i;
  eigen_ext::DualQuat dq_next;

  pos_dist = positionDistance(g_i, g_f);
  rot_dist = rotationDistance(dq_i, dq_f);

  Eigen::MatrixXd joint_limits;
  joint_limits.resize(manipulator_.joint_count_, 2);
  for(int i = 0; i < manipulator_.joint_count_; i++)
  {
    joint_limits(i,0) = manipulator_.joint_limits_[i].lower_limit_;
    joint_limits(i,1) = manipulator_.joint_limits_[i].upper_limit_;
  }

  double beta = 0.5;
  double step_size = beta;

  double tau = 0.01;
  double tau_i = tau;

  Eigen::VectorXd joint_values_delta;

  //trajectory_msgs::JointTrajectoryPoint jnt_trajectory_point;
  //jnt_trajectory.joint_names = manipulator_.joint_names_;
  //jnt_trajectory.points.clear();

  std::vector<double> jnt_values(7,0);

  int convergence_threshold = 1000;

  while((!(pos_dist < 0.0001 && rot_dist < 0.001)) && (itr_cnt < convergence_threshold))
  {


    /*
    if(pos_dist < 0.01 && rot_dist < 0.1)
    {
      tau = 0.25;
    }
    else if(pos_dist < 0.001 && rot_dist < 0.1)
    {
      tau = 0.5;
    }
    else
    {
      tau = tau_i;
    }
    */

    itr_cnt++;

    step_size = beta;

    dq_next = eigen_ext::DualQuat::dualQuatInterpolation(
        dq_current, dq_f, tau);

    if(tau_i < 1)
    {
      Eigen::Matrix4d g_next_temp = dq_next.getTransform();

      double pos_dist_temp = positionDistance(g_current, g_next_temp);

      if(pos_dist_temp < 0.01)
      {
        tau = tau + 0.01;

        dq_next = eigen_ext::DualQuat::dualQuatInterpolation(
            dq_current, dq_f, tau);
      }
    }

    getResolvedMotionRateControlStep(
        dq_current, dq_next, current_joint_values, joint_values_inc);

    if(joint_values_inc.hasNaN())
    {
      //jnt_values_seq.clear();
      std::cout << "\nJoint increments are not finite!\n";
      std::cout.flush();

#if DEBUG
      if(!(log_folder_error || log_file_error))
      {
        for(int temp_itr = 0; temp_itr < joint_values_inc.rows(); temp_itr++)
        {
          log_file << 1000 << ',';
        }
        log_file << '\n';

        log_file << joint_values_inc << '\n';
        
        log_file.flush();
      }
#endif

      plan_result.result = MotionPlanReturnCodes::JACOBIAN_PINV_NOT_FINITE;

      return ErrorCodes::OPERATION_FAILURE;
    }

    int joint_limit_id = 0;

determine_next_angles:

    joint_limit_id = 0;

    joint_values_delta = joint_values_inc * step_size;

    next_joint_values = current_joint_values + joint_values_delta;

    // Check if joint limits are satisfied
    for(int i = 0; i < manipulator_.joint_count_; i++)
    {

      jnt_values[i] = next_joint_values(i);

      if( (next_joint_values(i) > joint_limits(i,0)) &&
          (next_joint_values(i) < joint_limits(i,1)))
      {
        continue;
      }
      else
      {
        joint_limit_id = i + 1;

        double max_val = fabs(joint_values_delta(0));

        for(int j = 1; j < manipulator_.joint_count_; j++)
        {
          if(fabs(joint_values_delta(j)) > max_val)
          {
            max_val = fabs(joint_values_delta(j));
          }
        }

        if(max_val <= 0.0001)
        {
          std::cout << "\nJoint " << joint_limit_id << " limits reached!\n";
          std::cout << next_joint_values.transpose().format(PrintFormat)<<'\n';
          std::cout.flush();

#if DEBUG
          if(!(log_folder_error || log_file_error))
          {
            for(int temp_itr = 0; temp_itr < manipulator_.joint_count_; temp_itr++)
            {
              log_file << -1000 << ',';
            }
            log_file << '\n';

            log_file.flush();
          }
#endif
          plan_result.result = MotionPlanReturnCodes::JOINT_LIMITS_VIOLATED;
          plan_result.joint_id = joint_limit_id;

          return ErrorCodes::JOINT_LIMIT_ERROR;
        }

        step_size = step_size / 10;
        goto determine_next_angles;
      }

    }

    //jnt_trajectory_point.positions = jnt_values;
    //jnt_trajectory.points.push_back(jnt_trajectory_point);

    jnt_values_seq.push_back(next_joint_values);

#if DEBUG
    if(!(log_folder_error || log_file_error))
    {
      log_file << next_joint_values.transpose().format(CSVFormat) << '\n';
      log_file.flush();
    }
#endif

    getFK(next_joint_values, g_current);

    dq_current = eigen_ext::DualQuat::transformationToDualQuat(g_current);

    current_joint_values = next_joint_values;

    pos_dist = positionDistance(g_current, g_f);
    rot_dist = rotationDistance(dq_current, dq_f);
  }

  if(itr_cnt >= convergence_threshold)
  {
    plan_result.result = MotionPlanReturnCodes::PLANNER_NOT_CONVERGING;
    return ErrorCodes::OPERATION_FAILURE;
  }

  plan_result.result = MotionPlanReturnCodes::PLAN_SUCCES;

  return ErrorCodes::OPERATION_SUCCESS;
}

ErrorCodes KinematicsSolver::getEndEffectorTrajectory(
    const Eigen::MatrixXd &jnt_angle_seq,
    std::vector<Eigen::Matrix4d> &g_seq)
{
  g_seq.clear();
  Eigen::Matrix4d ee_pose;

  for(int i = 0; i < jnt_angle_seq.rows(); i++)
  {
    Eigen::VectorXd jnt_angle = jnt_angle_seq.block<1,7>(i, 0).transpose();
    getFK(jnt_angle, ee_pose);
    g_seq.push_back(ee_pose);
  }

  return ErrorCodes::OPERATION_SUCCESS;
}

} // namespace kinlib