/*
 * kinlib::Manipulator class definition
 * Author: Dasharadhan Mahalingam
 */

#include "manipulator.h"

namespace kinlib {

Manipulator::Manipulator(void) { joint_count_ = 0; }

std::vector<JointType> Manipulator::getJointTypes(void) { return joint_types_; }

std::map<std::string, unsigned int>
Manipulator::getJointNameAndIDMapping(void) {
  return joint_name_id_map_;
}

std::vector<Eigen::Vector4d> Manipulator::getJointAxes(void) {
  return joint_axes_;
}

std::vector<Eigen::Vector4d> Manipulator::getJointPositions(void) {
  return joint_q_;
}

Eigen::Matrix4d Manipulator::getReferenceConfiguration(void) { return gst0_; }

unsigned int Manipulator::getNumberOfJoints(void) { return joint_count_; }

ErrorCodes Manipulator::addJoint(const JointType &jnt_type,
                                 const std::string &jnt_name,
                                 const Eigen::Vector4d &jnt_axis,
                                 const Eigen::Vector4d &jnt_q,
                                 const JointLimits &jnt_limits,
                                 const Eigen::Matrix4d &jnt_tip_pose) {
  joint_types_.push_back(jnt_type);
  joint_names_.push_back(jnt_name);

  joint_axes_.push_back(jnt_axis.normalized());

  joint_limits_.push_back(jnt_limits);
  joint_q_.push_back(jnt_q);

  joint_name_id_map_.insert(std::make_pair(jnt_name, joint_count_++));

  joint_tip_pose_.push_back(jnt_tip_pose);

  gst0_ = jnt_tip_pose;

  return OPERATION_SUCCESS;
}

ErrorCodes Manipulator::modifyEndJointTipPose(const Eigen::Matrix4d &g_tip) {
  if (joint_count_ > 0) {
    joint_tip_pose_[joint_count_ - 1] = g_tip;
    setReferenceConfiguration(g_tip);
  } else {
  }

  return OPERATION_SUCCESS;
}

ErrorCodes
Manipulator::setReferenceConfiguration(const Eigen::Matrix4d &g_ref) {
  gst0_ = g_ref;

  return OPERATION_SUCCESS;
}
} // namespace kinlib