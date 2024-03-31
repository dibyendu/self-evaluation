/*
 * DualQuat class definiton
 * Author: Dasharadhan Mahalingam
 */

#include "DualQuat.h"
#include <cassert>
#include <cmath>

#define PI 3.14159265358979

namespace eigen_ext {
Eigen::Vector4d quatProduct(Eigen::Vector4d &q_1, Eigen::Vector4d &q_2) {
  Eigen::Vector4d q_res;

  q_res(0) = (q_1(0) * q_2(0)) - q_1.tail<3>().dot(q_2.tail<3>());
  q_res.tail<3>() = (q_1(0) * q_2.tail<3>()) + (q_2(0) * q_1.tail<3>()) +
                    q_1.tail<3>().cross(q_2.tail<3>());

  return q_res;
}

DualQuat::DualQuat(void) {
  real_part_.w() = 0;
  real_part_.x() = 0;
  real_part_.y() = 0;
  real_part_.z() = 0;

  dual_part_.w() = 0;
  dual_part_.x() = 0;
  dual_part_.y() = 0;
  dual_part_.z() = 0;

  vec_real_part_ << 0, 0, 0, 0;
  vec_dual_part_ << 0, 0, 0, 0;
}

DualQuat::DualQuat(const Eigen::Matrix4d &g) {
  // Determine real part of dual quaternion
  // Eigen::Quaterniond q_rot(g.block<3,3>(0,0));

  Eigen::Vector4d q_rot_vec;

  Eigen::Matrix4d co_eff_mat;
  co_eff_mat << +1, +1, +1, +1, +1, -1, -1, +1, -1, +1, -1, +1, -1, -1, +1, +1;

  Eigen::Vector4d rot_diag_mat;
  rot_diag_mat << g(0, 0), g(1, 1), g(2, 2), 1;

  Eigen::Vector4d q_square = 0.25 * (co_eff_mat * rot_diag_mat);

  int max_index = 0;

  for (int index = 1; index < 4; index++) {
    if (q_square(index) > q_square(max_index)) {
      max_index = index;
    }
  }

  switch (max_index) {
  case 0:
    q_rot_vec(0) = std::sqrt(q_square(0));
    q_rot_vec(1) = (g(2, 1) - g(1, 2)) / (4 * q_rot_vec(0));
    q_rot_vec(2) = (g(0, 2) - g(2, 0)) / (4 * q_rot_vec(0));
    q_rot_vec(3) = (g(1, 0) - g(0, 1)) / (4 * q_rot_vec(0));
    break;
  case 1:
    q_rot_vec(1) = std::sqrt(q_square(1));
    q_rot_vec(0) = (g(2, 1) - g(1, 2)) / (4 * q_rot_vec(1));
    q_rot_vec(2) = (g(0, 1) + g(1, 0)) / (4 * q_rot_vec(1));
    q_rot_vec(3) = (g(0, 2) + g(2, 0)) / (4 * q_rot_vec(1));
    break;
  case 2:
    q_rot_vec(2) = std::sqrt(q_square(2));
    q_rot_vec(0) = (g(0, 2) - g(2, 0)) / (4 * q_rot_vec(2));
    q_rot_vec(1) = (g(0, 1) + g(1, 0)) / (4 * q_rot_vec(2));
    q_rot_vec(3) = (g(1, 2) + g(2, 1)) / (4 * q_rot_vec(2));
    break;
  case 3:
    q_rot_vec(3) = std::sqrt(q_square(3));
    q_rot_vec(0) = (g(1, 0) - g(0, 1)) / (4 * q_rot_vec(3));
    q_rot_vec(1) = (g(0, 2) + g(2, 0)) / (4 * q_rot_vec(3));
    q_rot_vec(2) = (g(1, 2) + g(2, 1)) / (4 * q_rot_vec(3));
    break;
  }

  // Normalize quaternion
  q_rot_vec.normalize();

  // Determine dual part of dual quaternion
  Eigen::Vector4d q_pos_vec;

  Eigen::Vector4d temp_vec;

  temp_vec << 0, g(0, 3), g(1, 3), g(2, 3);

  q_pos_vec = quatProduct(temp_vec, q_rot_vec);

  /*
  q_pos_vec(0) = - temp_vec.dot(q_rot_vec);
  q_pos_vec.tail<3>() = (q_rot_vec(0) * temp_vec.tail<3>()) +
                          temp_vec.tail<3>().cross(q_rot_vec.tail<3>());
  */

  q_pos_vec = q_pos_vec / 2;

  real_part_.w() = q_rot_vec(0);
  real_part_.x() = q_rot_vec(1);
  real_part_.y() = q_rot_vec(2);
  real_part_.z() = q_rot_vec(3);

  dual_part_.w() = q_pos_vec(0);
  dual_part_.x() = q_pos_vec(1);
  dual_part_.y() = q_pos_vec(2);
  dual_part_.z() = q_pos_vec(3);

  updateValues();
}

DualQuat::DualQuat(const Eigen::Vector4d &real_part,
                   const Eigen::Vector4d &dual_part) {
  setValues(real_part, dual_part);
}

void DualQuat::setValues(const Eigen::Quaterniond &real_part,
                         const Eigen::Quaterniond &dual_part) {
  real_part_.w() = real_part.w();
  real_part_.x() = real_part.x();
  real_part_.y() = real_part.y();
  real_part_.z() = real_part.z();

  // vec_real_part_ << real_part.w(), real_part.x(), real_part.y(),
  // real_part.z();

  dual_part_.w() = dual_part.w();
  dual_part_.x() = dual_part.x();
  dual_part_.y() = dual_part.y();
  dual_part_.z() = dual_part.z();

  // vec_dual_part_ << dual_part.w(), dual_part.x(), dual_part.y(),
  // dual_part.z();

  updateValues();
}

void DualQuat::setValues(const Eigen::Vector4d &real_part,
                         const Eigen::Vector4d &dual_part) {
  real_part_.w() = real_part(0);
  real_part_.x() = real_part(1);
  real_part_.y() = real_part(2);
  real_part_.z() = real_part(3);

  dual_part_.w() = dual_part(0);
  dual_part_.x() = dual_part(1);
  dual_part_.y() = dual_part(2);
  dual_part_.z() = dual_part(3);

  updateValues();
  /*
  for(int itr = 0; itr < 4; itr++)
  {
    vec_real_part_(itr) = real_part(itr);
    vec_dual_part_(itr) = dual_part(itr);
  }
  */
}

void DualQuat::updateValues() {
  vec_real_part_ << real_part_.w(), real_part_.x(), real_part_.y(),
      real_part_.z();

  vec_dual_part_ << dual_part_.w(), dual_part_.x(), dual_part_.y(),
      dual_part_.z();
}

Eigen::Quaterniond DualQuat::getRealPart() { return real_part_; }

Eigen::Vector4d DualQuat::getRealPartVec() {
  Eigen::Vector4d ret = vec_real_part_;
  return ret;
}

Eigen::Quaterniond DualQuat::getDualPart() { return dual_part_; }

Eigen::Vector4d DualQuat::getDualPartVec() {
  Eigen::Vector4d ret = vec_dual_part_;
  return ret;
}

DualQuat DualQuat::getConjugate() {
  Eigen::Vector4d dq_real, dq_dual;
  dq_real = getRealPartVec();
  dq_dual = getDualPartVec();

  dq_real.tail<3>() = -dq_real.tail<3>();
  dq_dual.tail<3>() = -dq_dual.tail<3>();

  DualQuat dq_conjugate(dq_real, dq_dual);

  return dq_conjugate;
}

DualQuat DualQuat::dualQuatProduct(DualQuat &dq_1, DualQuat &dq_2) {
  Eigen::Vector4d dq_real, dq_dual;

  Eigen::Vector4d dq_1_vec_real = dq_1.getRealPartVec();
  Eigen::Vector4d dq_1_vec_dual = dq_1.getDualPartVec();
  Eigen::Vector4d dq_2_vec_real = dq_2.getRealPartVec();
  Eigen::Vector4d dq_2_vec_dual = dq_2.getDualPartVec();

  dq_real = quatProduct(dq_1_vec_real, dq_2_vec_real);
  dq_dual = quatProduct(dq_1_vec_dual, dq_2_vec_real) +
            quatProduct(dq_1_vec_real, dq_2_vec_dual);

  DualQuat dq_product(dq_real, dq_dual);

  return dq_product;
}

DualQuat DualQuat::raiseToPower(const double &pwr) {
  Eigen::Vector4d real_part = getRealPartVec();
  Eigen::Vector4d dual_part = getDualPartVec();

  double theta;

  if (real_part(0) <= -1.0) {
    theta = M_PI;
  } else if (real_part(0) >= 1.0) {
    theta = 0;
  } else {
    theta = 2 * std::acos(real_part(0));
  }

  // Check if angle is correct
  assert(!std::isnan(theta));

  // Wrap theta in range [-pi,pi]
  theta = std::fmod(theta + PI, (2 * PI));

  if (theta < 0) {
    theta = theta + (2 * PI);
  }
  theta = theta - PI;

  Eigen::Matrix4d g = getTransform();
  Eigen::Vector3d p = g.block<3, 1>(0, 3);

  Eigen::Vector4d res_real_part;
  Eigen::Vector4d res_dual_part;

  if (theta == 0) {
    Eigen::Vector3d v = getDualPartVec().tail<3>();

    double d = 2 * v.norm();

    v = v / (d / 2);

    res_real_part(0) = std::cos(pwr * (theta / 2));
    res_real_part.tail<3>() = v * std::sin(pwr * (theta / 2));

    res_dual_part(0) = -(pwr * d / 2) * std::sin(pwr * (theta / 2));
    res_dual_part.tail<3>() = (pwr * d / 2) * std::cos(pwr * (theta / 2)) * v;
  } else {
    Eigen::Vector3d rp = getRealPartVec().tail<3>();
    Eigen::Vector3d u = rp / rp.norm();

    double d = p.dot(u);

    Eigen::Vector3d m =
        (p.cross(u) + ((p - (d * u)) / std::tan(theta / 2))) / 2;

    res_real_part(0) = std::cos(pwr * (theta / 2));
    res_real_part.tail<3>() = std::sin(pwr * (theta / 2)) * u;

    res_dual_part(0) = -(pwr * d / 2) * std::sin(pwr * (theta / 2));
    res_dual_part.tail<3>() = (std::sin(pwr * (theta / 2)) * m) +
                              (((pwr * d * std::cos(pwr * theta / 2)) / 2) * u);
  }

  DualQuat dq_result(res_real_part, res_dual_part);

  return dq_result;
}

DualQuat DualQuat::dualQuatInterpolation(DualQuat &dq_i, DualQuat &dq_f,
                                         double &t) {
  DualQuat dq_result;

  DualQuat dq_i_conj = dq_i.getConjugate();

  dq_result = dualQuatProduct(dq_i_conj, dq_f);
  dq_result = dq_result.raiseToPower(t);
  dq_result = dualQuatProduct(dq_i, dq_result);

  return dq_result;
}

Eigen::Matrix3d DualQuat::getRotation(void) {
  return real_part_.toRotationMatrix();
}

Eigen::Matrix4d DualQuat::getTransform(void) {
  Eigen::Matrix4d g;

  g.setIdentity();
  g.block<3, 3>(0, 0) = getRotation();

  Eigen::Vector4d real_conj = vec_real_part_;
  real_conj.tail<3>() = -real_conj.tail<3>();

  Eigen::Vector4d q_prod = quatProduct(vec_dual_part_, real_conj);

  Eigen::Vector4d p = 2 * q_prod;

  g.block<3, 1>(0, 3) = p.tail<3>();

  return g;
}

DualQuat DualQuat::transformationToDualQuat(const Eigen::Matrix4d &g) {
  DualQuat ret(g);
  return ret;
}
} // namespace eigen_ext