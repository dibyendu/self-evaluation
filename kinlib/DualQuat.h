/*
 * DualQuat class declaration
 */

/* Author: Dasharadhan Mahalingam */

#pragma once

#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace eigen_ext {
class DualQuat {
public:
  /*!
    \brief  Constructor to initialize dual quaternion
  */
  DualQuat(void);

  /*!
    \brief  Constructor to initialize dual quaternion

    \param  g   Rigid body transformation
  */
  DualQuat(const Eigen::Matrix4d &g);

  /*!
    \brief  Constructor to initialize dual quaternion

    \param  real_part   Vector representing real part of dual quaternion
    \param  dual_part   Vector representing dual part of dual quaternion
  */
  DualQuat(const Eigen::Vector4d &real_part, const Eigen::Vector4d &dual_part);

  /*!
    \brief  Assign values to real and dual part of the dual quaternion
  */
  void setValues(const Eigen::Quaterniond &real_part,
                 const Eigen::Quaterniond &dual_part);

  /*!
    \brief  Assign values to real and dual part of the dual quaternion
  */
  void setValues(const Eigen::Vector4d &real_part,
                 const Eigen::Vector4d &dual_part);

  /*!
    \brief  To update the vector copies of the quaternions
  */
  void updateValues(void);

  /*!
    \brief  Get real part of dual quaternion
  */
  Eigen::Quaterniond getRealPart(void);

  /*!
    \brief  Get real part of dual quaternion as vectors
  */
  Eigen::Vector4d getRealPartVec(void);

  /*!
    \brief  Get dual part of dual quaternion
  */
  Eigen::Quaterniond getDualPart(void);

  /*!
    \brief  Get dual part of dual quaternion as vectors
  */
  Eigen::Vector4d getDualPartVec(void);

  /*!
    \brief  Get conjugate of dual quaternion
  */
  DualQuat getConjugate(void);

  /*!
    \brief  Dual Quaternion Product

    \param  dq1  Dual Quaternion object
    \param  dq2  Dual Quaternion object
  */
  static DualQuat dualQuatProduct(DualQuat &dq1, DualQuat &dq2);

  /*!
    \brief  Dual Quaternion raised to a power

    \param  pwr   Exponential to which the dual quaternion needs to be
                  raised to
  */
  DualQuat raiseToPower(const double &pwr);

  /*!
    \brief  Dual Quaternion interpolation

    \param  dq_i    Initial dual quaternion
    \param  dq_f    Final dual quaternion
    \param  t       Interpolation parameter (Takes values from 0 to 1)
  */
  static DualQuat dualQuatInterpolation(DualQuat &dq_i, DualQuat &dq_f,
                                        double &t);

  /*!
    \brief  Get rotation part of the transformation represented by the dual
            quaternion
  */
  Eigen::Matrix3d getRotation(void);

  /*!
    \brief  Dual quaternion to rigid body transformation
  */
  Eigen::Matrix4d getTransform(void);

  /*!
    \brief  Convert rigid body transform into unit dual quaternion

    \param  g   Transformation which needs to be converted into
                unit dual quaternion representation
  */
  static DualQuat transformationToDualQuat(const Eigen::Matrix4d &g);

private:
  /*!
    \brief  Real part of dual quaterion
  */
  Eigen::Quaterniond real_part_;
  Eigen::Vector4d vec_real_part_;

  /*!
    \brief  Dual part of dual quaternion
  */
  Eigen::Quaterniond dual_part_;
  Eigen::Vector4d vec_dual_part_;
};
} // namespace eigen_ext