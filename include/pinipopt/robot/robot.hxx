#ifndef PINIPOPT_ROBOT_HXX_
#define PINIPOPT_ROBOT_HXX_

#include "pinipopt/robot/robot.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"

#include <stdexcept>
#include <assert.h>

namespace pinipopt {

inline Robot::Robot(const std::string& path_to_urdf)
  : model_(),
    data_(model_),
    floating_base_(),
    fjoint_(),
    dimq_(0),
    dimv_(0),
    dimu_(0) {
  pinocchio::urdf::buildModel(path_to_urdf, model_);
  data_ = pinocchio::Data(model_);
  fjoint_ = pinocchio::container::aligned_vector<pinocchio::Force>(
                 model_.joints.size(), pinocchio::Force::Zero());
  floating_base_ = FloatingBase(model_);
  dimq_ = model_.nq;
  dimv_ = model_.nv;
  dimu_ = model_.nv - floating_base_.dim_passive();
}


inline Robot::Robot()
  : model_(),
    data_(model_),
    floating_base_(),
    fjoint_(),
    dimq_(0),
    dimv_(0),
    dimu_(0) {
}


inline Robot::~Robot() {
}


template <typename TangentVectorType, typename ConfigVectorType>
inline void Robot::integrateConfiguration(
    const Eigen::MatrixBase<TangentVectorType>& v, 
    const double integration_length, 
    const Eigen::MatrixBase<ConfigVectorType>& q) const {
  assert(v.size() == dimv_);
  assert(q.size() == dimq_);
  if (floating_base_.has_floating_base()) {
    const Eigen::VectorXd q_tmp = q;
    pinocchio::integrate(model_, q_tmp, integration_length*v, 
                         const_cast<Eigen::MatrixBase<ConfigVectorType>&>(q));
  }
  else {
    (const_cast<Eigen::MatrixBase<ConfigVectorType>&>(q)).noalias() 
        += integration_length * v;
  }
}


template <typename ConfigVectorType1, typename TangentVectorType,  
          typename ConfigVectorType2>
inline void Robot::integrateConfiguration(
    const Eigen::MatrixBase<ConfigVectorType1>& q, 
    const Eigen::MatrixBase<TangentVectorType>& v, 
    const double integration_length, 
    const Eigen::MatrixBase<ConfigVectorType2>& q_integrated) const {
  assert(q.size() == dimq_);
  assert(v.size() == dimv_);
  assert(q_integrated.size() == dimq_);
  if (floating_base_.has_floating_base()) {
    pinocchio::integrate(
        model_, q, integration_length*v, 
        const_cast<Eigen::MatrixBase<ConfigVectorType2>&>(q_integrated));
  }
  else {
    (const_cast<Eigen::MatrixBase<ConfigVectorType2>&>(q_integrated)).noalias() 
        = q + integration_length * v;
  }
}


template <typename ConfigVectorType1, typename ConfigVectorType2, 
          typename TangentVectorType>
inline void Robot::subtractConfiguration(
    const Eigen::MatrixBase<ConfigVectorType1>& q_plus, 
    const Eigen::MatrixBase<ConfigVectorType2>& q_minus,
    const Eigen::MatrixBase<TangentVectorType>& difference) const {
  assert(q_plus.size() == dimq_);
  assert(q_minus.size() == dimq_);
  assert(difference.size() == dimv_);
  if (floating_base_.has_floating_base()) {
    pinocchio::difference(
        model_, q_minus, q_plus, 
        const_cast<Eigen::MatrixBase<TangentVectorType>&>(difference));
  }
  else {
    const_cast<Eigen::MatrixBase<TangentVectorType>&>(difference) 
        = q_plus - q_minus;
  }
}


template <typename ConfigVectorType1, typename ConfigVectorType2, 
          typename MatrixType>
inline void Robot::dSubtractdConfigurationPlus(
    const Eigen::MatrixBase<ConfigVectorType1>& q_plus,
    const Eigen::MatrixBase<ConfigVectorType2>& q_minus,
    const Eigen::MatrixBase<MatrixType>& dSubtract_dqplus) const {
  assert(q_plus.size() == dimq_);
  assert(q_minus.size() == dimq_);
  assert(dSubtract_dqplus.rows() == dimv_);
  assert(dSubtract_dqplus.cols() == dimv_);
  pinocchio::dDifference(
      model_, q_minus, q_plus, 
      const_cast<Eigen::MatrixBase<MatrixType>&>(dSubtract_dqplus),
      pinocchio::ARG1);
}


template <typename ConfigVectorType1, typename ConfigVectorType2, 
          typename MatrixType>
inline void Robot::dSubtractdConfigurationMinus(
    const Eigen::MatrixBase<ConfigVectorType1>& q_plus,
    const Eigen::MatrixBase<ConfigVectorType2>& q_minus,
    const Eigen::MatrixBase<MatrixType>& dSubtract_dqminus) const {
  assert(q_plus.size() == dimq_);
  assert(q_minus.size() == dimq_);
  assert(dSubtract_dqminus.rows() == dimv_);
  assert(dSubtract_dqminus.cols() == dimv_);
  pinocchio::dDifference(
      model_, q_minus, q_plus, 
      const_cast<Eigen::MatrixBase<MatrixType>&>(dSubtract_dqminus),
      pinocchio::ARG0);
}


inline const Eigen::Vector3d& Robot::framePosition(const int frame_id) const {
  return data_.oMf[frame_id].translation();
}


inline const Eigen::Matrix3d& Robot::frameRotation(const int frame_id) const {
  return data_.oMf[frame_id].rotation();
}


inline const pinocchio::SE3& Robot::framePlacement(const int frame_id) const {
  return data_.oMf[frame_id];
}


template <typename MatrixType>
inline void Robot::getFrameJacobian(const int frame_id, 
                                    const Eigen::MatrixBase<MatrixType>& J) {
  assert(J.rows() == 6);
  assert(J.cols() == dimv_);
  pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL, 
                              const_cast<Eigen::MatrixBase<MatrixType>&>(J));
}


template <typename ConfigVectorType, typename TangentVectorType1, 
          typename TangentVectorType2>
inline void Robot::updateKinematics(
    const Eigen::MatrixBase<ConfigVectorType>& q, 
    const Eigen::MatrixBase<TangentVectorType1>& v, 
    const Eigen::MatrixBase<TangentVectorType2>& a) {
  assert(q.size() == dimq_);
  assert(v.size() == dimv_);
  assert(a.size() == dimv_);
  pinocchio::forwardKinematics(model_, data_, q, v, a);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeForwardKinematicsDerivatives(model_, data_, q, v, a);
}


template <typename ConfigVectorType, typename TangentVectorType>
inline void Robot::updateKinematics(
    const Eigen::MatrixBase<ConfigVectorType>& q, 
    const Eigen::MatrixBase<TangentVectorType>& v) {
  assert(q.size() == dimq_);
  assert(v.size() == dimv_);
  pinocchio::forwardKinematics(model_, data_, q, v);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeForwardKinematicsDerivatives(model_, data_, q, v, 
                                                 Eigen::VectorXd::Zero(dimv_));
}


template <typename ConfigVectorType, typename TangentVectorType1, 
          typename TangentVectorType2, typename TangentVectorType3>
inline void Robot::RNEA(const Eigen::MatrixBase<ConfigVectorType>& q, 
                        const Eigen::MatrixBase<TangentVectorType1>& v, 
                        const Eigen::MatrixBase<TangentVectorType2>& a, 
                        const Eigen::MatrixBase<TangentVectorType3>& tau) {
  assert(q.size() == dimq_);
  assert(v.size() == dimv_);
  assert(a.size() == dimv_);
  assert(tau.size() == dimv_);
  const_cast<Eigen::MatrixBase<TangentVectorType3>&>(tau)
        = pinocchio::rnea(model_, data_, q, v, a);
}


template <typename ConfigVectorType, typename TangentVectorType1, 
          typename TangentVectorType2, typename MatrixType1, 
          typename MatrixType2, typename MatrixType3>
inline void Robot::RNEADerivatives(
    const Eigen::MatrixBase<ConfigVectorType>& q, 
    const Eigen::MatrixBase<TangentVectorType1>& v, 
    const Eigen::MatrixBase<TangentVectorType2>& a, 
    const Eigen::MatrixBase<MatrixType1>& dRNEA_partial_dq, 
    const Eigen::MatrixBase<MatrixType2>& dRNEA_partial_dv, 
    const Eigen::MatrixBase<MatrixType3>& dRNEA_partial_da) {
  assert(q.size() == dimq_);
  assert(v.size() == dimv_);
  assert(a.size() == dimv_);
  assert(dRNEA_partial_dq.cols() == dimv_);
  assert(dRNEA_partial_dq.rows() == dimv_);
  assert(dRNEA_partial_dv.cols() == dimv_);
  assert(dRNEA_partial_dv.rows() == dimv_);
  assert(dRNEA_partial_da.cols() == dimv_);
  assert(dRNEA_partial_da.rows() == dimv_);
  pinocchio::computeRNEADerivatives(
      model_, data_, q, v, a, 
      const_cast<Eigen::MatrixBase<MatrixType1>&>(dRNEA_partial_dq),
      const_cast<Eigen::MatrixBase<MatrixType2>&>(dRNEA_partial_dv),
      const_cast<Eigen::MatrixBase<MatrixType3>&>(dRNEA_partial_da));
  (const_cast<Eigen::MatrixBase<MatrixType3>&>(dRNEA_partial_da)) 
      .template triangularView<Eigen::StrictlyLower>() 
      = (const_cast<Eigen::MatrixBase<MatrixType3>&>(dRNEA_partial_da)).transpose()
          .template triangularView<Eigen::StrictlyLower>();
}


template <typename ConfigVectorType, typename TangentVectorType1, 
          typename TangentVectorType2, typename MatrixType1, 
          typename MatrixType2, typename MatrixType3>
inline void Robot::ABADerivatives(
    const Eigen::MatrixBase<ConfigVectorType>& q, 
    const Eigen::MatrixBase<TangentVectorType1>& v, 
    const Eigen::MatrixBase<TangentVectorType2>& tau, 
    const Eigen::MatrixBase<MatrixType1>& dABA_partial_dq, 
    const Eigen::MatrixBase<MatrixType2>& dABA_partial_dv, 
    const Eigen::MatrixBase<MatrixType3>& dABA_partial_dtau) {
  assert(q.size() == dimq_);
  assert(v.size() == dimv_);
  assert(tau.size() == dimv_);
  assert(dABA_partial_dq.cols() == dimv_);
  assert(dABA_partial_dq.rows() == dimv_);
  assert(dABA_partial_dv.cols() == dimv_);
  assert(dABA_partial_dv.rows() == dimv_);
  assert(dABA_partial_dtau.cols() == dimv_);
  assert(dABA_partial_dtau.rows() == dimv_);
  pinocchio::computeABADerivatives(
      model_, data_, q, v, tau, 
      const_cast<Eigen::MatrixBase<MatrixType1>&>(dABA_partial_dq),
      const_cast<Eigen::MatrixBase<MatrixType2>&>(dABA_partial_dv),
      const_cast<Eigen::MatrixBase<MatrixType3>&>(dABA_partial_dtau));
  (const_cast<Eigen::MatrixBase<MatrixType3>&>(dABA_partial_dtau)) 
      .template triangularView<Eigen::StrictlyLower>() 
      = (const_cast<Eigen::MatrixBase<MatrixType3>&>(dABA_partial_dtau)).transpose()
          .template triangularView<Eigen::StrictlyLower>();
}


template <typename MatrixType1, typename MatrixType2>
inline void Robot::computeMinv(const Eigen::MatrixBase<MatrixType1>& M, 
                               const Eigen::MatrixBase<MatrixType2>& Minv) {
  assert(M.rows() == dimv_);
  assert(M.cols() == dimv_);
  assert(Minv.rows() == dimv_);
  assert(Minv.cols() == dimv_);
  data_.M = M;
  pinocchio::cholesky::decompose(model_, data_);
  pinocchio::cholesky::computeMinv(
      model_, data_, const_cast<Eigen::MatrixBase<MatrixType2>&>(Minv));
}


template <typename ConfigVectorType, typename TangentVectorType1, 
          typename TangentVectorType2, typename TangentVectorType3,
          typename TangentVectorType4>
inline void Robot::stateEquation(
    const Eigen::MatrixBase<ConfigVectorType>& q, 
    const Eigen::MatrixBase<TangentVectorType1>& v, 
    const Eigen::MatrixBase<TangentVectorType2>& tau, 
    const Eigen::MatrixBase<TangentVectorType3>& dq, 
    const Eigen::MatrixBase<TangentVectorType4>& dv) {
  assert(q.size() == dimq_);
  assert(v.size() == dimv_);
  assert(tau.size() == dimv_);
  assert(dq.size() == dimv_);
  assert(dv.size() == dimv_);
  const_cast<Eigen::MatrixBase<TangentVectorType3>&> (dq) = v;
  const_cast<Eigen::MatrixBase<TangentVectorType3>&> (dv)
      = pinocchio::aba(model_, data_, q, v, tau);
}


template <typename ConfigVectorType>
inline void Robot::generateFeasibleConfiguration(
    const Eigen::MatrixBase<ConfigVectorType>& q) const {
  assert(q.size() == dimq_);
  Eigen::VectorXd q_min = model_.lowerPositionLimit;
  Eigen::VectorXd q_max = model_.upperPositionLimit;
  if (floating_base_.has_floating_base()) {
    q_min.head(7) = - Eigen::VectorXd::Ones(7);
    q_max.head(7) = Eigen::VectorXd::Ones(7);
  }
  const_cast<Eigen::MatrixBase<ConfigVectorType>&> (q) 
      = pinocchio::randomConfiguration(model_, q_min, q_max);
}


template <typename ConfigVectorType>
inline void Robot::normalizeConfiguration(
    const Eigen::MatrixBase<ConfigVectorType>& q) const {
  assert(q.size() == dimq_);
  if (floating_base_.has_floating_base()) {
    if (q.template segment<4>(3).squaredNorm() 
          <= std::numeric_limits<double>::epsilon()) {
      (const_cast<Eigen::MatrixBase<ConfigVectorType>&> (q)).coeffRef(3) = 1;
    }
    pinocchio::normalize(model_, 
                         const_cast<Eigen::MatrixBase<ConfigVectorType>&>(q));
  }
}


inline int Robot::dimq() const {
  return dimq_;
}


inline int Robot::dimv() const {
  return dimv_;
}


inline int Robot::dimu() const {
  return dimu_;
}


inline int Robot::dim_passive() const {
  return floating_base_.dim_passive();
}


inline bool Robot::has_floating_base() const {
  return floating_base_.has_floating_base();
}

} // namespace idocp

#endif // PINIPOPT_ROBOT_HXX_ 