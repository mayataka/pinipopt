#ifndef IPOPT_PINO_ROBOT_HPP_
#define IPOPT_PINO_ROBOT_HPP_

#include <string>
#include <vector>

#include "Eigen/Core"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/container/aligned-vector.hpp"
#include "pinocchio/spatial/force.hpp"

#include "ipopt-pino/robot/floating_base.hpp"


namespace ipoptpino {

///
/// @class Robot
/// @brief Dynamics and kinematics model of robots. Wraps pinocchio::Model and 
/// pinocchio::Data.
///
class Robot {
public:
  ///
  /// @brief Build the robot model and data from URDF. The model is assumed to
  /// have no contacts with the environment.
  /// @param[in] path_to_urdf Path to the URDF file.
  ///
  Robot(const std::string& path_to_urdf);

  ///
  /// @brief Default constructor. 
  ///
  Robot();

  ///
  /// @brief Destructor. 
  ///
  ~Robot();

  ///
  /// @brief Use default copy constructor. 
  ///
  Robot(const Robot&) = default;

  ///
  /// @brief Use default copy assign operator. 
  ///
  Robot& operator=(const Robot&) = default;

  ///
  /// @brief Use default move constructor. 
  ///
  Robot(Robot&&) noexcept = default;

  ///
  /// @brief Use default move assign operator. 
  ///
  Robot& operator=(Robot&&) noexcept = default;

  ///
  /// @brief Integrates the generalized velocity by integration_length * v. 
  // The configuration q is then incremented.
  /// @param[in, out] q Configuration. Size must be Robot::dimq().
  /// @param[in] v Generalized velocity. Size must be Robot::dimv().
  /// @param[in] integration_length The length of the integration.
  ///
  template <typename TangentVectorType, typename ConfigVectorType>
  void integrateConfiguration(
      const Eigen::MatrixBase<TangentVectorType>& v, 
      const double integration_length, 
      const Eigen::MatrixBase<ConfigVectorType>& q) const;

  ///
  /// @brief Integrates the generalized velocity by integration_length * v. 
  /// @param[in] q Configuration. Size must be Robot::dimq().
  /// @param[in] v Generalized velocity. Size must be Robot::dimv().
  /// @param[in] integration_length The length of the integration.
  /// @param[out] q_integrated Resultant configuration. Size must be 
  /// Robot::dimq().
  ///
  template <typename ConfigVectorType1, typename TangentVectorType,  
            typename ConfigVectorType2>
  void integrateConfiguration(
      const Eigen::MatrixBase<ConfigVectorType1>& q, 
      const Eigen::MatrixBase<TangentVectorType>& v, 
      const double integration_length, 
      const Eigen::MatrixBase<ConfigVectorType2>& q_integrated) const;

  ///
  /// @brief Computes q_plus - q_minus at the tangent space. 
  /// @param[in] q_plus Configuration. Size must be Robot::dimq().
  /// @param[in] q_minus Configuration. Size must be Robot::dimq().
  /// @param[out] difference Result. Size must be Robot::dimv().
  ///
  template <typename ConfigVectorType1, typename ConfigVectorType2, 
            typename TangentVectorType>
  void subtractConfiguration(
      const Eigen::MatrixBase<ConfigVectorType1>& q_plus, 
      const Eigen::MatrixBase<ConfigVectorType2>& q_minus,
      const Eigen::MatrixBase<TangentVectorType>& difference) const;

  ///
  /// @brief Computes the partial derivative of the function of q_plus - q_minus 
  /// with respect to q_plus at the tangent space. 
  /// @param[in] q_plus Configuration. Size must be Robot::dimq().
  /// @param[in] q_minus Configuration. Size must be Robot::dimq().
  /// @param[out] dSubtract_dqplus The resultant partial derivative. 
  /// Size must be Robot::dimv() x Robot::dimv().
  ///
  template <typename ConfigVectorType1, typename ConfigVectorType2, 
            typename MatrixType>
  inline void dSubtractdConfigurationPlus(
      const Eigen::MatrixBase<ConfigVectorType1>& q_plus,
      const Eigen::MatrixBase<ConfigVectorType2>& q_minus,
      const Eigen::MatrixBase<MatrixType>& dSubtract_dqplus) const;

  ///
  /// @brief Computes the partial derivative of the function of q_plus - q_minus 
  /// with respect to q_minus at the tangent space. 
  /// @param[in] q_plus Configuration. Size must be Robot::dimq().
  /// @param[in] q_minus Configuration. Size must be Robot::dimq().
  /// @param[out] dSubtract_dqminus The resultant partial derivative. 
  /// Size must be Robot::dimv() x Robot::dimv().
  ///
  template <typename ConfigVectorType1, typename ConfigVectorType2, 
            typename MatrixType>
  void dSubtractdConfigurationMinus(
      const Eigen::MatrixBase<ConfigVectorType1>& q_plus,
      const Eigen::MatrixBase<ConfigVectorType2>& q_minus,
      const Eigen::MatrixBase<MatrixType>& dSubtract_dqminus) const;

  ///
  /// @brief Updates the kinematics of the robot. The frame placements, frame 
  /// velocity, frame acceleration, and the relevant Jacobians are calculated. 
  /// @param[in] q Configuration. Size must be Robot::dimq().
  /// @param[in] v Generalized velocity. Size must be Robot::dimv().
  /// @param[in] a Generalized acceleration. Size must be Robot::dimv().
  ///
  template <typename ConfigVectorType, typename TangentVectorType1, 
            typename TangentVectorType2>
  void updateKinematics(const Eigen::MatrixBase<ConfigVectorType>& q, 
                        const Eigen::MatrixBase<TangentVectorType1>& v, 
                        const Eigen::MatrixBase<TangentVectorType2>& a);

  template <typename ConfigVectorType, typename TangentVectorType>
  void updateKinematics(const Eigen::MatrixBase<ConfigVectorType>& q, 
                        const Eigen::MatrixBase<TangentVectorType>& v);

  ///
  /// @brief Returns the position of the frame. Before calling this function, 
  /// updateKinematics() must be called.
  /// @param[in] frame_id Index of the frame.
  /// @return Const reference to the position of the frame.
  ///
  const Eigen::Vector3d& framePosition(const int frame_id) const;

  ///
  /// @brief Returns the rotation matrix of the frame. Before calling this  
  /// function, updateKinematics() must be called.
  /// @param[in] frame_id Index of the frame.
  /// @return Const reference to the rotation matrix of the frame.
  ///
  const Eigen::Matrix3d& frameRotation(const int frame_id) const;

  ///
  /// @brief Returns the SE(3) of the frame. Before calling this function, 
  /// updateKinematics() must be called.
  /// @param[in] frame_id Index of the frame.
  /// @return Const reference to the SE(3) of the frame.
  ///
  const pinocchio::SE3& framePlacement(const int frame_id) const;

  ///
  /// @brief Computes the frame Jacobian of the position. Before calling this  
  /// function, updateKinematics() must be called.
  /// @param[in] frame_id Index of the frame.
  /// @param[out] J Jacobian. Size must be 6 x Robot::dimv().
  ///
  template <typename MatrixType>
  void getFrameJacobian(const int frame_id, 
                        const Eigen::MatrixBase<MatrixType>& J);

  ///
  /// @brief Computes inverse dynamics, i.e., generalized torques corresponding 
  /// to the given configuration, velocity, acceleration, and contact forces. 
  /// If the robot has contacts, update contact forces by calling 
  /// setContactForces().
  /// @param[in] q Configuration. Size must be Robot::dimq().
  /// @param[in] v Generalized velocity. Size must be Robot::dimv().
  /// @param[in] a Generalized acceleration. Size must be Robot::dimv().
  /// @param[out] tau Generalized torques for fully actuated system. Size must 
  /// be Robot::dimv().
  ///
  template <typename ConfigVectorType, typename TangentVectorType1, 
            typename TangentVectorType2, typename TangentVectorType3>
  void RNEA(const Eigen::MatrixBase<ConfigVectorType>& q, 
            const Eigen::MatrixBase<TangentVectorType1>& v, 
            const Eigen::MatrixBase<TangentVectorType2>& a, 
            const Eigen::MatrixBase<TangentVectorType3>& tau);

  ///
  /// @brief Computes Computes the partial dervatives of the function of 
  /// inverse dynamics with respect to the given configuration, velocity, and
  /// acceleration. If the robot has contacts, update contact forces by 
  /// calling setContactForces().
  /// @param[in] q Configuration. Size must be Robot::dimq().
  /// @param[in] v Generalized velocity. Size must be Robot::dimv().
  /// @param[in] a Generalized acceleration. Size must be Robot::dimv().
  /// @param[out] dRNEA_partial_dq The partial derivative of inverse dynamics 
  /// with respect to the configuration. The size must be 
  /// Robot::dimv() x Robot::dimv().
  /// @param[out] dRNEA_partial_dv The partial derivative of inverse dynamics 
  /// with respect to the velocity. The size must be 
  /// Robot::dimv() x Robot::dimv().
  /// @param[out] dRNEA_partial_da The partial derivative of inverse dynamics 
  /// with respect to the acceleration. The size must be 
  /// Robot::dimv() x Robot::dimv().
  ///   
  template <typename ConfigVectorType, typename TangentVectorType1, 
            typename TangentVectorType2, typename MatrixType1, 
            typename MatrixType2, typename MatrixType3>
  void RNEADerivatives(const Eigen::MatrixBase<ConfigVectorType>& q, 
                       const Eigen::MatrixBase<TangentVectorType1>& v, 
                       const Eigen::MatrixBase<TangentVectorType2>& a,
                       const Eigen::MatrixBase<MatrixType1>& dRNEA_partial_dq, 
                       const Eigen::MatrixBase<MatrixType2>& dRNEA_partial_dv, 
                       const Eigen::MatrixBase<MatrixType3>& dRNEA_partial_da);

  template <typename ConfigVectorType, typename TangentVectorType1, 
            typename TangentVectorType2, typename MatrixType1, 
            typename MatrixType2, typename MatrixType3>
  void ABADerivatives(const Eigen::MatrixBase<ConfigVectorType>& q, 
                      const Eigen::MatrixBase<TangentVectorType1>& v, 
                      const Eigen::MatrixBase<TangentVectorType2>& tau,
                      const Eigen::MatrixBase<MatrixType1>& dABA_partial_dq, 
                      const Eigen::MatrixBase<MatrixType2>& dABA_partial_dv, 
                      const Eigen::MatrixBase<MatrixType3>& dABA_partial_dtau);

  ///
  /// @brief Computes the inverse of the joint inertia matrix M.
  /// @param[in] M Joint inertia matrix. Size must be 
  /// Robot::dimv() x Robot::dimv().
  /// @param[out] Minv Inverse of the joint inertia matrix M. Size must be 
  /// Robot::dimv() x Robot::dimv().
  ///   
  template <typename MatrixType1, typename MatrixType2>
  void computeMinv(const Eigen::MatrixBase<MatrixType1>& M, 
                   const Eigen::MatrixBase<MatrixType2>& Minv);

  ///
  /// @brief Computes the state equation, i.e., the velocity and forward 
  /// dynamics.
  /// @param[in] q Configuration. Size must be Robot::dimq().
  /// @param[in] v Generalized velocity. Size must be Robot::dimv().
  /// @param[in] tau Generalized acceleration. Size must be Robot::dimv().
  /// @param[out] dq The resultant generalized velocity. Size must be 
  /// Robot::dimv().
  /// @param[out] dv The resultant generalized acceleration. Size must be 
  /// Robot::dimv().
  ///
  template <typename ConfigVectorType, typename TangentVectorType1, 
            typename TangentVectorType2, typename TangentVectorType3,
            typename TangentVectorType4>
  void stateEquation(const Eigen::MatrixBase<ConfigVectorType>& q, 
                     const Eigen::MatrixBase<TangentVectorType1>& v, 
                     const Eigen::MatrixBase<TangentVectorType2>& tau, 
                     const Eigen::MatrixBase<TangentVectorType3>& dq,
                     const Eigen::MatrixBase<TangentVectorType4>& dv);

  ///
  /// @brief Generates feasible configuration randomly.
  /// @param[out] q The random configuration. Size must be Robot::dimq().
  ///
  template <typename ConfigVectorType>
  void generateFeasibleConfiguration(
      const Eigen::MatrixBase<ConfigVectorType>& q) const;

  ///
  /// @brief Normalizes a configuration vector.
  /// @param[in, out] q The normalized configuration. Size must be Robot::dimq().
  ///
  template <typename ConfigVectorType>
  void normalizeConfiguration(
      const Eigen::MatrixBase<ConfigVectorType>& q) const;

  ///
  /// @brief Returns the dimensiton of the configuration.
  /// @return The dimensiton of the configuration.
  /// 
  int dimq() const;

  ///
  /// @brief Returns the dimensiton of the velocity, i.e, tangent space.
  /// @return The dimensiton of the velocity, i.e, the dimension of the tangent 
  /// space of the configuration space.
  /// 
  int dimv() const;
  
  ///
  /// @brief Returns the dimensiton of the velocity, i.e, tangent space.
  /// @return The dimensiton of the velocity, i.e, the dimension of the tangent 
  /// space of the configuration space.
  /// 
  int dimu() const;

  ///
  /// @brief Returns the dimensiton of the generalized torques corresponding to 
  /// the passive joints.
  /// @return The dimensiton of the generalized torques corresponding to the 
  //// passive joints.
  /// 
  int dim_passive() const;

  ///
  /// @brief Returns true if the robot has a floating base and false if not.
  /// @returns true if the robot has a floating base and false if not.
  /// 
  bool has_floating_base() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  FloatingBase floating_base_;
  pinocchio::container::aligned_vector<pinocchio::Force> fjoint_;
  int dimq_, dimv_, dimu_;
};

} // namespace ipoptpino

#include "ipopt-pino/robot/robot.hxx"

#endif // IPOPT_PINO_ROBOT_HPP_ 