#ifndef KINEMATIC_CHAIN_HPP
#define KINEMATIC_CHAIN_HPP

#include "serial-kinematics/kinematics.hpp"

namespace kinematics {

using HomMat = Eigen::Matrix4d;
using RotMat = Eigen::Matrix3d;
using HomVec = Eigen::Vector4d;
using PosVec = Eigen::Vector3d;

// TODO: Formalize these assumptions in documentation:
// 1. Frame 0 is a fixed origin Frame
// 2. Row 0 of DH parameters corresponds to link 1, etc.
class KinematicChain {
 public:
  KinematicChain(const DenhavitHartenbergParam& params);

  const Eigen::VectorXd& get_joint_vars() const;
  void set_joint_vars(Eigen::VectorXd joint_vars);

  /**
   * @brief Get transform from frame index ind_from to frame index ind_to,
   * where index 0 corresponds to the fixed base frame.
   *
   * - By default gives transformations to the origin frame.
   * - Works for transformations where ind_from < ind_to.
   */
  HomMat get_transform_upto(const int ind_from, const int ind_to = 0);

  RotMat get_rotation_upto(const int ind_from, const int ind_to = 0);
  HomVec get_position_vec_hom(const int ind_from, const int ind_to = 0);
  PosVec get_position_vec(const int ind_from, const int ind_to = 0);

 private:
  const DenhavitHartenbergParam _params{};
  const int _dof{};

  Eigen::VectorXd _joint_vars{};
};

}  // namespace kinematics

#endif
