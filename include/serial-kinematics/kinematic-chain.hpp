#ifndef KINEMATIC_CHAIN_HPP
#define KINEMATIC_CHAIN_HPP

#include <boost/functional/hash.hpp>
#include <unordered_map>

#include "serial-kinematics/kinematics.hpp"

namespace kinematics {

using HomMat = Eigen::Matrix4d;
using RotMat = Eigen::Matrix3d;
using HomVec = Eigen::Vector4d;
using PosVec = Eigen::Vector3d;
using TransformMap = std::unordered_map<std::pair<int, int>, HomMat,
                                        boost::hash<std::pair<int, int>>>;
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

  HomVec transform_point(const HomVec& pt_in, const int frame_in,
                         const int frame_out = 0);
  PosVec transform_point(const PosVec& pt_in, const int frame_in,
                         const int frame_out = 0);

  bool check_transform_known(const int frame_in, const int frame_out,
                             TransformMap::iterator& it);

 private:
  const DenhavitHartenbergParam _params{};
  const int _dof{};

  Eigen::VectorXd _joint_vars{};
  TransformMap _transform_map;
};

}  // namespace kinematics

#endif
