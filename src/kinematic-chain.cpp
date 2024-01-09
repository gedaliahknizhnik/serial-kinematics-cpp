#include "serial-kinematics/kinematic-chain.hpp"

#include <cassert>
#include <iostream>
#include <stdexcept>

namespace kinematics {

// TODO: Formalize these assumptions in documentation:
// 1. Frame 0 is a fixed origin Frame
// 2. Row 0 of DH parameters corresponds to link 1, etc.
KinematicChain::KinematicChain(const DenhavitHartenbergParam& params)
    : _params{params}, _dof{params.dof()} {
  _joint_vars.resize(_dof);
  _joint_vars *= 0;
}

const Eigen::VectorXd& KinematicChain::get_joint_vars() const {
  return _joint_vars;
}

void KinematicChain::set_joint_vars(Eigen::VectorXd joint_vars) {
  if (joint_vars.size() != _dof) {
    throw std::invalid_argument("Joint variables must be _dof x 1.");
  }
  _joint_vars = joint_vars;
}

/**
 * @brief Get transform from frame index ind_from to frame index ind_to, where
 * index 0 corresponds to the fixed base frame.
 *
 * - By default gives transformations to the origin frame.
 * - Works for transformations where ind_from < ind_to.
 */
HomMat KinematicChain::get_transform_upto(const int ind_from,
                                          const int ind_to) {
  if (ind_from < 0 || ind_to > _dof || ind_to < 0 ||
      ind_from > _dof) {  // TODO: Check if this should be > or >=
    throw std::range_error(
        "Transforms outside the range [0, _dof] are inaccessible.");
  }

  HomMat out{HomMat::Identity()};

  for (int ii{std::min(ind_to, ind_from)}; ii < std::max(ind_to, ind_from);
       ++ii) {
    out = out * get_transform(_params[ii], _joint_vars[ii]);
  }

  if (ind_to > ind_from) {
    out = out.inverse().eval();
  }
  return out;
}

RotMat KinematicChain::get_rotation_upto(const int ind_from, const int ind_to) {
  HomMat H{get_transform_upto(ind_from, ind_to)};
  return H.block<3, 3>(0, 0);
}
HomVec KinematicChain::get_position_vec_hom(const int ind_from,
                                            const int ind_to) {
  HomMat H{get_transform_upto(ind_from, ind_to)};
  return H.block<4, 1>(0, 3);
}
PosVec KinematicChain::get_position_vec(const int ind_from, const int ind_to) {
  HomMat H{get_transform_upto(ind_from, ind_to)};
  return H.block<3, 1>(0, 3);
}

}  // namespace kinematics
