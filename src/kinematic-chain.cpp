#include "serial-kinematics/kinematic-chain.hpp"

#include <cassert>
#include <stdexcept>

namespace kinematics {

// TODO: Formalize these assumptions in documentation:
// 1. Frame 0 is a fixed origin Frame
// 2. Row 0 of DH parameters corresponds to link 1, etc.
KinematicChain::KinematicChain(const DenhavitHartenbergParam &params)
    : _params{params}, _dof{params.dof()} {
  _joint_vars.resize(_dof);
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
  if (ind_from < 0 || ind_to >= _dof || ind_to < 0 ||
      ind_from >= _dof) {  // TODO: Check if this should be > or >=
    throw std::range_error(
        "Transforms outside the range [0, _dof] are inaccessible.");
  }

  HomMat out{HomMat::Identity()};
  int step = sgn(ind_from - ind_to);
  for (int ii{ind_to}; ii != ind_from;
       ii += step) {  // TODO: Check what happens when ii is 0
    out = out * get_transform(_params[ii], _joint_vars[ii]);
  }

  return out;
}

}  // namespace kinematics
