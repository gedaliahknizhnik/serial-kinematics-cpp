#include "serial-kinematics/kinematic-chain.hpp"

#include <stdexcept>

namespace kinematics {

// CONSTRUCTORS ****************************************************************

KinematicChain::KinematicChain(const DenhavitHartenbergParam& params)
    : _params{params}, _dof{params.dof()} {
  _joint_vars.resize(_dof);
  _joint_vars *= 0;  // Make sure all elements are set to zero
}

// JOINT VARIABLES *************************************************************

const Eigen::VectorXd& KinematicChain::get_joint_vars() const {
  return _joint_vars;
}

void KinematicChain::set_joint_vars(Eigen::VectorXd joint_vars) {
  if (joint_vars.size() != _dof) {
    throw std::invalid_argument("Joint variables must be _dof x 1.");
  }
  _joint_vars = joint_vars;
  _transform_map
      .clear();  // Changing the joint variables changes all transforms. So they
                 // have to be recalculated and can't be looked up.
}

// TRANFORMATIONS **************************************************************

HomMat KinematicChain::get_transform_upto(const int ind_from,
                                          const int ind_to) {
  if (ind_from < 0 || ind_to > _dof || ind_to < 0 || ind_from > _dof) {
    throw std::range_error(
        "Transforms outside the range [0, _dof] are inaccessible.");
  }

  // If the transform is known - return it
  TransformMap::iterator it;
  if (check_transform_known(ind_from, ind_to, it)) {
    return it->second;
  }

  // Otherwise, calculate the forward direction transform
  HomMat out{HomMat::Identity()};
  int ind_min{std::min(ind_to, ind_from)}, ind_max{std::max(ind_to, ind_from)};
  for (int ii{ind_min}; ii < ind_max; ++ii) {
    out = out * get_transform(_params[ii], _joint_vars[ii]);
  }

  // If what we want is an inverse transform, invert the result
  if (ind_to > ind_from) {
    _transform_map.insert(
        {{ind_to, ind_from}, out});  // Store forward direction
    out = out.inverse().eval();
  }

  _transform_map.insert({{ind_from, ind_to}, out});
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

HomVec KinematicChain::transform_point(const HomVec& pt_in, const int frame_in,
                                       const int frame_out) {
  HomMat H{get_transform_upto(frame_in, frame_out)};
  return H * pt_in;
}
PosVec KinematicChain::transform_point(const PosVec& pt_in, const int frame_in,
                                       const int frame_out) {
  HomVec v;
  v.block<3, 1>(0, 0) = pt_in;
  v(3) = 1;
  HomVec v_out{transform_point(v, frame_in, frame_out)};
  return v_out.block<3, 1>(0, 0);
}

PosVec KinematicChain::transform_velocity(const PosVec& vel_in,
                                          const int frame_in,
                                          const int frame_out) {
  RotMat R{get_rotation_upto(frame_in, frame_out)};
  return R * vel_in;
}

Vector6d KinematicChain::transform_velocity(const Vector6d& vel_in,
                                            const int frame_in,
                                            const int frame_out) {
  RotMat R{get_rotation_upto(frame_in, frame_out)};

  Vector6d vel_out;
  vel_out.block<3, 1>(0, 0) = R * vel_in.block<3, 1>(0, 0);
  vel_out.block<3, 1>(3, 0) = R * vel_in.block<3, 1>(3, 0);

  return vel_out;
}

bool KinematicChain::check_transform_known(const int frame_from,
                                           const int frame_to,
                                           TransformMap::iterator& it) {
  it = _transform_map.find({frame_from, frame_to});
  return (it != _transform_map.end());
}

}  // namespace kinematics
