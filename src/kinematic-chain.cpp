#include <stdexcept>

#include "serial-kinematics/kinematics.hpp"

namespace kinematics {

using HomMat = Eigen::Matrix4d;
using RotMat = Eigen::Matrix3d;
using HomVec = Eigen::Vector4d;
using PosVec = Eigen::Vector3d;

class KinematicChain {
 public:
  KinematicChain(const DenhavitHartenbergParam &params)
      : _params{params}, _dof{params.dof()} {
    _joint_vars.resize(_dof);
  }

  HomMat get_transform_upto(int ind) {
    if (ind < 0 || ind >= _dof) {  // TODO: Check if this should be > or >=
      throw std::range_error(
          "Transforms outside the range [0, _dof] are inaccessible.");
    }

    HomMat out{HomMat::Identity()};
    for (int ii{0}; ii < ind; ++ii) {  // TODO: Check what happens when ii is 0
      out = out * get_transform(_params[ii], _joint_vars[ii]);
    }

    return out;
  }

 private:
  const DenhavitHartenbergParam _params{};
  const int _dof{};

  Eigen::VectorXd _joint_vars{};
};

}  // namespace kinematics
