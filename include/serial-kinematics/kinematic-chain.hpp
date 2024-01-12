#ifndef KINEMATIC_CHAIN_HPP
#define KINEMATIC_CHAIN_HPP

#include <boost/functional/hash.hpp>
#include <unordered_map>

#include "serial-kinematics/kinematics.hpp"

namespace kinematics {

// TYPE ALIASES ****************************************************************

using HomMat = Eigen::Matrix4d;
using RotMat = Eigen::Matrix3d;
using HomVec = Eigen::Vector4d;
using PosVec = Eigen::Vector3d;
using Vector6d = Eigen::Vector<double, 6>;
using TransformMap = std::unordered_map<std::pair<int, int>, HomMat,
                                        boost::hash<std::pair<int, int>>>;

/**
 * @brief A class to represent serial kinematic chains, as parameterized by DH
 * parameters. Alternatively, a serial manipulator or robot.
 *
 * This class supports chains of serial and prismatic joints, in any
 * combination.
 *
 * In the definition of this class, we define frame 0 to be a fixed world frame.
 * Therefore, row 0 of the DH parameters corresponds to link 1, row 1 to link 2,
 * and so on.
 *
 * Therefore, if I have a 6 DOF manipulator, the transform from its end effector
 * frame is given by `get_transform_upto(6)` even though conventional array
 * indexing would place the end effector at index 5 of the DH parameters.
 *
 * This class uses a form of lazy evaluation. Whenever a transform is required
 * by any of the methods it is calculated and saved into a hashmap. If that
 * transform is required again, it is simply pulled from the map. Resetting
 * joint variables resets the map, of course.
 */
class KinematicChain {
 public:
  // CONSTRUCTORS **************************************************************

  /**
   * @brief Create a kinematic chain from its DH parameters
   *
   * @param DenhavitHartenbergParam struct of DH parameters
   */
  KinematicChain(const DenhavitHartenbergParam& params);

  // JOINT VARIABLES ***********************************************************

  /**
   * @brief Set the internal joint variables
   *
   * @param Eigen::Vector6d joint variables vector. Interpreted as [rad] or
   * [dist], depending on the associated joint.
   * @throws std::invalid_argument if dimension of input vector is not the same
   * as the chain's DOF.
   */
  void set_joint_vars(Eigen::VectorXd joint_vars);

  /**
   * @brief Get the internval joint variables
   *
   * @return const reference to Eigen::VectorXd
   */
  const Eigen::VectorXd& get_joint_vars() const;

  // TRANSFORMATIONS ***********************************************************

  /**
   * @brief Check if a transform is already stored in the internal map.
   *
   * While this method is provided publicly, users do _not_ need to use it
   * before using a transform. Invoking _any_ method that requires knowledge of
   * a transform will automatically compute a transform if it is not known.
   *
   * @param frame_in - int frame from which we are converting
   * @param frame_out - int frame to which we are converting
   * @param it - TransformMap iterator where the transform lives, if found.
   *
   * @return true - the transform is in the map and is located at `it`
   * @return false - the trasnform is not found
   */
  bool check_transform_known(const int frame_in, const int frame_out,
                             TransformMap::iterator& it);
  /**
   * @brief Get transform from frame index ind_from to frame index ind_to,
   * where index 0 corresponds to the fixed base frame.
   *
   * That is, `get_transform_upto(6)` gives a transform that converts points in
   * frame 6 to the base frame.
   *
   * - By default gives transformations to the origin frame.
   * - Works for transformations where ind_from < ind_to.
   *
   * @param ind_from - int frame from which to transform
   * @param ind_to - int frame to which to transform
   * @return Homogeneous Transform (4x4) matrix
   */
  HomMat get_transform_upto(const int ind_from, const int ind_to = 0);

  /**
   * @brief Get rotation from frame index ind_from to frame index ind_to, where
   * index 0 corresponds to the fixed base frame.
   *
   * @param ind_from - int frame from which to transform
   * @param ind_to - int frame to which to transform
   * @return Rotation Matrix (3x3)
   */
  RotMat get_rotation_upto(const int ind_from, const int ind_to = 0);
  /**
   * @brief Get homogeneous translation from frame index ind_from to frame index
   * ind_to, where index 0 corresponds to the fixed base frame.
   *
   * @param ind_from - int frame from which to transform
   * @param ind_to - int frame to which to transform
   * @return Homogeneous (4x1) translation vector
   */
  HomVec get_position_vec_hom(const int ind_from, const int ind_to = 0);
  /**
   * @brief Get translation from frame index ind_from to frame index ind_to,
   * where index 0 corresponds to the fixed base frame.
   *
   * @param ind_from - int frame from which to transform
   * @param ind_to - int frame to which to transform
   * @return (3x1) translation vector
   */
  PosVec get_position_vec(const int ind_from, const int ind_to = 0);

  /**
   * @brief Transform a point from one frame to another
   *
   * @param pt_in - Homogeneous Vector representation of the input point
   * @param frame_in - int index of frame in which pt_in is expressed
   * @param frame_out - ind index of frame into which pt_in will be converted.
   *
   * @return Homogeneous vector representation of the converted point
   */
  HomVec transform_point(const HomVec& pt_in, const int frame_in,
                         const int frame_out = 0);
  /**
   * @brief Transform a point from one frame to another. @override
   *
   * @param pt_in - Vector representation of the input point
   * @param frame_in - int index of frame in which pt_in is expressed
   * @param frame_out - ind index of frame into which pt_in will be converted.
   *
   * @return Vector representation of the converted point
   */
  PosVec transform_point(const PosVec& pt_in, const int frame_in,
                         const int frame_out = 0);
  /**
   * @brief Transform a 3x1 velocity from one frame to another. Unlike position
   * transformations, this uses only the rotation matrix portion of the
   * trasnform.
   *
   * @param vel_in - Vector representation of the input velocity
   * @param frame_in - int index of frame in which vel_in is expressed
   * @param frame_out - ind index of frame into which vel_in will be converted.
   *
   * @return Vector representation of the converted velocity
   */
  PosVec transform_velocity(const PosVec& vel_in, const int frame_in,
                            const int frame_out);
  /**
   * @brief Transform a 6x1 velocity from one frame to another. Unlike position
   * transformations, this uses only the rotation matrix portion of the
   * trasnform. @override
   *
   * @param vel_in - 6x1 vector representation of the input velocity, assumed to
   * be stacked 3x1 linear velocity, then stacked 3x1 angular velocity.
   * @param frame_in - int index of frame in which vel_in is expressed
   * @param frame_out - ind index of frame into which vel_in will be converted.
   *
   * @return Vector representation of the converted velocity
   */
  Vector6d transform_velocity(const Vector6d& vel_in, const int frame_in,
                              const int frame_out);

 private:
  // VARIABLES *****************************************************************

  const DenhavitHartenbergParam
      _params{};                  ///< Internal DH parameter representation
  const int _dof{};               ///< DOF of the manipulator chain
  Eigen::VectorXd _joint_vars{};  ///< Current state of each joint variable
  TransformMap _transform_map;    ///< Map of currently known transforms
};

// TYPE ALIASES FOR KINEMATIC_CHAIN ********************************************

using SerialManipulator = KinematicChain;
using SerialRobot = KinematicChain;

}  // namespace kinematics

#endif
