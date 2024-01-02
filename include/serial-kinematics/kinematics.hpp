#ifndef SERIAL_KINEMATICS_HPP
#define SERIAL_KINEMATICS_HPP

#include <Eigen/Geometry>
#include <string>
#include <vector>

/**
 * @brief Kinematics namespace
 */
namespace kinematics {

// JOINT TYPES *****************************************************************

/**
 * @brief Enumeration of possible joint types
 */
enum class Type { revolute, prismatic, not_a_joint };
/**
 * @brief Convert joint type into a string representation
 */
std::string type_to_string(const Type& type);

// SINGLE JOINT ****************************************************************

/**
 * @brief Single joint in a kinematic chain, defined by DH parameters.
 *
 * Angles are specified in __radians__. Distances can be any unit, but the user
 * is expected to provide consistent units throughout use.
 */
struct Joint {
  Type type{Type::not_a_joint};  ///< Joint type
  double theta{};
  double a{};
  double d{};
  double alpha{};
};
/**
 * @brief Overloaded print operator for a single joint.
 */
std::ostream& operator<<(std::ostream& out, const Joint& joint);

// KINEMATIC CHAIN *************************************************************

/**
 * @brief Parameterization of a kinematic chain as a list of joints.
 */
struct DenhavitHartenbergParam {
  std::vector<Joint> joints{};

  /**
   * @brief Overload access operator to the internal joints.
   */
  Joint& operator[](const int index) { return joints[index]; }
  const Joint& operator[](const int index) const { return joints[index]; }
  /**
   * @brief Get size of the DH set.
   */
  int size() const { return joints.size(); }
  /**
   * @brief Get number of DOF in the system.
   */
  int dof() const { return size(); }
};
/**
 * @brief Overload print operator for a kinematic chain representation
 */
std::ostream& operator<<(std::ostream& out,
                         const DenhavitHartenbergParam& params);

// TRANSFORMATIONS *************************************************************

/**
 * @brief Get the homogeneous transform representation of a DH parameterized
 * joint.
 *
 * @param joint - a Joint structure representation of the joint
 * @param joint_var - the [double ]current value of the joint variable. If the
 * joint is revolute, this is understood as an angle in radians. If the joint is
 * prismatic, this is understood as a distance in the appropriate unit.
 *
 * @return Eigen::Matrix4d homogeneous transform matrix
 */
Eigen::Matrix4d get_transform(const Joint joint, const double joint_var);

}  // namespace kinematics

#endif
