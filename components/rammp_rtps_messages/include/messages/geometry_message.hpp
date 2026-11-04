/**
 * @file geometry_message.hpp
 * @brief The ROS 2 geometry_msgs types, as wire structs. Shared by every message that
 * carries a pose, a twist or a transform, so there is one definition of each.
 *
 * - C++20. Each struct IS the wire layout: XCDR1, fields in declaration order. Each one
 *   mirrors its geometry_msgs .msg field for field, so a ROS 2 node decodes it with no
 *   bridge. Do not reorder, add, or retype fields.
 * - Units are REP-103: metres, radians, x forward, y left, z up, yaw counter-clockwise
 *   from above.
 */

#ifndef RAMMP_RTPS_GEOMETRY_MESSAGE_HPP
#define RAMMP_RTPS_GEOMETRY_MESSAGE_HPP

#include <array>
#include <string>

#include "messages/header.hpp"

namespace rammp {

/** geometry_msgs/Vector3 */
struct Vector3 {
  double x;
  double y;
  double z;
};

/** geometry_msgs/Point */
struct Point {
  double x; /**< m */
  double y; /**< m */
  double z; /**< m */
};

/** geometry_msgs/Quaternion. Identity is {0, 0, 0, 1}; a value-initialised one is not. */
struct Quaternion {
  double x;
  double y;
  double z;
  double w;
};

/** geometry_msgs/Pose */
struct Pose {
  Point position;
  Quaternion orientation;
};

/** geometry_msgs/Twist */
struct Twist {
  Vector3 linear;  /**< m/s */
  Vector3 angular; /**< rad/s */
};

/** A row-major 6x6 covariance, as geometry_msgs declares it: float64[36]. */
using Covariance6 = std::array<double, 36>;

/** geometry_msgs/PoseWithCovariance */
struct PoseWithCovariance {
  Pose pose;
  Covariance6 covariance; /**< x, y, z, roll, pitch, yaw; all-zero = unknown */
};

/** geometry_msgs/TwistWithCovariance */
struct TwistWithCovariance {
  Twist twist;
  Covariance6 covariance; /**< vx, vy, vz, wx, wy, wz; all-zero = unknown */
};

/** geometry_msgs/Transform */
struct Transform {
  Vector3 translation; /**< m */
  Quaternion rotation;
};

/** geometry_msgs/TransformStamped: the transform from header.frame_id to child_frame_id. */
struct TransformStamped {
  Header header;              /**< stamp: when the transform holds; frame_id: the parent */
  std::string child_frame_id; /**< the frame the transform leads to */
  Transform transform;
};

} // namespace rammp

#endif /* RAMMP_RTPS_GEOMETRY_MESSAGE_HPP */
