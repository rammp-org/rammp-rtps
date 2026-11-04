/**
 * @file odometry_message.hpp
 * @brief The base's wheel odometry, which the MIB publishes to ROS 2: nav_msgs/Odometry on
 * /base/odom, and the odom -> base_link transform on /tf.
 *
 * - C++20. Each struct IS the wire layout: XCDR1, fields in declaration order.
 * - These are standard ROS 2 types (nav_msgs, tf2_msgs), on purpose: the Jetson's
 *   navigation stack reads them as is. So the names are the ROS 2 wire names, and there
 *   is no mirror .msg on rammp-interfaces-ros2: the ROS definitions are the source of
 *   truth, and the structs here mirror them field for field.
 * - Frames follow REP-105: the pose is in `odom`, the twist in `base_link`. Units are
 *   REP-103 (m, rad, m/s, rad/s; x forward, y left, yaw counter-clockwise from above).
 * - The MIB is the only odometry source today, so it publishes /tf itself. Once a filter
 *   on the Jetson (robot_localization or similar) owns odom -> base_link, it subscribes to
 *   /base/odom and the MIB's /tf publisher goes away: two publishers of the same
 *   transform fight.
 *
 * Example (espp, `rtps` a started espp::RtpsParticipant): the MIB publishing odometry
 *
 *   espp::Publisher<rammp::Odometry> odom_pub(
 *       rtps, {.topic = rammp::kBaseOdom.name, .type_name = rammp::kBaseOdom.type,
 *              .reliability = espp::RtpsParticipant::Reliability::RELIABLE});
 */

#ifndef RAMMP_RTPS_ODOMETRY_MESSAGE_HPP
#define RAMMP_RTPS_ODOMETRY_MESSAGE_HPP

#include <string>
#include <vector>

#include "messages/geometry_message.hpp"
#include "messages/header.hpp"
#include "messages/topic.hpp"

/* -------------------------------------------------------------------------
 * Topics and types: ROS 2 wire names (see the file comment). The base serves under the
 * "base" namespace, like its arbitration (control_message.hpp); /tf is global by design,
 * tf2 listeners read it at the root.
 * ---------------------------------------------------------------------- */

/* MIB -> anyone. Reliable: tf2 and the navigation stack subscribe reliable, and a
   best-effort writer never matches a reliable reader. */
#define RAMMP_ROS_TOPIC_BASE_ODOM "rt/base/odom"
#define RAMMP_ROS_TYPE_ODOMETRY "nav_msgs::msg::dds_::Odometry_"

/* MIB -> tf2. Reliable, same reason. */
#define RAMMP_ROS_TOPIC_TF "rt/tf"
#define RAMMP_ROS_TYPE_TF_MESSAGE "tf2_msgs::msg::dds_::TFMessage_"

namespace rammp {

struct Odometry;
struct TFMessage;

inline constexpr Topic<Odometry> kBaseOdom{RAMMP_ROS_TOPIC_BASE_ODOM, RAMMP_ROS_TYPE_ODOMETRY};
inline constexpr Topic<TFMessage> kTf{RAMMP_ROS_TOPIC_TF, RAMMP_ROS_TYPE_TF_MESSAGE};

/** The frames the MIB uses (REP-105). */
inline constexpr char kOdomFrame[] = "odom";
inline constexpr char kBaseFrame[] = "base_link";

/* -------------------------------------------------------------------------
 * Messages
 * ---------------------------------------------------------------------- */

/** nav_msgs/Odometry: where the base thinks it is, from its wheels. */
struct Odometry {
  Header header;              /**< stamp: when the wheel samples were taken; frame_id: the
                                   frame the pose is in, kOdomFrame */
  std::string child_frame_id; /**< the frame the twist is in, kBaseFrame */
  PoseWithCovariance pose;    /**< in header.frame_id; planar: z, roll, pitch unobserved */
  TwistWithCovariance twist;  /**< in child_frame_id; linear.x and angular.z only */
};

/** tf2_msgs/TFMessage: a batch of transforms for tf2. The MIB sends one, odom -> base_link. */
struct TFMessage {
  std::vector<TransformStamped> transforms;
};

} // namespace rammp

#endif /* RAMMP_RTPS_ODOMETRY_MESSAGE_HPP */
