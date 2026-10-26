/**
 * @file control_message.hpp
 * @brief Token arbitration for the base: who may command it, and the tokened commands.
 * The service half (acquire, release, revoke) is services/control_service.hpp.
 *
 * - C++20. Each struct IS the wire layout: XCDR1, fields in declaration order.
 * - Unlike the other headers here, these are ROS 2 types, on purpose. The arm runs the
 *   same arbitration protocol from ROS 2 (kinova-gen3-ros2), and one orchestrator on the
 *   Jetson holds a token for each. So the names are the ROS 2 wire names of the
 *   rammp-interfaces-ros2 types, and a ROS 2 node talks to the MIB with no bridge:
 *     rt/<ns>/<topic>                         topic
 *     <pkg>::msg::dds_::<Type>_               type
 * - The base serves under the "base" namespace; the arm, at the root, has the
 *   un-namespaced names. "One arbitration server per arbitrated resource, each on its
 *   own namespace" (rammp_common_interfaces/msg/ControlStatus.msg).
 *
 * Mirrors, field for field:
 *   Token          uint8[16] token          (every tokened command, and the services)
 *   ControlStatus  rammp_common_interfaces/msg/ControlStatus
 *   TwistSetpoint  rammp_base_interfaces/msg/TwistSetpoint   (rammp-interfaces-ros2 dev)
 *
 * Example (espp, `rtps` a started espp::RtpsParticipant): the MIB publishing who owns it
 *
 *   espp::Publisher<rammp::ControlStatus> status_pub(
 *       rtps, {.topic = rammp::kBaseControlStatus.name,
 *              .type_name = rammp::kBaseControlStatus.type,
 *              .reliability = espp::RtpsParticipant::Reliability::RELIABLE});
 */

#ifndef RAMMP_RTPS_CONTROL_MESSAGE_HPP
#define RAMMP_RTPS_CONTROL_MESSAGE_HPP

#include <array>
#include <cstdint>
#include <string>

#include "messages/header.hpp"
#include "messages/topic.hpp"

/* -------------------------------------------------------------------------
 * Topics and types: ROS 2 wire names (see the file comment).
 * ---------------------------------------------------------------------- */

/* MIB -> anyone. Reliable; ROS 2 subscribers should ask for transient_local, depth 1,
   as the arm's ControlStatus is latched. */
#define RAMMP_ROS_TOPIC_BASE_CONTROL_STATUS "rt/base/control_status"
#define RAMMP_ROS_TYPE_CONTROL_STATUS "rammp_common_interfaces::msg::dds_::ControlStatus_"

/* The owner -> MIB. Best-effort, latest wins. */
#define RAMMP_ROS_TOPIC_BASE_TWIST_SETPOINT "rt/base/setpoint/twist"
#define RAMMP_ROS_TYPE_TWIST_SETPOINT "rammp_base_interfaces::msg::dds_::TwistSetpoint_"

namespace rammp {

struct ControlStatus;
struct TwistSetpoint;

inline constexpr Topic<ControlStatus> kBaseControlStatus{RAMMP_ROS_TOPIC_BASE_CONTROL_STATUS,
                                                         RAMMP_ROS_TYPE_CONTROL_STATUS};
inline constexpr Topic<TwistSetpoint> kBaseTwistSetpoint{RAMMP_ROS_TOPIC_BASE_TWIST_SETPOINT,
                                                         RAMMP_ROS_TYPE_TWIST_SETPOINT};

/* -------------------------------------------------------------------------
 * Messages
 * ---------------------------------------------------------------------- */

/** The capability: 128 random bits minted on every grant, or adopted from the request.
    A fixed array on the wire, no length prefix. All-zero is "no token": no grant ever
    holds it, so an untokened command never matches. */
using Token = std::array<uint8_t, 16>;

/** Who may command the base right now. Sent on change, and resent periodically.
    Deliberately carries no token: publishing a capability would defeat it. */
struct ControlStatus {
  Header header;            /**< stamp: when the state was read; frame_id "" */
  bool arbitration_enabled; /**< false = tokens are ignored (a build-time setting) */
  bool estopped;            /**< latched; nothing is admitted */
  bool owned;               /**< does anyone hold the base? */
  std::string owner_id;     /**< "" if none */
  uint64_t generation;      /**< bumps on every grant; a client that sees it move was
                                 dispossessed */
  uint64_t rejected_count;  /**< commands refused since boot */
};

/** geometry_msgs/Vector3 */
struct Vector3 {
  double x;
  double y;
  double z;
};

/** geometry_msgs/Twist */
struct Twist {
  Vector3 linear;  /**< m/s */
  Vector3 angular; /**< rad/s */
};

/** A velocity command for the base, in base_link. Only linear.x (forward) and
    angular.z (counter-clockwise from above) apply to a differential drive; the rest are
    ignored. Admitted only if `token` is the live grant's. */
struct TwistSetpoint {
  Twist twist;
  Token token;
};

} // namespace rammp

#endif /* RAMMP_RTPS_CONTROL_MESSAGE_HPP */
