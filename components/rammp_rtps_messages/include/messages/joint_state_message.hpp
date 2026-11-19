/**
 * @file joint_state_message.hpp
 * @brief The base's joint positions, which the MIB publishes to ROS 2: sensor_msgs/JointState
 * on /base/joint_states.
 *
 * - C++20. Each struct IS the wire layout: XCDR1, fields in declaration order.
 * - A standard ROS 2 type, on purpose: robot_state_publisher reads it as is and moves the
 *   chair's model (TF, RViz, the simulation). So the names are the ROS 2 wire names, and
 *   there is no mirror .msg on rammp-interfaces-ros2: the ROS definition is the source of
 *   truth, and the struct here mirrors it field for field.
 * - Joint names and units are the base URDF's (Demo-Software,
 *   core/rammp_prototype_description/urdf/rammp_prototype_base.urdf): the names below are its
 *   non-fixed joints, positions are measured from the URDF's zero along the joint's axis, in
 *   rad for a revolute joint and m for a prismatic one (REP-103).
 * - The message may carry only some of the joints: name, position (and velocity and effort,
 *   when not empty) are parallel arrays, and a reader looks a joint up by name.
 *
 * Example (espp, `rtps` a started espp::RtpsParticipant): the MIB publishing joint states
 *
 *   espp::Publisher<rammp::JointState> joints_pub(
 *       rtps, {.topic = rammp::kBaseJointStates.name, .type_name = rammp::kBaseJointStates.type,
 *              .reliability = espp::RtpsParticipant::Reliability::RELIABLE});
 */

#ifndef RAMMP_RTPS_JOINT_STATE_MESSAGE_HPP
#define RAMMP_RTPS_JOINT_STATE_MESSAGE_HPP

#include <string>
#include <vector>

#include "messages/header.hpp"
#include "messages/topic.hpp"

/* -------------------------------------------------------------------------
 * Topics and types: ROS 2 wire names (see the file comment). Under the "base" namespace,
 * like the base's odometry (odometry_message.hpp), so it does not collide with the arm's
 * joint states.
 * ---------------------------------------------------------------------- */

/* MIB -> anyone. Reliable: robot_state_publisher subscribes reliable, and a best-effort
   writer never matches a reliable reader. */
#define RAMMP_ROS_TOPIC_BASE_JOINT_STATES "rt/base/joint_states"
#define RAMMP_ROS_TYPE_JOINT_STATE "sensor_msgs::msg::dds_::JointState_"

namespace rammp {

struct JointState;

inline constexpr Topic<JointState> kBaseJointStates{RAMMP_ROS_TOPIC_BASE_JOINT_STATES,
                                                    RAMMP_ROS_TYPE_JOINT_STATE};

/* -------------------------------------------------------------------------
 * The base URDF's joint names (see the file comment). _l / _r: the chair's left / right.
 * ---------------------------------------------------------------------- */

/** Carriages: the main-leg plates' fore/aft travel. Prismatic, m. */
inline constexpr char kJointMainPlateLeft[] = "dw_main_plate_l";
inline constexpr char kJointMainPlateRight[] = "dw_main_plate_r";

/** Main legs' swing arms (the drive wheels' legs). Revolute, rad. */
inline constexpr char kJointMainSwingArmLeft[] = "motor_swing_arm_l";
inline constexpr char kJointMainSwingArmRight[] = "motor_swing_arm_r";

/** Main legs' linear actuators: the elevator rods' stroke. Prismatic, m. */
inline constexpr char kJointMainElevatorRodLeft[] = "motor_elevator_rod_l";
inline constexpr char kJointMainElevatorRodRight[] = "motor_elevator_rod_r";

/** Front caster: its swing arm (revolute, rad) and its actuator's rod (prismatic, m). */
inline constexpr char kJointFrontCasterSwingArm[] = "front_caster_swing_arm";
inline constexpr char kJointFrontCasterMotorRod[] = "front_caster_motor_rod";

/** Rear caster: its swing arm (revolute, rad), its actuator's rod and the shock absorber's
    rod between them (prismatic, m). */
inline constexpr char kJointRearCasterSwingArm[] = "rear_caster_swing_arm";
inline constexpr char kJointRearCasterMotorRod[] = "rear_caster_motor_rod";
inline constexpr char kJointRearCasterDampenerRod[] = "rear_caster_dampener_rod";

/* -------------------------------------------------------------------------
 * Messages
 * ---------------------------------------------------------------------- */

/** sensor_msgs/JointState: where the joints are. Parallel arrays, one entry per joint. */
struct JointState {
  Header header;                 /**< stamp: when the joints were read; frame_id: "" */
  std::vector<std::string> name; /**< the joints, from the k*Joint* names above */
  std::vector<double> position;  /**< rad (revolute) or m (prismatic), per name */
  std::vector<double> velocity;  /**< rad/s or m/s, per name; empty if not measured */
  std::vector<double> effort;    /**< N*m or N, per name; empty if not measured */
};

} // namespace rammp

#endif /* RAMMP_RTPS_JOINT_STATE_MESSAGE_HPP */
