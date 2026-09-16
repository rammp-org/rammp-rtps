/**
 * @file motor_message.hpp
 * @brief RAMMP MIB <-> PACE RACER motor controller RTPS messages: topics, types, and payloads.
 * The MIB commands each wheel's PACE RACER board; each board reports its state. Both sides
 * use this header. Only what both must agree on lives here; controller tuning, limit
 * ceilings and telemetry timing are the board's own.
 *
 * - C++20. Each struct IS the wire layout: XCDR1, fields in declaration order.
 * - Best-effort, no durability. Both messages are state, resent periodically; the next
 *   packet replaces a lost one. The one event, MotorCommand::clear_fault_req_id, is a
 *   counter so a resend cannot repeat it.
 * - One topic per axis in each direction. A board subscribes to its own command topic
 *   only, so another axis's commands do not reach it.
 * - Units: SI at the motor output shaft. rad (multiturn, counted from board boot),
 *   rad/s, N.m. Counter-clockwise looking at the shaft is positive on every axis
 *   (right-hand rule); the MIB negates mirrored wheels.
 *
 * Naming:
 *   RAMMP_TYPE_MOTOR_COMMAND                  "rammp/msg/MotorCommand"
 *   RAMMP_TYPE_MOTOR_STATE                    "rammp/msg/MotorState"
 *   RAMMP_TOPIC_MIB_MOTOR_COMMAND(segment)    "rammp/mib/motor_command/<axis>"   MIB -> board
 *   RAMMP_TOPIC_MOTOR_STATE(segment)          "rammp/<axis>/motor_state"         board -> MIB
 *   kAxes[i].command / kAxes[i].state         the typed Topic<> pair for axis i
 *
 * Contents: Version, Types, Axis table, MotorCommand (MIB -> board), MotorState (board -> MIB).
 *
 * Example (espp, `rtps` a started espp::RtpsParticipant): the MIB driving the left wheel
 *
 *   const auto &axis = rammp::axis(rammp::AxisId::DRIVE_LEFT);
 *   espp::Publisher<rammp::MotorCommand> cmd_pub(
 *       rtps, {.topic = axis.command.name, .type_name = axis.command.type});
 *   espp::Subscriber<rammp::MotorState> state_sub(
 *       rtps, {.topic = axis.state.name, .type_name = axis.state.type,
 *              .on_message = [](const rammp::MotorState &s) { odometry(s.position); }});
 *   // Keep publishing at >= 10 Hz while idle too; 200 ms of silence is a WATCHDOG fault.
 *   cmd_pub.publish({.seq = seq++,
 *                    .requested_state = rammp::RequestedState::ARMED,
 *                    .mode = rammp::ControlMode::VELOCITY,
 *                    .velocity = 1.2f});
 */

#ifndef RAMMP_RTPS_MOTOR_MESSAGE_HPP
#define RAMMP_RTPS_MOTOR_MESSAGE_HPP

#include <array>
#include <cstddef>
#include <cstdint>

#include "topic.hpp"

/* -------------------------------------------------------------------------
 * Version: bump on any wire-layout or semantic change. A board reports the version it
 * was built with in MotorState::api_version; a mismatch is the MIB's to refuse.
 * ---------------------------------------------------------------------- */

#define RAMMP_MOTOR_API_VERSION 1

/* -------------------------------------------------------------------------
 * Types: one type each way, shared by every axis.
 * ---------------------------------------------------------------------- */

#define RAMMP_TYPE_MOTOR_COMMAND "rammp/msg/MotorCommand"
#define RAMMP_TYPE_MOTOR_STATE "rammp/msg/MotorState"

/** MIB -> board. `segment` is the axis's table segment, a string literal. */
#define RAMMP_TOPIC_MIB_MOTOR_COMMAND(segment) "rammp/mib/motor_command/" segment
/** board -> MIB. */
#define RAMMP_TOPIC_MOTOR_STATE(segment) "rammp/" segment "/motor_state"

namespace rammp {

/* -------------------------------------------------------------------------
 * Axis table: one row per PACE RACER board on the chair. To add one, add ONE row: the
 * next id, and a trailing `\` on every row but the last. Each row yields an AxisId, a
 * command topic and a state topic. A board stores its AxisId in NVS; the factory
 * default is AxisId::UNASSIGNED, which is on no topic, so an unprovisioned board is inert.
 * ---------------------------------------------------------------------- */

/* X(id, NAME, segment, label) */
#define RAMMP_AXIS_TABLE(X)                                                                        \
  X(0, FRONT_CASTER_LEFT, "front_caster_left", "Front caster L")                                   \
  X(1, FRONT_CASTER_RIGHT, "front_caster_right", "Front caster R")                                 \
  X(2, DRIVE_LEFT, "drive_left", "Drive L")                                                        \
  X(3, DRIVE_RIGHT, "drive_right", "Drive R")

enum class AxisId : uint8_t {
  UNASSIGNED = 255, /**< NVS default: not provisioned, no topics */
#define RAMMP_AXIS_ID(id_, name_, ...) name_ = id_,
  RAMMP_AXIS_TABLE(RAMMP_AXIS_ID)
#undef RAMMP_AXIS_ID
};

struct MotorCommand;
struct MotorState;

struct AxisSpec {
  AxisId id;
  const char *segment;         /**< "drive_left", the topic path segment */
  const char *label;           /**< "Drive L" */
  Topic<MotorCommand> command; /**< MIB -> this axis */
  Topic<MotorState> state;     /**< this axis -> MIB */
};

inline constexpr std::array kAxes{
#define RAMMP_AXIS_ROW(id_, name_, segment_, label_)                                               \
  AxisSpec{AxisId::name_,                                                                          \
           segment_,                                                                               \
           label_,                                                                                 \
           {RAMMP_TOPIC_MIB_MOTOR_COMMAND(segment_), RAMMP_TYPE_MOTOR_COMMAND},                    \
           {RAMMP_TOPIC_MOTOR_STATE(segment_), RAMMP_TYPE_MOTOR_STATE}},
    RAMMP_AXIS_TABLE(RAMMP_AXIS_ROW)
#undef RAMMP_AXIS_ROW
};
inline constexpr size_t kAxisCount = kAxes.size();

/** The row for `id`. Only for table ids: UNASSIGNED is the caller's to check first. */
inline constexpr const AxisSpec &axis(AxisId id) { return kAxes[static_cast<size_t>(id)]; }

/* -------------------------------------------------------------------------
 * Messages: MIB -> board
 * ---------------------------------------------------------------------- */

/** What the MIB wants the board to be. A level: the board converges to it and stays. */
enum class RequestedState : uint8_t {
  DISARMED = 0,  /**< gate driver off, motor coasts (Hi-Z) */
  ARMED = 1,     /**< run `mode` on the setpoints */
  SAFE_STOP = 2, /**< ramp velocity to zero under the torque limit, then hold position */
  ESTOP = 3,     /**< Hi-Z now, held while requested */
};

/** Which setpoint the board tracks while ARMED. */
enum class ControlMode : uint8_t {
  COAST = 0,    /**< Hi-Z */
  TORQUE = 1,   /**< track `torque` */
  VELOCITY = 2, /**< track `velocity`; the reference steps unless accel_limit is set */
  POSITION = 3, /**< track `position`, the latest one received */
  HOLD = 4,     /**< hold the position at the moment HOLD was entered */
};

/** One axis's command. Send at 10..50 Hz, idle or not: 200 ms without one is a WATCHDOG
    fault (safe stop, then hold). `mode` and setpoints apply only while ARMED.
    A mode change takes effect at once, controller integrators reset. 28 bytes on the wire. */
struct MotorCommand {
  uint8_t seq;                    /**< +1 per message, wraps; echoed as last_cmd_seq */
  RequestedState requested_state; /**< the level the board converges to */
  ControlMode mode;               /**< which setpoint to track */
  uint8_t clear_fault_req_id;     /**< +1 per clear request, wraps; a NEW value clears a
                                       latched fault into DISARMED. Echoed as clear_fault_ack. */
  float position;                 /**< rad, multiturn from boot   (POSITION) */
  float velocity;                 /**< rad/s                      (VELOCITY) */
  float torque;                   /**< N.m at the shaft           (TORQUE) */
  float torque_limit;             /**< N.m;     0 = the board's hard ceiling */
  float vel_limit;                /**< rad/s;   0 = the board's hard ceiling */
  float accel_limit;              /**< rad/s^2; 0 = no ramp, the reference steps */
};

/* -------------------------------------------------------------------------
 * Messages: board -> MIB (the board owns its state and resends it, changed or not)
 * ---------------------------------------------------------------------- */

enum class BoardState : uint8_t {
  DISARMED = 0,      /**< Hi-Z */
  ARMED = 1,         /**< running `mode` */
  SAFE_STOPPING = 2, /**< ramping velocity to zero */
  HOLDING = 3,       /**< safe stop finished, holding position */
  FAULT = 4,         /**< latched; see fault_code; clear with clear_fault_req_id */
};

/** Why the board is in FAULT. Every fault latches until cleared. */
enum class FaultCode : uint8_t {
  NONE = 0,
  WATCHDOG = 1,        /**< no MotorCommand for 200 ms; the board is HOLDING under it */
  OVERCURRENT = 2,     /**< phase current over the board trip level */
  VDS_OCP = 3,         /**< gate driver VDS over-current trip; see drv_status */
  OVERTEMP = 4,        /**< a board temperature over its limit; see temps */
  ENCODER = 5,         /**< angle source invalid */
  DRV_FAULT = 6,       /**< any other gate-driver fault; see drv_status */
  UNASSIGNED_AXIS = 7, /**< AxisId::UNASSIGNED in NVS. The board has no topic to publish
                            on, so this shows on its console. */
};

/** One axis's state, published at 20 Hz. Byte fields first, then 16-bit, 32-bit, floats:
    XCDR1 aligns each field to its size, so this order pads 2 bytes. 80 bytes on the wire. */
struct MotorState {
  uint8_t api_version;        /**< RAMMP_MOTOR_API_VERSION the board was built with */
  AxisId axis_id;             /**< which board this is */
  uint8_t seq;                /**< +1 per message, wraps */
  uint8_t last_cmd_seq;       /**< seq of the last MotorCommand received */
  BoardState state;           /**< what the board is doing */
  ControlMode mode;           /**< mode in force; COAST unless ARMED */
  FaultCode fault_code;       /**< NONE unless state == FAULT */
  uint8_t clear_fault_ack;    /**< last clear_fault_req_id acted on; 0 = none yet */
  uint8_t vbus_measured;      /**< 0 = vbus is the board's compiled constant, 1 = measured */
  uint8_t flags;              /**< reserved, send 0 */
  uint16_t cmd_age_ms;        /**< time since last_cmd_seq arrived; saturates at 65535 */
  uint16_t drv_status;        /**< raw DRV8353 fault bits, for diagnosis */
  uint32_t uptime_ms;         /**< board time since boot, wraps at 49.7 days */
  uint32_t fw_version;        /**< board firmware, packed major<<24 | minor<<16 | patch */
  float position;             /**< rad, multiturn from boot */
  float velocity;             /**< rad/s */
  float torque_est;           /**< N.m, Kt * iq */
  float iq;                   /**< A, torque-producing current */
  float id;                   /**< A, flux current */
  float vbus;                 /**< V; see vbus_measured */
  float ibus_est;             /**< A, (vd*id + vq*iq) / vbus */
  float torque_limit_eff;     /**< N.m, the limit in force after clamping */
  float vel_limit_eff;        /**< rad/s */
  float accel_limit_eff;      /**< rad/s^2; 0 = no ramp */
  std::array<float, 4> temps; /**< C, the four board sensors */
};

} // namespace rammp

#endif /* RAMMP_RTPS_MOTOR_MESSAGE_HPP */
