/**
 * @file joystick_message.hpp
 * @brief RAMMP joystick <-> MCB RTPS messages: topics, types, and payload structs.
 * Any publisher or subscriber of these topics uses this header, so both sides match.
 * Only what both sides must agree on lives here; timing, display names and helpers are
 * each device's own.
 *
 * - C++20. Each struct IS the wire layout: espp/cdr serializes it as XCDR1 (classic
 *   little-endian CDR, what DDS / ROS 2 peers speak), fields in declaration order.
 * - Enums are scoped with a fixed wire width, and Topic<Message> ties each topic to its
 *   message: a wrong value, message or handler does not compile.
 * - IDL: uint8_t = octet, int8_t = int8, int32_t = long, uint32_t = unsigned long,
 *   float = float, std::string = string, std::vector<T> = sequence<T>; an enum is its width.
 * - Best-effort, no durability: state is resent periodically.
 *
 * Naming, for every message:
 *   struct <Message>                    XYTwist
 *   RAMMP_TYPE_<MESSAGE>                RAMMP_TYPE_XY_TWIST            "rammp/msg/XYTwist"
 *   RAMMP_TOPIC_<PUBLISHER>_<MESSAGE>   RAMMP_TOPIC_JOYSTICK_XY_TWIST  "rammp/joystick/xy_twist"
 *   k<Publisher><Message>               kJoystickXYTwist               the typed Topic<XYTwist>
 *
 * Contents: Topics and types, Tables (actuators, diagnostics), Messages.
 *
 * Example (espp, `rtps` a started espp::RtpsParticipant): an MCB publishing Diagnostics
 * and taking the joystick's actuator requests
 *
 *   espp::Publisher<rammp::Diagnostics> diag_pub(
 *       rtps, {.topic = rammp::kMcbDiagnostics.name, .type_name = rammp::kMcbDiagnostics.type});
 *   diag_pub.publish({.seq = seq++, .items = {{.values = {305, 150, 450}}}});
 *
 *   espp::Subscriber<rammp::ActuatorCommand> cmd_sub(
 *       rtps, {.topic = rammp::kJoystickActuatorCommand.name,
 *              .type_name = rammp::kJoystickActuatorCommand.type,
 *              .on_message = [](const rammp::ActuatorCommand &cmd) {
 *                move_actuator(cmd.actuator_id, cmd.steps); // then publish an ActuatorState
 *              }});
 */

#ifndef RAMMP_RTPS_JOYSTICK_MESSAGE_HPP
#define RAMMP_RTPS_JOYSTICK_MESSAGE_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/* -------------------------------------------------------------------------
 * Topics and types: rammp/<publisher>/<message> and rammp/msg/<Message>.
 * ---------------------------------------------------------------------- */

/* joystick -> MCB */
#define RAMMP_TOPIC_JOYSTICK_XY_TWIST "rammp/joystick/xy_twist"
#define RAMMP_TYPE_XY_TWIST "rammp/msg/XYTwist"
#define RAMMP_TOPIC_JOYSTICK_ACTUATOR_COMMAND "rammp/joystick/actuator_command"
#define RAMMP_TYPE_ACTUATOR_COMMAND "rammp/msg/ActuatorCommand"

/* MCB -> joystick */
#define RAMMP_TOPIC_MCB_STATUS "rammp/mcb/status"
#define RAMMP_TYPE_MCB_STATUS "rammp/msg/McbStatus"
#define RAMMP_TOPIC_MCB_ACTUATOR_STATE "rammp/mcb/actuator_state"
#define RAMMP_TYPE_ACTUATOR_STATE "rammp/msg/ActuatorState"
#define RAMMP_TOPIC_MCB_DIAGNOSTICS "rammp/mcb/diagnostics"
#define RAMMP_TYPE_DIAGNOSTICS "rammp/msg/Diagnostics"

namespace rammp {

/** A DDS topic and type name, tied to the message it carries. */
template <class Message> struct Topic {
  const char *name; /**< DDS topic name */
  const char *type; /**< DDS type name */
};

struct XYTwist;
struct ActuatorCommand;
struct McbStatus;
struct ActuatorState;
struct Diagnostics;

inline constexpr Topic<XYTwist> kJoystickXYTwist{RAMMP_TOPIC_JOYSTICK_XY_TWIST,
                                                 RAMMP_TYPE_XY_TWIST};
inline constexpr Topic<ActuatorCommand> kJoystickActuatorCommand{
    RAMMP_TOPIC_JOYSTICK_ACTUATOR_COMMAND, RAMMP_TYPE_ACTUATOR_COMMAND};
inline constexpr Topic<McbStatus> kMcbStatus{RAMMP_TOPIC_MCB_STATUS, RAMMP_TYPE_MCB_STATUS};
inline constexpr Topic<ActuatorState> kMcbActuatorState{RAMMP_TOPIC_MCB_ACTUATOR_STATE,
                                                        RAMMP_TYPE_ACTUATOR_STATE};
inline constexpr Topic<Diagnostics> kMcbDiagnostics{RAMMP_TOPIC_MCB_DIAGNOSTICS,
                                                    RAMMP_TYPE_DIAGNOSTICS};

/* -------------------------------------------------------------------------
 * Tables: one row per actuator / diagnostics item. To add one, add ONE row: the next
 * id, and a trailing `\` on every row but the last. The MCB then sends one more value
 * per row (ActuatorState.values, Diagnostics.items). A value is a raw integer in units
 * of 10^-decimals `unit`: ELEVATION 2500 with 1 decimal and "mm" is 250.0 mm.
 * Display names for the rows are each device's own.
 * ---------------------------------------------------------------------- */

/* X(id, NAME, min, max, step, decimals, unit) */
#define RAMMP_ACTUATOR_TABLE(X)                                                                    \
  X(0, ELEVATION, 0, 2500, 50, 1, "mm")                                                            \
  X(1, REAR_TILT, 0, 900, 25, 1, "deg")                                                            \
  X(2, FORWARD_TILT, 0, 450, 25, 1, "deg")                                                         \
  X(3, SIDE_TILT, -300, 300, 25, 1, "deg")

/* D(id, NAME, unit1, dec1, unit2, dec2, unit3, dec3); a unit "" = reading unused */
#define RAMMP_DIAG_TABLE(D)                                                                        \
  D(0, TEST_1, "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)                                    \
  D(1, TEST_2, "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)                                    \
  D(2, TEST_3, "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)

/* Generated from the tables: the ids (ActuatorId::ELEVATION = 0, ...) and the rows. */

enum class ActuatorId : uint8_t {
#define RAMMP_ACTUATOR_ID(id_, name_, ...) name_ = id_,
  RAMMP_ACTUATOR_TABLE(RAMMP_ACTUATOR_ID)
#undef RAMMP_ACTUATOR_ID
};

struct ActuatorSpec {
  ActuatorId id;
  int32_t min_value; /**< raw units */
  int32_t max_value; /**< raw units */
  int32_t step;      /**< raw units moved per ActuatorCommand step */
  uint8_t decimals;  /**< raw units are 10^-decimals `unit` */
  const char *unit;  /**< "mm" */
};

inline constexpr std::array kActuators{
#define RAMMP_ACTUATOR_ROW(id_, name_, min_, max_, step_, dec_, unit_)                             \
  ActuatorSpec{ActuatorId::name_, min_, max_, step_, dec_, unit_},
    RAMMP_ACTUATOR_TABLE(RAMMP_ACTUATOR_ROW)
#undef RAMMP_ACTUATOR_ROW
};
inline constexpr size_t kActuatorCount = kActuators.size();

enum class DiagId : uint8_t {
#define RAMMP_DIAG_ID(id_, name_, ...) name_ = id_,
  RAMMP_DIAG_TABLE(RAMMP_DIAG_ID)
#undef RAMMP_DIAG_ID
};

inline constexpr size_t kDiagFields = 3; /**< readings per diagnostics item */

struct DiagSpec {
  DiagId id;
  std::array<const char *, kDiagFields> unit; /**< "Temp [C]"; "" = reading unused */
  std::array<uint8_t, kDiagFields> decimals;  /**< raw readings are 10^-decimals unit */
};

inline constexpr std::array kDiagItems{
#define RAMMP_DIAG_ROW(id_, name_, u1_, d1_, u2_, d2_, u3_, d3_)                                   \
  DiagSpec{DiagId::name_, {u1_, u2_, u3_}, {d1_, d2_, d3_}},
    RAMMP_DIAG_TABLE(RAMMP_DIAG_ROW)
#undef RAMMP_DIAG_ROW
};
inline constexpr size_t kDiagCount = kDiagItems.size();

/* -------------------------------------------------------------------------
 * Messages: joystick -> MCB
 * ---------------------------------------------------------------------- */

/** Pressed buttons, a bit set. */
enum class Buttons : uint32_t {
  NONE = 0x0,
  JOYSTICK = 0x1, /**< the stick's button */
};

enum class DriveMode : uint32_t {
  NORMAL = 0, /**< car-like: Y drives, X steers */
  HOLO = 1,   /**< holonomic: X/Y is the velocity vector */
  AUTO = 2,   /**< reserved, not implemented */
};

/** The stick, calibrated on the joystick: 0 at rest, deadzones applied, X/Y within the
    unit circle. Twist always rotates in place. */
struct XYTwist {
  float x;              /**< -1..+1, + = right */
  float y;              /**< -1..+1, + = forward */
  float twist;          /**< -1..+1, + = clockwise */
  Buttons buttons;      /**< pressed buttons */
  DriveMode drive_mode; /**< chosen on the joystick */
};

/** Move one actuator by `steps` of its table step. The MCB answers with an ActuatorState. */
struct ActuatorCommand {
  uint8_t req_id;         /**< +1 per command, wraps; echoed back in ActuatorState */
  ActuatorId actuator_id; /**< which actuator */
  int8_t steps;           /**< -1 = one step down, +1 = one step up */
};

/* -------------------------------------------------------------------------
 * Messages: MCB -> joystick (the MCB owns the state and resends it, changed or not)
 * ---------------------------------------------------------------------- */

enum class DriveStatus : uint8_t {
  INACTIVE = 0, /**< chair ignores the stick */
  ACTIVE = 1,   /**< chair drives on the stick */
};

enum class SystemState : uint8_t {
  OK = 0,    /**< drive and seat allowed */
  ERROR = 1, /**< a fault: drive and seat not allowed; error_text says why */
};

inline constexpr uint8_t kSpeedMaxTenths = 99; /**< speed_tenths is 0..99 (0.0..9.9) */

/** Drive status, system state, speed, clock and texts. */
struct McbStatus {
  DriveStatus drive_status;     /**< what the chair does with the stick */
  SystemState system_state;     /**< OK, or a fault */
  uint8_t flags;                /**< reserved, send 0 */
  uint8_t seq;                  /**< +1 per message, wraps */
  uint8_t speed_tenths;         /**< 0..kSpeedMaxTenths */
  uint8_t hour, minute, second; /**< MCB local time */
  uint8_t day, month, year;     /**< month 0 = time unknown; year since 2000 */
  std::string drive_text;       /**< optional wording for drive_status; "" = none */
  std::string state_text;       /**< optional wording for system_state; "" = none */
  std::string error_text;       /**< what the fault is, while system_state != OK */
  std::string error_footer;     /**< what to do about it */
};

/** The MCB's verdict on a command. Anything but OK = value unchanged. */
enum class ActuatorResult : uint8_t {
  OK = 0,         /**< moved */
  AT_MIN = 1,     /**< already at its low end */
  AT_MAX = 2,     /**< already at its high end */
  INHIBITED = 3,  /**< refused now: interlock, fault, driving */
  UNKNOWN_ID = 4, /**< no such actuator */
};

/** Every actuator's position, on change and periodically. */
struct ActuatorState {
  uint8_t req_id;              /**< command this answers; 0 = none yet */
  ActuatorResult result;       /**< verdict on that command */
  uint8_t seq;                 /**< +1 per message, wraps */
  std::vector<int32_t> values; /**< one per RAMMP_ACTUATOR_TABLE row, raw units */
};

struct DiagItem {
  std::vector<int32_t> values; /**< raw readings, in RAMMP_DIAG_TABLE unit order */
};

/** Live readings, one item per RAMMP_DIAG_TABLE row. */
struct Diagnostics {
  uint8_t seq;                 /**< +1 per message, wraps */
  std::vector<DiagItem> items; /**< one per RAMMP_DIAG_TABLE row */
};

} // namespace rammp

#endif /* RAMMP_RTPS_JOYSTICK_MESSAGE_HPP */
