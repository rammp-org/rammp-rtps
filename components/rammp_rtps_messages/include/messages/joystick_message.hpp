/**
 * @file joystick_message.hpp
 * @brief RAMMP joystick <-> MCB RTPS messages: topics, types, and payload structs.
 * Any publisher or subscriber of these topics uses this header, so both sides match.
 *
 * - C++20. Each struct IS the wire layout: espp/cdr serializes it as XCDR1 (classic
 *   little-endian CDR, what DDS / ROS 2 peers speak), fields in declaration order.
 * - Enums are scoped with a fixed wire width, and Topic<Message> ties each topic to its
 *   message: a wrong value, message or handler does not compile.
 * - IDL: uint8_t = octet, int8_t = int8, int32_t = long, uint32_t = unsigned long,
 *   float = float, std::string = string, std::vector<T> = sequence<T>; an enum is its width.
 * - Best-effort, no durability: state is resent periodically. Timing is each device's own.
 *
 * Contents: Topics, Tables (actuators, diagnostics), Messages, Names and helpers.
 *
 * Example (espp, `rtps` a started espp::RtpsParticipant): an MCB publishing Diagnostics
 * and taking the joystick's actuator requests
 *
 *   espp::Publisher<rammp::Diagnostics> diag_pub(
 *       rtps, {.topic = rammp::kMcbDiagnostics.name, .type_name = rammp::kMcbDiagnostics.type});
 *   diag_pub.publish({.seq = seq++, .items = {{.values = {305, 150, 450}}}});
 *
 *   espp::Subscriber<rammp::ActuatorCommand> cmd_sub(
 *       rtps, {.topic = rammp::kActuatorCommand.name,
 *              .type_name = rammp::kActuatorCommand.type,
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

namespace rammp {

/* -------------------------------------------------------------------------
 * Topics: every topic and the message it carries.
 * ---------------------------------------------------------------------- */

template <class Message> struct Topic {
  const char *name; /**< DDS topic name */
  const char *type; /**< DDS type name */
};

struct XYTwist;
struct ActuatorCommand;
struct McbStatus;
struct ActuatorState;
struct Diagnostics;

/* joystick -> MCB */
inline constexpr Topic<XYTwist> kJoystickXYTwist{"rammp/joystick/xy_twist", "rammp/msg/XYTwist"};
inline constexpr Topic<ActuatorCommand> kActuatorCommand{"rammp/actuator/command",
                                                         "rammp/msg/ActuatorCommand"};
/* MCB -> joystick */
inline constexpr Topic<McbStatus> kMcbStatus{"rammp/mcb/status", "rammp/msg/McbStatus"};
inline constexpr Topic<ActuatorState> kActuatorState{"rammp/actuator/state",
                                                     "rammp/msg/ActuatorState"};
inline constexpr Topic<Diagnostics> kMcbDiagnostics{"rammp/mcb/diagnostics",
                                                    "rammp/msg/Diagnostics"};

/* -------------------------------------------------------------------------
 * Tables: one row per actuator / diagnostics item. To add one, add ONE row: the next
 * id, and a trailing `\` on every row but the last. The MCB then sends one more value
 * per row (ActuatorState.values, Diagnostics.items). Values are raw integers; decimals
 * are display only (2500 with 1 decimal shows "250.0").
 * ---------------------------------------------------------------------- */

/* X(id, NAME, short, label, min, max, step, decimals, unit) */
#define RAMMP_ACTUATOR_TABLE(X)                                                                    \
  X(0, ELEVATION, "M1", "Elevation", 0, 2500, 50, 1, "mm")                                         \
  X(1, REAR_TILT, "M2", "Rear Tilt", 0, 900, 25, 1, "deg")                                         \
  X(2, FORWARD_TILT, "M3", "Forward Tilt", 0, 450, 25, 1, "deg")                                   \
  X(3, SIDE_TILT, "M4", "Side Tilt", -300, 300, 25, 1, "deg")

/* D(id, NAME, short, label, unit1, dec1, unit2, dec2, unit3, dec3); a unit "" = unused */
#define RAMMP_DIAG_TABLE(D)                                                                        \
  D(0, TEST_1, "T1", "Test actuator 1", "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)           \
  D(1, TEST_2, "T2", "Test actuator 2", "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)           \
  D(2, TEST_3, "T3", "Test actuator 3", "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)

/* Generated from the tables: the ids (ActuatorId::ELEVATION = 0, ...) and the rows. */

enum class ActuatorId : uint8_t {
#define RAMMP_ACTUATOR_ID(id_, name_, ...) name_ = id_,
  RAMMP_ACTUATOR_TABLE(RAMMP_ACTUATOR_ID)
#undef RAMMP_ACTUATOR_ID
};

struct ActuatorSpec {
  ActuatorId id;
  const char *short_name; /**< "M1" */
  const char *label;      /**< "Elevation" */
  int32_t min_value;      /**< raw units */
  int32_t max_value;      /**< raw units */
  int32_t step;           /**< raw units per press */
  uint8_t decimals;       /**< display only */
  const char *unit;       /**< "mm" */
};

inline constexpr std::array kActuators{
#define RAMMP_ACTUATOR_ROW(id_, name_, short_, label_, min_, max_, step_, dec_, unit_)             \
  ActuatorSpec{ActuatorId::name_, short_, label_, min_, max_, step_, dec_, unit_},
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
  const char *short_name;                     /**< "T1" */
  const char *label;                          /**< "Test actuator 1" */
  std::array<const char *, kDiagFields> unit; /**< "Temp [C]"; "" = reading unused */
  std::array<uint8_t, kDiagFields> decimals;  /**< display only */
};

inline constexpr std::array kDiagItems{
#define RAMMP_DIAG_ROW(id_, name_, short_, label_, u1_, d1_, u2_, d2_, u3_, d3_)                   \
  DiagSpec{DiagId::name_, short_, label_, {u1_, u2_, u3_}, {d1_, d2_, d3_}},
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
  int8_t steps;           /**< -1 = one "-" press, +1 = one "+" press */
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
  ERROR = 1, /**< a fault: the joystick blocks drive/seat and shows error_text */
};

inline constexpr uint8_t kSpeedMaxTenths = 99; /**< speed_tenths 0..99, shown as 0.0..9.9 */

/** Drive status, system state, speed, clock and texts. */
struct McbStatus {
  DriveStatus drive_status;     /**< what the chair does with the stick */
  SystemState system_state;     /**< OK, or a fault that blocks drive/seat */
  uint8_t flags;                /**< reserved, send 0 */
  uint8_t seq;                  /**< +1 per message, wraps */
  uint8_t speed_tenths;         /**< 0..kSpeedMaxTenths */
  uint8_t hour, minute, second; /**< MCB local time */
  uint8_t day, month, year;     /**< month 0 = time unknown; year since 2000 */
  std::string drive_text;       /**< "" = show the drive_status name */
  std::string state_text;       /**< "" = show the system_state name */
  std::string error_text;       /**< banner body while system_state != OK */
  std::string error_footer;     /**< banner footer */
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

/* -------------------------------------------------------------------------
 * Names and helpers
 * ---------------------------------------------------------------------- */

constexpr Buttons operator|(Buttons a, Buttons b) {
  return static_cast<Buttons>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
constexpr bool has(Buttons set, Buttons button) {
  return (static_cast<uint32_t>(set) & static_cast<uint32_t>(button)) != 0;
}

constexpr size_t index_of(ActuatorId id) { return static_cast<size_t>(id); }

constexpr const char *to_string(DriveStatus v) {
  return v == DriveStatus::ACTIVE ? "ACTIVE" : v == DriveStatus::INACTIVE ? "INACTIVE" : "?";
}

constexpr const char *to_string(SystemState v) {
  return v == SystemState::OK ? "OK" : v == SystemState::ERROR ? "ERROR" : "?";
}

constexpr const char *to_string(ActuatorResult v) {
  return v == ActuatorResult::OK           ? "OK"
         : v == ActuatorResult::AT_MIN     ? "AT_MIN"
         : v == ActuatorResult::AT_MAX     ? "AT_MAX"
         : v == ActuatorResult::INHIBITED  ? "INHIBITED"
         : v == ActuatorResult::UNKNOWN_ID ? "UNKNOWN_ID"
                                           : "?";
}

} // namespace rammp

#endif /* RAMMP_RTPS_JOYSTICK_MESSAGE_HPP */
