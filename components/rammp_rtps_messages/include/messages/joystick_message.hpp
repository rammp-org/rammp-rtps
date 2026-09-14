/**
 * @file joystick_message.hpp
 * @brief RAMMP joystick RTPS message definitions.
 * Defines the topics, types, and payload structs the joystick (HMI) and the MCB exchange.
 * Any publisher or subscriber of these messages should use this header so the publisher and
 * subscriber match the message definition. New message types can be added later.
 *
 * - C++20. The structs ARE the wire layout: espp/cdr serializes them as XCDR1 (classic
 *   little-endian CDR, what DDS / ROS 2 peers speak), fields in declaration order.
 * - Every enum is scoped with a fixed wire width, and every topic carries its message type
 *   (Topic<XYTwist>), so the wrong value, message or handler for a topic does not compile.
 * - IDL mapping: uint8_t / enum : uint8_t = octet, int8_t = int8, int32_t = long,
 *   uint32_t / enum : uint32_t = unsigned long, float = float, std::string = string,
 *   std::vector<T> = sequence<T>.
 * - Every topic is best-effort, no durability: state is resent periodically. How often,
 *   and how long a device waits before calling a peer lost, is each device's own business.
 *
 * Example (espp): an MCB sending Diagnostics and taking the joystick's actuator requests
 *
 *   #include "rtps_pubsub.hpp"
 *   #include "messages.hpp"
 *
 *   espp::RtpsParticipant rtps({.interface_address = my_ip});
 *   rtps.start();
 *   espp::Publisher<rammp::Diagnostics> diag_pub(
 *       rtps, {.topic = rammp::kMcbDiagnostics.name, .type_name = rammp::kMcbDiagnostics.type});
 *   diag_pub.publish({.seq = seq++, .items = {{.values = {305, 150, 450}}}}); // T1: 30.5 C ...
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
 * Topic: a DDS topic and type name, tied to the message it carries.
 * ---------------------------------------------------------------------- */

template <class Message> struct Topic {
  const char *name; /**< DDS topic name */
  const char *type; /**< DDS type name */
};

/* -------------------------------------------------------------------------
 * XYTwist: joystick -> MCB normalized stick position, button state, and mode.
 * Calibrated on the joystick: 0 at rest, deadzones applied, X/Y within the unit circle.
 * ---------------------------------------------------------------------- */

enum class Buttons : uint32_t { /**< bitfield; bit set = pressed */
  NONE = 0x0,
  JOYSTICK = 0x1, /**< the stick's button */
};
constexpr Buttons operator|(Buttons a, Buttons b) {
  return static_cast<Buttons>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
constexpr bool has(Buttons set, Buttons button) {
  return (static_cast<uint32_t>(set) & static_cast<uint32_t>(button)) != 0;
}

enum class DriveMode : uint32_t {
  NORMAL = 0, /**< car-like: Y drives, X steers */
  HOLO = 1,   /**< holonomic: X/Y is the velocity vector */
  AUTO = 2,   /**< reserved, not implemented */
};

struct XYTwist {
  float x;              /**< normalized -1.0..1.0, + = right */
  float y;              /**< normalized -1.0..1.0, + = forward */
  float twist;          /**< normalized -1.0..1.0, + = clockwise; always rotates in place */
  Buttons buttons;      /**< pressed buttons */
  DriveMode drive_mode; /**< drive mode selected by the joystick */
};

inline constexpr Topic<XYTwist> kJoystickXYTwist{"rammp/joystick/xy_twist", "rammp/msg/XYTwist"};

/* -------------------------------------------------------------------------
 * McbStatus: MCB -> joystick drive status, system state, speed, clock, and texts.
 * The MCB owns the vehicle state and resends it periodically, changed or not.
 * ---------------------------------------------------------------------- */

enum class DriveStatus : uint8_t {
  INACTIVE = 0, /**< chair ignores the stick */
  ACTIVE = 1,   /**< chair drives on the stick */
};

enum class SystemState : uint8_t {
  OK = 0,
  ERROR = 1, /**< a fault: the joystick blocks drive/seat and shows error_text */
};

inline constexpr uint8_t kSpeedMaxTenths = 99; /**< speed_tenths 0..99, shown as 0.0..9.9 */

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

inline constexpr Topic<McbStatus> kMcbStatus{"rammp/mcb/status", "rammp/msg/McbStatus"};

constexpr const char *to_string(DriveStatus v) {
  switch (v) {
  case DriveStatus::INACTIVE:
    return "INACTIVE";
  case DriveStatus::ACTIVE:
    return "ACTIVE";
  }
  return "?";
}

constexpr const char *to_string(SystemState v) {
  switch (v) {
  case SystemState::OK:
    return "OK";
  case SystemState::ERROR:
    return "ERROR";
  }
  return "?";
}

/* -------------------------------------------------------------------------
 * Actuators: joystick -> MCB step requests (ActuatorCommand),
 *            MCB -> joystick positions and verdicts (ActuatorState).
 * The MCB owns every position. Values are raw integers; `decimals` is display only
 * (2500 with 1 shows "250.0").
 *
 * To add an actuator, add ONE X(...) row: next id, and a trailing `\` on every row but
 * the last. ActuatorId and kActuators follow; the MCB then sends one more value in
 * ActuatorState.values and accepts the new id in ActuatorCommand.
 * ---------------------------------------------------------------------- */

/* X(id, NAME, short, label, min, max, step, decimals, unit); id = row index */
#define RAMMP_ACTUATOR_TABLE(X)                                                                    \
  X(0, ELEVATION, "M1", "Elevation", 0, 2500, 50, 1, "mm")                                         \
  X(1, REAR_TILT, "M2", "Rear Tilt", 0, 900, 25, 1, "deg")                                         \
  X(2, FORWARD_TILT, "M3", "Forward Tilt", 0, 450, 25, 1, "deg")                                   \
  X(3, SIDE_TILT, "M4", "Side Tilt", -300, 300, 25, 1, "deg")

enum class ActuatorId : uint8_t { /**< ELEVATION = 0, ... (the table's rows) */
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

constexpr size_t index_of(ActuatorId id) { return static_cast<size_t>(id); }

/** The MCB's verdict on a command. Anything but OK = value unchanged. */
enum class ActuatorResult : uint8_t {
  OK = 0,
  AT_MIN = 1,     /**< already at its low end */
  AT_MAX = 2,     /**< already at its high end */
  INHIBITED = 3,  /**< refused now: interlock, fault, driving */
  UNKNOWN_ID = 4, /**< no such actuator */
};

struct ActuatorCommand {
  uint8_t req_id;         /**< +1 per command, wraps; echoed back in ActuatorState */
  ActuatorId actuator_id; /**< RAMMP_ACTUATOR_TABLE row */
  int8_t steps;           /**< -1 = one "-" press, +1 = one "+" press */
};

struct ActuatorState {
  uint8_t req_id;              /**< command this answers; 0 = none yet */
  ActuatorResult result;       /**< verdict on that command */
  uint8_t seq;                 /**< +1 per message, wraps */
  std::vector<int32_t> values; /**< one per table row, raw units */
};

inline constexpr Topic<ActuatorCommand> kActuatorCommand{"rammp/actuator/command",
                                                         "rammp/msg/ActuatorCommand"};
inline constexpr Topic<ActuatorState> kActuatorState{"rammp/actuator/state",
                                                     "rammp/msg/ActuatorState"};

constexpr const char *to_string(ActuatorResult v) {
  switch (v) {
  case ActuatorResult::OK:
    return "OK";
  case ActuatorResult::AT_MIN:
    return "AT_MIN";
  case ActuatorResult::AT_MAX:
    return "AT_MAX";
  case ActuatorResult::INHIBITED:
    return "INHIBITED";
  case ActuatorResult::UNKNOWN_ID:
    return "UNKNOWN_ID";
  }
  return "?";
}

/* -------------------------------------------------------------------------
 * Diagnostics: MCB -> joystick live readings from the actuators and anything else
 * worth watching. Values are raw integers; `decN` is display only (2345 with 2 shows
 * "23.45").
 *
 * To add a diagnostics item, add ONE D(...) row: next id, same `\` rule. The MCB then
 * sends one more DiagItem.
 * ---------------------------------------------------------------------- */

inline constexpr size_t kDiagFields = 3; /**< readings per item */

/* D(id, NAME, short, label, unit1, dec1, unit2, dec2, unit3, dec3); id = row index.
   A unit of "" leaves that reading out. */
#define RAMMP_DIAG_TABLE(D)                                                                        \
  D(0, TEST_1, "T1", "Test actuator 1", "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)           \
  D(1, TEST_2, "T2", "Test actuator 2", "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)           \
  D(2, TEST_3, "T3", "Test actuator 3", "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)

enum class DiagId : uint8_t { /**< TEST_1 = 0, ... (the table's rows) */
#define RAMMP_DIAG_ID(id_, name_, ...) name_ = id_,
  RAMMP_DIAG_TABLE(RAMMP_DIAG_ID)
#undef RAMMP_DIAG_ID
};

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

struct DiagItem {
  std::vector<int32_t> values; /**< raw readings, in RAMMP_DIAG_TABLE unit order */
};

struct Diagnostics {
  uint8_t seq;                 /**< +1 per message, wraps */
  std::vector<DiagItem> items; /**< one per RAMMP_DIAG_TABLE row */
};

inline constexpr Topic<Diagnostics> kMcbDiagnostics{"rammp/mcb/diagnostics",
                                                    "rammp/msg/Diagnostics"};

} // namespace rammp

#endif /* RAMMP_RTPS_JOYSTICK_MESSAGE_HPP */
