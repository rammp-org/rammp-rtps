/**
 * @file joystick_message.hpp
 * @brief RAMMP joystick <-> MCB RTPS messages: topics, types, and payload structs.
 * Any publisher or subscriber of these topics uses this header, so both sides match.
 * Only what both sides must agree on lives here; timing and helpers are each device's own.
 *
 * - C++20. Each struct IS the wire layout: espp/cdr serializes it as XCDR1 (classic
 *   little-endian CDR, what DDS / ROS 2 peers speak), fields in declaration order.
 * - Enums are scoped with a fixed wire width, and Topic<Message> ties each topic to its
 *   message: a wrong value, message or handler does not compile.
 * - IDL: uint8_t = octet, int8_t = int8, int32_t = long, uint32_t = unsigned long,
 *   float = float, std::string = string, std::vector<T> = sequence<T>; an enum is its width.
 * - Best-effort, no durability: state is resent periodically.
 * - The joystick asks, the MCB decides: a command is a request, and the joystick shows
 *   nothing until the matching state message says it happened.
 *
 * Naming, for every message:
 *   struct <Message>                    SeatCommand
 *   RAMMP_TYPE_<MESSAGE>                RAMMP_TYPE_SEAT_COMMAND  "rammp/msg/SeatCommand"
 *   RAMMP_TOPIC_<PUBLISHER>_<MESSAGE>   RAMMP_TOPIC_JOYSTICK_SEAT_COMMAND
 *   k<Publisher><Message>               kJoystickSeatCommand     the typed Topic<>
 *
 * Contents: Topics and types, Tables (seat axes, diagnostics), Messages.
 *
 * Example (espp, `rtps` a started espp::RtpsParticipant): an MCB taking the joystick's
 * seat requests and answering with the seat state
 *
 *   espp::Publisher<rammp::SeatState> seat_pub(
 *       rtps, {.topic = rammp::kMcbSeatState.name, .type_name = rammp::kMcbSeatState.type});
 *
 *   espp::Subscriber<rammp::SeatCommand> seat_sub(
 *       rtps, {.topic = rammp::kJoystickSeatCommand.name,
 *              .type_name = rammp::kJoystickSeatCommand.type,
 *              .on_message = [](const rammp::SeatCommand &cmd) {
 *                move_seat(cmd.axis, cmd.target); // clamp to the axis' range, then publish
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
#define RAMMP_TOPIC_JOYSTICK_DRIVE_COMMAND "rammp/joystick/drive_command"
#define RAMMP_TYPE_DRIVE_COMMAND "rammp/msg/DriveCommand"
#define RAMMP_TOPIC_JOYSTICK_SEAT_COMMAND "rammp/joystick/seat_command"
#define RAMMP_TYPE_SEAT_COMMAND "rammp/msg/SeatCommand"

/* MCB -> joystick */
#define RAMMP_TOPIC_MCB_SYSTEM_STATE "rammp/mcb/system_state"
#define RAMMP_TYPE_SYSTEM_STATE "rammp/msg/SystemState"
#define RAMMP_TOPIC_MCB_SEAT_STATE "rammp/mcb/seat_state"
#define RAMMP_TYPE_SEAT_STATE "rammp/msg/SeatState"
#define RAMMP_TOPIC_MCB_DIAGNOSTICS "rammp/mcb/diagnostics"
#define RAMMP_TYPE_DIAGNOSTICS "rammp/msg/Diagnostics"

namespace rammp {

/** A DDS topic and type name, tied to the message it carries. */
template <class Message> struct Topic {
  const char *name; /**< DDS topic name */
  const char *type; /**< DDS type name */
};

struct XYTwist;
struct DriveCommand;
struct SeatCommand;
struct SystemState;
struct SeatState;
struct Diagnostics;

inline constexpr Topic<XYTwist> kJoystickXYTwist{RAMMP_TOPIC_JOYSTICK_XY_TWIST,
                                                 RAMMP_TYPE_XY_TWIST};
inline constexpr Topic<DriveCommand> kJoystickDriveCommand{RAMMP_TOPIC_JOYSTICK_DRIVE_COMMAND,
                                                           RAMMP_TYPE_DRIVE_COMMAND};
inline constexpr Topic<SeatCommand> kJoystickSeatCommand{RAMMP_TOPIC_JOYSTICK_SEAT_COMMAND,
                                                         RAMMP_TYPE_SEAT_COMMAND};
inline constexpr Topic<SystemState> kMcbSystemState{RAMMP_TOPIC_MCB_SYSTEM_STATE,
                                                    RAMMP_TYPE_SYSTEM_STATE};
inline constexpr Topic<SeatState> kMcbSeatState{RAMMP_TOPIC_MCB_SEAT_STATE, RAMMP_TYPE_SEAT_STATE};
inline constexpr Topic<Diagnostics> kMcbDiagnostics{RAMMP_TOPIC_MCB_DIAGNOSTICS,
                                                    RAMMP_TYPE_DIAGNOSTICS};

/* -------------------------------------------------------------------------
 * Tables: one row per seat axis / diagnostics item. To add one, add ONE row: the next
 * id, and a trailing `\` on every row but the last. The MCB then sends one more value
 * per row (SeatState.values, Diagnostics.items). `short` and `label` are the names
 * every device uses for the row. A value is a raw integer in units of 10^-decimals
 * `unit`: ELEVATION 2500 with 1 decimal and "mm" is 250.0 mm.
 * ---------------------------------------------------------------------- */

/* X(id, NAME, short, label, min, max, step, decimals, unit) */
#define RAMMP_SEAT_AXIS_TABLE(X)                                                                   \
  X(0, ELEVATION, "M1", "Elevation", 0, 2500, 50, 1, "mm")                                         \
  X(1, REAR_TILT, "M2", "Rear Tilt", 0, 900, 25, 1, "deg")                                         \
  X(2, FORWARD_TILT, "M3", "FW Tilt", 0, 450, 25, 1, "deg")                                        \
  X(3, SIDE_TILT, "M4", "Side Tilt", -300, 300, 25, 1, "deg")

/* D(id, NAME, short, label, unit1, dec1, unit2, dec2, unit3, dec3); a unit "" = unused */
#define RAMMP_DIAG_TABLE(D)                                                                        \
  D(0, TEST_1, "T1", "Test actuator 1", "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)           \
  D(1, TEST_2, "T2", "Test actuator 2", "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)           \
  D(2, TEST_3, "T3", "Test actuator 3", "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)

/* Generated from the tables: the ids (SeatAxis::ELEVATION = 0, ...) and the rows. */

enum class SeatAxis : uint8_t {
#define RAMMP_SEAT_AXIS_ID(id_, name_, ...) name_ = id_,
  RAMMP_SEAT_AXIS_TABLE(RAMMP_SEAT_AXIS_ID)
#undef RAMMP_SEAT_AXIS_ID
};

struct SeatAxisSpec {
  SeatAxis id;
  const char *short_name; /**< "M1" */
  const char *label;      /**< "Elevation" */
  int32_t min_value;      /**< raw units */
  int32_t max_value;      /**< raw units */
  int32_t step;           /**< raw units per step the joystick asks for */
  uint8_t decimals;       /**< raw units are 10^-decimals `unit` */
  const char *unit;       /**< "mm" */
};

inline constexpr std::array kSeatAxes{
#define RAMMP_SEAT_AXIS_ROW(id_, name_, short_, label_, min_, max_, step_, dec_, unit_)            \
  SeatAxisSpec{SeatAxis::name_, short_, label_, min_, max_, step_, dec_, unit_},
    RAMMP_SEAT_AXIS_TABLE(RAMMP_SEAT_AXIS_ROW)
#undef RAMMP_SEAT_AXIS_ROW
};
inline constexpr size_t kSeatAxisCount = kSeatAxes.size();

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
  std::array<uint8_t, kDiagFields> decimals;  /**< raw readings are 10^-decimals unit */
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

/** The stick, calibrated on the joystick: 0 at rest, deadzones applied, X/Y within the
    unit circle. Twist always rotates in place. Sent continuously; the MCB drives on it
    only while it has enabled driving (see DriveCommand). */
struct XYTwist {
  float x;         /**< -1..+1, + = right */
  float y;         /**< -1..+1, + = forward */
  float twist;     /**< -1..+1, + = clockwise */
  Buttons buttons; /**< pressed buttons */
};

enum class DriveRequest : uint8_t {
  DISABLE = 0, /**< stop driving on the stick */
  ENABLE = 1,  /**< drive on the stick */
};

/** How the MCB should drive: chosen on the joystick, confirmed in SystemState. */
enum class DriveProfile : uint8_t {
  NORMAL = 0, /**< car-like: Y drives, X steers */
  HOLO = 1,   /**< holonomic: X/Y is the velocity vector */
  AUTO = 2,   /**< reserved, not implemented */
};

/** Ask the MCB to enable or disable driving, with the profile to drive with. Sent on
    change; the joystick shows driving only once SystemState confirms it. */
struct DriveCommand {
  DriveRequest request; /**< enable or disable */
  DriveProfile profile; /**< which profile to drive with */
};

/** Ask the MCB to put one seat axis at `target`. Absolute, so a lost or repeated
    message cannot drift the seat; the MCB clamps to the axis' min/max. */
struct SeatCommand {
  SeatAxis axis;  /**< which axis to move */
  int32_t target; /**< where to put it, in the axis' raw units */
};

/* -------------------------------------------------------------------------
 * Messages: MCB -> joystick (the MCB owns the state and resends it, changed or not)
 * ---------------------------------------------------------------------- */

enum class DriveStatus : uint8_t {
  INACTIVE = 0, /**< chair ignores the stick */
  ACTIVE = 1,   /**< chair drives on the stick */
};

enum class FaultState : uint8_t {
  OK = 0,    /**< drive and seat allowed */
  ERROR = 1, /**< a fault: drive and seat not allowed; error_text says why */
};

inline constexpr uint8_t kSpeedMaxTenths = 99; /**< speed_tenths is 0..99 (0.0..9.9) */

/** What the chair is doing: the answer to DriveCommand, and what the joystick shows. */
struct SystemState {
  DriveStatus drive_status;     /**< what the chair does with the stick */
  FaultState fault;             /**< OK, or a fault */
  DriveProfile profile;         /**< the profile the MCB accepted */
  uint8_t flags;                /**< reserved, send 0 */
  uint8_t seq;                  /**< +1 per message, wraps */
  uint8_t speed_tenths;         /**< 0..kSpeedMaxTenths */
  uint8_t hour, minute, second; /**< MCB local time */
  uint8_t day, month, year;     /**< month 0 = time unknown; year since 2000 */
  std::string drive_text;       /**< optional wording for drive_status; "" = none */
  std::string state_text;       /**< optional wording for fault; "" = none */
  std::string error_text;       /**< what the fault is, while fault != OK */
  std::string error_footer;     /**< what to do about it */
};

/** The MCB's verdict on a SeatCommand. Anything but OK = the axis did not move. */
enum class SeatResult : uint8_t {
  OK = 0,           /**< moving, or already there */
  AT_MIN = 1,       /**< already at its low end */
  AT_MAX = 2,       /**< already at its high end */
  INHIBITED = 3,    /**< refused now: interlock, fault, driving */
  UNKNOWN_AXIS = 4, /**< no such axis */
};

/** Where every seat axis is, on change and periodically. The joystick's seat screen
    shows these values and nothing else. */
struct SeatState {
  SeatResult result;           /**< verdict on the last SeatCommand */
  SeatAxis last_axis;          /**< the axis that verdict is about */
  uint8_t seq;                 /**< +1 per message, wraps */
  std::vector<int32_t> values; /**< one per RAMMP_SEAT_AXIS_TABLE row, raw units */
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
