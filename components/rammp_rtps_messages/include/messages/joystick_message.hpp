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
 * Contents: Topics and types, Tables (seat axes, diagnostics), Messages (MotorJog and
 * Motors are the motor debug list: the motors themselves are in messages/motor_message.hpp).
 *
 * The MIB's own state - what the chair is doing, where the seat is - is MIB::MibStatus
 * in messages/mib_message.hpp, which this header includes for MIB::DriveProfile.
 *
 * Example (espp, `rtps` a started espp::RtpsParticipant): a MIB taking the joystick's
 * seat requests and answering with its status
 *
 *   espp::Publisher<MIB::MibStatus> status_pub(
 *       rtps, {.topic = MIB::kMibStatus.name, .type_name = MIB::kMibStatus.type});
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

#include "messages/mib_message.hpp"   /* MIB::DriveProfile, and MibStatus for the topic list */
#include "messages/motor_message.hpp" /* BoardState and FaultCode, for the motor list */
#include "messages/topic.hpp"

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
#define RAMMP_TOPIC_JOYSTICK_MOTOR_JOG "rammp/joystick/motor_jog"
#define RAMMP_TYPE_MOTOR_JOG "rammp/msg/MotorJog"

/* MCB -> joystick. The state of the chair itself is RAMMP_TOPIC_MIB_STATUS, in
   messages/mib_message.hpp. */
#define RAMMP_TOPIC_MCB_DIAGNOSTICS "rammp/mcb/diagnostics"
#define RAMMP_TYPE_DIAGNOSTICS "rammp/msg/Diagnostics"
#define RAMMP_TOPIC_MIB_MOTORS "rammp/mib/motors"
#define RAMMP_TYPE_MOTORS "rammp/msg/Motors"

namespace rammp {

/* Topic<Message> is in messages/topic.hpp: the MIB's status topic needs it too. */

struct XYTwist;
struct DriveCommand;
struct SeatCommand;
struct MotorJog;
struct Diagnostics;
struct Motors;

inline constexpr Topic<XYTwist> kJoystickXYTwist{RAMMP_TOPIC_JOYSTICK_XY_TWIST,
                                                 RAMMP_TYPE_XY_TWIST};
inline constexpr Topic<DriveCommand> kJoystickDriveCommand{RAMMP_TOPIC_JOYSTICK_DRIVE_COMMAND,
                                                           RAMMP_TYPE_DRIVE_COMMAND};
inline constexpr Topic<SeatCommand> kJoystickSeatCommand{RAMMP_TOPIC_JOYSTICK_SEAT_COMMAND,
                                                         RAMMP_TYPE_SEAT_COMMAND};
inline constexpr Topic<MotorJog> kJoystickMotorJog{RAMMP_TOPIC_JOYSTICK_MOTOR_JOG,
                                                   RAMMP_TYPE_MOTOR_JOG};
inline constexpr Topic<Diagnostics> kMcbDiagnostics{RAMMP_TOPIC_MCB_DIAGNOSTICS,
                                                    RAMMP_TYPE_DIAGNOSTICS};
inline constexpr Topic<Motors> kMibMotors{RAMMP_TOPIC_MIB_MOTORS, RAMMP_TYPE_MOTORS};

/* -------------------------------------------------------------------------
 * Tables: one row per seat axis / diagnostics item. To add one, add ONE row: the next
 * id, and a trailing `\` on every row but the last. The MCB then sends one more value
 * per row (the matching MIB::seatState field, Diagnostics.items). `short` and `label` are the names
 * every device uses for the row. A value is a raw integer in units of 10^-decimals
 * `unit`: ELEVATION 2500 with 1 decimal and "mm" is 250.0 mm.
 * ---------------------------------------------------------------------- */

/* X(id, NAME, short, label, min, max, step, decimals, unit)
   One row per field of MIB::seatState, in that struct's order: the id indexes both.
   Ranges are raw integers in units of 10^-decimals `unit`; the wire carries the same
   quantity as a float in whole units, so raw 2500 with 1 decimal is 250.0 mm. */
#define RAMMP_SEAT_AXIS_TABLE(X)                                                                   \
  X(0, FRONT_BACK_TILT, "M1", "FB Tilt", -450, 900, 25, 1, "deg")                                  \
  X(1, LATERAL_TILT, "M2", "Side Tilt", -300, 300, 25, 1, "deg")                                   \
  X(2, ELEVATION, "M3", "Elevation", 0, 2500, 50, 1, "mm")                                         \
  X(3, TRANSLATION, "M4", "Translation", 0, 500, 25, 1, "mm")

/* D(id, NAME, short, label, unit1, dec1, unit2, dec2, unit3, dec3); a unit "" = unused */
#define RAMMP_DIAG_TABLE(D)                                                                        \
  D(0, TEST_1, "T1", "Test actuator 1", "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)           \
  D(1, TEST_2, "T2", "Test actuator 2", "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)           \
  D(2, TEST_3, "T3", "Test actuator 3", "Temp [C]", 1, "Current [A]", 2, "Pos [deg]", 1)           \
  D(3, IMU_TILT, "I1", "IMU tilt", "Roll [deg]", 1, "Pitch [deg]", 1, "Heading [deg]", 1)          \
  D(4, IMU_STATUS, "I2", "IMU status", "Yaw rate [deg/s]", 1, "Gyro cal [0-3]", 0, "Temp [C]", 0)

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
  DISABLE = 0,            /**< stop driving on the stick */
  ENABLE = 1,             /**< drive on the stick */
  PUSH_ASSIST_ENABLE = 2, /**< from IDLE: an assistant drives from the push handle, the stick is not read for driving */
  PUSH_ASSIST_DISABLE = 3 /**< leave push assist, back to IDLE */
};

/** Ask the MIB to enable or disable driving, with the profile to drive with, or to enter or
    leave push assist. Sent on change; the joystick shows driving only once MibStatus says
    ENABLED, and the push-assist page only once it says PUSH_ASSIST. The profile is
    MIB::DriveProfile, the one the MIB reports back in MibStatus.activeProfile; push assist
    ignores it. */
struct DriveCommand {
  DriveRequest request;      /**< enable or disable */
  MIB::DriveProfile profile; /**< which profile to drive with */
};

/** Ask the MIB to put one seat axis at `target`. Absolute, so a lost or repeated
    message cannot drift the seat; the MIB clamps to the axis' min/max. The target is in
    whole units (degrees, millimetres), matching the MIB::seatState field it moves. */
struct SeatCommand {
  SeatAxis axis; /**< which axis to move */
  float target;  /**< where to put it, in the axis' unit */
};

/** Ask the MIB to jog one motor, for debugging: hold-to-run. Resent at least every
    100 ms while held; the MIB stops the motor 300 ms after the last one, so a lost
    link or a released button stops it. The MIB picks the speed, and jogs only while
    it is IDLE and the e-stop is clear. */
struct MotorJog {
  uint8_t axis_id;  /**< a Motors entry's axis_id */
  int8_t direction; /**< -1 or +1; 0 = stop now */
};

/* -------------------------------------------------------------------------
 * Messages: MCB -> joystick (the MCB owns the state and resends it, changed or not)
 * ---------------------------------------------------------------------- */

struct DiagItem {
  std::vector<int32_t> values; /**< raw readings, in RAMMP_DIAG_TABLE unit order */
};

/** Live readings, one item per RAMMP_DIAG_TABLE row. */
struct Diagnostics {
  uint8_t seq;                 /**< +1 per message, wraps */
  std::vector<DiagItem> items; /**< one per RAMMP_DIAG_TABLE row */
};

/** One motor board, as the MIB last heard from it. */
struct MotorSummary {
  uint8_t axis_id;      /**< AxisId as a number: a joystick lists it, never switches on it */
  uint8_t online;       /**< 1 = a MotorState arrived recently; 0 = the rest is stale */
  BoardState state;     /**< MotorState.state */
  FaultCode fault_code; /**< MotorState.fault_code */
  float position;       /**< rad, MotorState.position */
  float velocity;       /**< rad/s, MotorState.velocity */
  std::string label;    /**< "Drive L", the axis table's label */
};

/** The motors the MIB commands: one entry per RAMMP_AXIS_TABLE row, in table order, at
    about 5 Hz. The joystick builds its motor list from this, so a row added to or
    removed from the table needs only the MIB rebuilt. */
struct Motors {
  uint8_t seq;                      /**< +1 per message, wraps */
  std::vector<MotorSummary> motors; /**< one per axis */
};

} // namespace rammp

#endif /* RAMMP_RTPS_JOYSTICK_MESSAGE_HPP */
