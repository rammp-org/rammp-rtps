/**
 * @file mib_message.hpp
 * @brief RAMMP MIB RTPS messages: the MIB's status, and the profiles, states and seat
 * pose it carries.
 *
 * The MIB publishes MibStatus for the joystick and for the ROS 2 nodes on the Jetson, so
 * the message is ROS 2 facing: its topic is the ROS 2 wire name of /rammp/mib/status
 * and its type is rammp_base_interfaces/msg/MibStatus from rammp-org/rammp-interfaces-ros2.
 * That .msg mirrors this struct field for field and is not the source of truth. An espp
 * subscriber (the joystick) uses the same two names, so one publisher serves both.
 *
 * Wire rules, the same as joystick_message.hpp: each struct IS the wire layout (espp/cdr
 * serializes it as XCDR1, fields in declaration order), and every enum is scoped with a
 * fixed width so its size on the wire is not the compiler's choice. MibStatus is resent
 * periodically, best-effort and without durability, so a subscriber that starts late
 * catches up on the next tick.
 *
 * Contents: Topics and types, drive profiles, system states, seat state, status message,
 * and the drive profile limits (DriveProfileValues, DriveProfileState).
 */

#ifndef RAMMP_RTPS_MIB_MESSAGE_HPP
#define RAMMP_RTPS_MIB_MESSAGE_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "messages/topic.hpp"

/* -------------------------------------------------------------------------
 * Topics and types: ROS 2 wire names. ROS 2 prefixes a topic with "rt/" and writes the
 * type pkg/msg/T as pkg::msg::dds_::T_, so a ROS 2 node sees /rammp/mib/status of type
 * rammp_base_interfaces/msg/MibStatus.
 * ---------------------------------------------------------------------- */

/* MIB -> joystick and ROS 2 */
#define RAMMP_ROS_TOPIC_MIB_STATUS "rt/rammp/mib/status"
#define RAMMP_ROS_TYPE_MIB_STATUS "rammp_base_interfaces::msg::dds_::MibStatus_"

namespace rammp {

struct MibStatus;

inline constexpr Topic<MibStatus> kMibStatus{RAMMP_ROS_TOPIC_MIB_STATUS, RAMMP_ROS_TYPE_MIB_STATUS};

/* -------------------------------------------------------------------------
 * Messages: MIB -> joystick and ROS 2 (the MIB owns the state and resends it)
 * ---------------------------------------------------------------------- */

/** Drive response profile. A DriveCommand asks for one; MibStatus.active_profile is the
    one in force. */
enum class DriveProfile : uint8_t {
  LOW = 0,    /**< reduced drive response */
  NORMAL = 1, /**< standard drive response */
  HIGH = 2,   /**< increased drive response */
};

/** Number of DriveProfile values; DriveProfileState holds one entry per profile. */
inline constexpr std::size_t kDriveProfileCount = 3;

/** High-level operating state of the MIB.

    The joystick picks its page from this value, so every state the joystick must draw
    differently is its own value here; MibStatus.status_text only words the label. New
    values are appended so the ones already on the wire keep their numbers. A decoder
    that meets a value it does not know should fall back to a neutral page rather than
    reject the message. */
enum class MibSystemState : uint8_t {
  INITIALIZING = 0, /**< bringing up the board; not ready */
  IDLE = 1,         /**< waiting for a DriveCommand; seat control is available, drive is disabled */
  ENABLED = 2,      /**< the stick drives; manual seat control is disabled */
  ERROR = 3,        /**< bring-up or runtime fault; error_message says why */
  AUTONOMOUS = 4,   /**< another node (the Jetson) drives on TwistSetpoint; the stick is not
                         read for driving, and touching it hands control back (IDLE) */
  SHARED = 5,       /**< another node and the stick drive together; the stick is read and
                         blended */
  ESTOP = 6,        /**< emergency stop: drive is cut and held; the MIB returns to IDLE on
                         its own once the e-stop latch clears */
  PUSH_ASSIST = 7,  /**< an assistant drives from the push handle; the stick is not read
                         for driving. Entered and left by DriveCommand.push_assist */
};

/** Where the seat is: one field per RAMMP_SEAT_AXIS_TABLE row (joystick_message.hpp), in
    that order, so a SeatAxis id indexes both. Whole units, the ones a SeatCommand asks
    in. Mirrored by rammp_base_interfaces/msg/SeatState. */
struct SeatState {
  float front_back_tilt{0.0f}; /**< deg, front-to-back seat tilt */
  float lateral_tilt{0.0f};    /**< deg, side-to-side seat tilt */
  float elevation{0.0f};       /**< mm, seat elevation */
  float translation{0.0f};     /**< mm, seat translation */
};

/** The MIB's status, every 500 ms.

    The fields after error_message are what the joystick draws: the speed readout, the
    TopBar clock (and the RTC it keeps across reboots), the wording for the state label,
    and the second line of the error banner. They are appended, so a decoder that reads
    only the first four fields still reads them correctly.

    Real quantities, not display units: speed is metres per second and the clock is Unix
    time, both device-neutral. A joystick showing mph at one decimal converts at its own
    edge, the way it already does for current_seat_state.

    The clock is UTC and carries its offset separately, so epoch_s is an ordinary Unix
    timestamp that nothing has to second-guess; a screen showing the wall clock adds
    utc_offset_min itself. */
struct MibStatus {
  MibSystemState system_state{MibSystemState::INITIALIZING}; /**< current system state */
  DriveProfile active_profile{DriveProfile::NORMAL};         /**< profile in force */
  SeatState current_seat_state{};                            /**< where the seat is */
  std::string error_message{"No error"}; /**< human-readable; "No error" when none */
  int64_t epoch_s{0};         /**< Unix time, seconds since 1970-01-01 UTC; 0 = clock not known */
  float speed{0.0f};          /**< ground speed, m/s */
  int16_t utc_offset_min{0};  /**< minutes east of UTC, to turn epoch_s into local wall time */
  uint8_t seq{0};             /**< +1 per message, wraps; a gap is a dropped sample */
  std::string status_text{};  /**< optional wording for the state label; "" = the enum's name */
  std::string error_footer{}; /**< what to do about error_message, shown under it */
};

/* -------------------------------------------------------------------------
 * Drive profile limits. The joystick (or a bench tool) changes them with
 * rammp::DriveProfileCommand in messages/joystick_message.hpp; the MIB reports
 * every profile's limits in force in DriveProfileState.
 * ---------------------------------------------------------------------- */

/** Topic the MIB publishes its drive profile limits on. */
#define RAMMP_TOPIC_MIB_DRIVE_PROFILES "rammp/mib/drive_profiles"

/** Message type identifier associated with the drive profiles topic. */
#define RAMMP_TYPE_DRIVE_PROFILE_STATE "rammp/msg/DriveProfileState"

/**
 * @brief The limits of one drive profile, as the MIB's trajectory planner uses them.
 *
 * Field for field the MIB's DriveProfileConfig (MIBconfig.hpp in RAMMP-mib-fw).
 * The driving limits apply while the stick is off centre, the stopping limits once
 * it is back at exactly zero. 40 bytes on the wire.
 */
struct DriveProfileValues {
    float max_linear_velocity_mps;              /**< Forward speed limit, m/s; > 0. */
    float max_reverse_linear_velocity_mps;      /**< Reverse speed, m/s; 0 = same as forward. */
    float max_angular_velocity_radps;           /**< Turn rate limit, rad/s; > 0. */
    float max_linear_acceleration_mps2;         /**< Driving acceleration, m/s^2; > 0. */
    float max_angular_acceleration_radps2;      /**< Driving angular accel, rad/s^2; > 0. */
    float max_linear_jerk_mps3;                 /**< Driving jerk, m/s^3; 0 = trapezoidal. */
    float max_angular_jerk_radps3;              /**< Driving angular jerk; 0 = trapezoidal. */
    float stopping_linear_acceleration_mps2;    /**< Stopping deceleration, m/s^2; > 0. */
    float stopping_angular_acceleration_radps2; /**< Stopping angular decel, rad/s^2; > 0. */
    float max_centripetal_acceleration_mps2;    /**< Limit on |v * w|, m/s^2; 0 = no limit. */
};

/**
 * @brief Where the limits a profile is using came from.
 */
enum class DriveProfileSource : uint8_t {
    DEFAULT = 0, /**< Compiled-in: nothing saved, or the saved copy could not be used. */
    SAVED = 1,   /**< Loaded from the MIB's flash, or saved there by a command. */
    UNSAVED = 2, /**< Changed by a command without persist; a reboot reverts it. */
};

/**
 * @brief How the MIB handled the last rammp::DriveProfileCommand.
 */
enum class DriveProfileResult : uint8_t {
    NONE = 0,        /**< No command acted on since boot. */
    OK = 1,          /**< Applied, and saved or erased if persist was set. */
    INVALID = 2,     /**< A value was out of range or not a number; nothing changed. */
    SAVE_FAILED = 3, /**< Applied, but the flash write failed, so it lasts until reboot. */
    BAD_REQUEST = 4, /**< Unknown profile or action; nothing changed. */
};

/**
 * @brief Every drive profile's limits in force, and the outcome of the last command.
 *
 * Published with MibStatus (twice a second). A sender of a DriveProfileCommand
 * repeats it until command_ack shows its req_id, then reads result. Byte fields
 * first: XCDR1 aligns each field to its size, so this order needs no padding.
 * 128 bytes on the wire.
 */
struct DriveProfileState {
    uint8_t seq;               /**< +1 per message, wraps. */
    uint8_t command_ack;       /**< Last DriveProfileCommand req_id acted on; 0 = none. */
    DriveProfileResult result; /**< How that command went. */
    uint8_t reserved0;         /**< Sent as 0. */
    std::array<DriveProfileSource, kDriveProfileCount> sources; /**< Indexed by DriveProfile. */
    uint8_t reserved1;         /**< Sent as 0. */
    std::array<DriveProfileValues, kDriveProfileCount> profiles; /**< Indexed by DriveProfile. */
};

/** The MIB's drive profiles topic, typed. */
inline constexpr rammp::Topic<DriveProfileState> kMibDriveProfiles{RAMMP_TOPIC_MIB_DRIVE_PROFILES,
                                                                   RAMMP_TYPE_DRIVE_PROFILE_STATE};
} // namespace rammp

#endif /* RAMMP_RTPS_MIB_MESSAGE_HPP */
