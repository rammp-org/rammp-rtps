/**
 * @file mib_message.hpp
 * @brief RAMMP MIB RTPS messages: topic, type, and status payload definitions.
 *
 * This header contains the message definitions shared by MIB RTPS publishers
 * and subscribers so both sides use the same topic names, type names, and
 * payload fields.
 *
 * Contents: MIB drive profiles, system states, seat state, status message, and
 * the drive profile limits (DriveProfileValues, DriveProfileState).
 *
 * Wire rules, the same as joystick_message.hpp: each struct IS the wire layout
 * (espp/cdr serializes it as XCDR1, fields in declaration order), and every enum
 * is scoped with a fixed width so its size on the wire is not the compiler's
 * choice. MibStatus and DriveProfileState are resent periodically, best-effort
 * and without durability, so a joystick that reboots catches up on the next tick.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "messages/topic.hpp"
namespace MIB{


/**
 * @brief Drive response profile used by the MIB.
 */
enum class DriveProfile : uint8_t {
    LOW,    /**< Reduced drive response. */
    NORMAL, /**< Standard drive response. */
    HIGH    /**< Increased drive response. */
};

/** Number of DriveProfile values; DriveProfileState holds one entry per profile. */
inline constexpr std::size_t kDriveProfileCount = 3;

/**
 * @brief High-level operating state of the MIB system.
 *
 * The joystick picks its page from this value, so every state the joystick must
 * draw differently is its own value here; status_text only words the label.
 * New values are appended so the ones already on the wire keep their numbers.
 * A decoder that meets a value it does not know should fall back to a neutral
 * page rather than reject the message.
 */
enum class MibSystemState : uint8_t {
    INITIALIZING, /**< The system is initializing and is not ready. */
    IDLE,         /**< The system is idle and waiting for enable; seat control is available, but drive is disabled. */
    ENABLED,      /**< The system is enabled and operational; manual seat control is disabled. */
    ERROR,        /**< The system has encountered an error. */
    AUTONOMOUS,   /**< Another node is driving; the joystick is not read for driving, and touching it hands control back (the MIB returns to IDLE). */
    SHARED,       /**< Another node and the joystick drive together; the joystick stick is read and blended. */
    ESTOP         /**< Emergency stop: drive is cut and held; the MIB returns to IDLE on its own when the hold ends. */
};

/**
 * @brief Current position and orientation of the MIB seat.
 */
struct seatState{
    float front_back_tilt{0.0f}; /**< Front-to-back seat tilt. */
    float lateral_tilt{0.0f};    /**< Side-to-side seat tilt. */
    float elevation{0.0f};       /**< Seat elevation. */
    float translation{0.0f};      /**< Seat translation. */
};

/** Topic used to publish the MIB status message. */
#define RAMMP_TOPIC_MIB_STATUS "rammp/mib/status"

/** Message type identifier associated with the MIB status topic. */
#define RAMMP_TYPE_MIB_STATUS "rammp/msg/MibStatus"

/**
 * @brief Status message published by the MIB.
 *
 * The fields after error_message are what the joystick draws: the speed readout,
 * the TopBar clock (and the RTC it keeps across reboots), the wording for the
 * state label, and the second line of the error banner. They are appended, so a
 * decoder that reads only the first four fields still reads them correctly.
 *
 * Real quantities, not display units: speed is metres per second and the clock is
 * Unix time, both device-neutral. A joystick showing mph at one decimal converts
 * at its own edge, the way it already does for currentSeatState.
 *
 * The clock is UTC and carries its offset separately, so epoch_s is an ordinary
 * Unix timestamp that nothing has to second-guess; a screen showing the wall
 * clock adds utc_offset_min itself.
 */
struct MibStatus {
    MibSystemState systemState{MibSystemState::INITIALIZING}; /**< Current system state. */
    DriveProfile activeProfile{DriveProfile::NORMAL};         /**< Currently selected drive profile. */
    seatState currentSeatState{};                             /**< Current seat position and orientation. */
    std::string error_message{"No error"};                    /**< Human-readable error description. */
    int64_t epoch_s{0};        /**< Unix time, seconds since 1970-01-01 UTC; 0 = clock not known. */
    float speed{0.0f};         /**< Ground speed, metres per second. */
    int16_t utc_offset_min{0}; /**< Minutes east of UTC, to turn epoch_s into local wall time. */
    uint8_t seq{0};            /**< +1 per message, wraps; a gap is a dropped sample. */
    std::string status_text{}; /**< Optional wording for the state label; "" = use the enum's name. */
    std::string error_footer{}; /**< What to do about error_message, shown under it. */
};

/** The MIB's status topic, typed. */
inline constexpr rammp::Topic<MibStatus> kMibStatus{RAMMP_TOPIC_MIB_STATUS, RAMMP_TYPE_MIB_STATUS};

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
} // namespace MIB
