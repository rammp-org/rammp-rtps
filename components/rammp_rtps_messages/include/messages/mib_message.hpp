/**
 * @file mib_message.hpp
 * @brief RAMMP MIB RTPS messages: topic, type, and status payload definitions.
 *
 * This header contains the message definitions shared by MIB RTPS publishers
 * and subscribers so both sides use the same topic names, type names, and
 * payload fields.
 *
 * Contents: MIB drive profiles, system states, seat state, and status message.
 *
 * Wire rules, the same as joystick_message.hpp: each struct IS the wire layout
 * (espp/cdr serializes it as XCDR1, fields in declaration order), and every enum
 * is scoped with a fixed width so its size on the wire is not the compiler's
 * choice. MibStatus is resent periodically, best-effort and without durability,
 * so a joystick that reboots catches up on the next tick.
 */

#pragma once

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

/**
 * @brief High-level operating state of the MIB system.
 */
enum class MibSystemState : uint8_t {
    INITIALIZING, /**< The system is initializing and is not ready. */
    IDLE,         /**< The system is idle and waiting for enable; seat control is available, but drive is disabled. */
    ENABLED,      /**< The system is enabled and operational; manual seat control is disabled. */
    ERROR         /**< The system has encountered an error. */
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
} // namespace MIB
