/**
 * @file mib_message.hpp
 * @brief RAMMP MIB RTPS messages: topic, type, and status payload definitions.
 *
 * This header contains the message definitions shared by MIB RTPS publishers
 * and subscribers so both sides use the same topic names, type names, and
 * payload fields.
 *
 * Contents: MIB drive profiles, system states, seat state, and status message.
 */

#pragma once

#include <string>
namespace MIB{


/**
 * @brief Drive response profile used by the MIB.
 */
enum class DriveProfile {
    LOW,    /**< Reduced drive response. */
    NORMAL, /**< Standard drive response. */
    HIGH    /**< Increased drive response. */
};

/**
 * @brief High-level operating state of the MIB system.
 */
enum class MibSystemState {
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
 */
struct MibStatus {
    MibSystemState systemState{MibSystemState::INITIALIZING}; /**< Current system state. */
    DriveProfile activeProfile{DriveProfile::NORMAL};         /**< Currently selected drive profile. */
    seatState currentSeatState{};                             /**< Current seat position and orientation. */
    std::string error_message{"No error"};                    /**< Human-readable error description. */
};
} // namespace MIB
