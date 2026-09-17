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

/** speed_tenths is 0..99, i.e. 0.0 .. 9.9 in the displayed unit. */
inline constexpr uint8_t kSpeedMaxTenths = 99;

/**
 * @brief Status message published by the MIB.
 *
 * The fields after error_message are what the joystick draws: the speed readout,
 * the TopBar clock (and the RTC it keeps across reboots), the wording for the
 * state label, and the second line of the error banner. They are appended, so a
 * decoder that reads only the first four fields still reads them correctly.
 */
struct MibStatus {
    MibSystemState systemState{MibSystemState::INITIALIZING}; /**< Current system state. */
    DriveProfile activeProfile{DriveProfile::NORMAL};         /**< Currently selected drive profile. */
    seatState currentSeatState{};                             /**< Current seat position and orientation. */
    std::string error_message{"No error"};                    /**< Human-readable error description. */
    uint8_t seq{0};                        /**< +1 per message, wraps. */
    uint8_t speed_tenths{0};               /**< 0..kSpeedMaxTenths. */
    uint8_t hour{0}, minute{0}, second{0}; /**< MIB local time. */
    uint8_t day{0}, month{0}, year{0};     /**< month 0 = time unknown; year since 2000. */
    std::string status_text{};             /**< Optional wording for the state label; "" = use the enum's name. */
    std::string error_footer{};            /**< What to do about error_message, shown under it. */
};

/** The MIB's status topic, typed. */
inline constexpr rammp::Topic<MibStatus> kMibStatus{RAMMP_TOPIC_MIB_STATUS, RAMMP_TYPE_MIB_STATUS};
} // namespace MIB
