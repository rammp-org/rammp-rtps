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
 * Wire rules, the same as joystick_message.hpp: C++20, each struct IS the wire
 * layout (espp/cdr serializes it as XCDR1, fields in declaration order), and every
 * enum is scoped with a fixed width so its size on the wire is not the compiler's
 * choice. MibStatus is the MIB's whole state: it is resent periodically, best-effort
 * and without durability, so a joystick that reboots catches up on the next tick.
 */

#ifndef RAMMP_RTPS_MIB_MESSAGE_HPP
#define RAMMP_RTPS_MIB_MESSAGE_HPP

#include <cstdint>
#include <string>

#include "messages/topic.hpp"

/** Topic used to publish the MIB status message. */
#define RAMMP_TOPIC_MIB_STATUS "rammp/mib/status"

/** Message type identifier associated with the MIB status topic. */
#define RAMMP_TYPE_MIB_STATUS "rammp/msg/MibStatus"

namespace MIB {

/**
 * @brief Drive response profile used by the MIB.
 *
 * Chosen on the joystick (DriveCommand) and confirmed here in MibStatus: the
 * joystick shows the profile it asked for only once this says the MIB took it.
 */
enum class DriveProfile : uint8_t {
  LOW = 0,    /**< Reduced drive response. */
  NORMAL = 1, /**< Standard drive response. */
  HIGH = 2    /**< Increased drive response. */
};

/**
 * @brief High-level operating state of the MIB system.
 *
 * This is the interlock the joystick obeys: it offers driving only in ENABLED and
 * manual seat adjustment only in IDLE, and shows the error banner in ERROR.
 */
enum class MibSystemState : uint8_t {
  INITIALIZING = 0, /**< The system is initializing and is not ready. */
  IDLE = 1,         /**< Idle, waiting for enable; seat control available, drive disabled. */
  ENABLED = 2,      /**< Enabled and operational; manual seat control is disabled. */
  ERROR = 3         /**< The system has encountered an error. */
};

/**
 * @brief Current position and orientation of the MIB seat.
 *
 * Real units, not raw counts: degrees for the tilts, millimetres for elevation and
 * translation. The MIB owns these - the joystick only ever asks (SeatCommand) and
 * draws what comes back here.
 */
struct seatState {
  float front_back_tilt{0.0f}; /**< Front-to-back seat tilt, degrees. */
  float lateral_tilt{0.0f};    /**< Side-to-side seat tilt, degrees. */
  float elevation{0.0f};       /**< Seat elevation, millimetres. */
  float translation{0.0f};     /**< Seat translation, millimetres. */
};

/** speed_tenths is 0..99, i.e. 0.0 .. 9.9 in the displayed unit. */
inline constexpr uint8_t kSpeedMaxTenths = 99;

/**
 * @brief Status message published by the MIB.
 *
 * Everything the joystick draws comes from here, so the fields after the seat state
 * exist for its screens: the speed readout, the TopBar clock (and the RTC it keeps
 * across reboots), and the two lines of the error banner.
 */
struct MibStatus {
  MibSystemState systemState{MibSystemState::INITIALIZING}; /**< Current system state. */
  DriveProfile activeProfile{DriveProfile::NORMAL};         /**< Currently selected drive profile. */
  seatState currentSeatState{};                             /**< Current seat position and orientation. */
  uint8_t seq{0};                                           /**< +1 per message, wraps. */
  uint8_t speed_tenths{0};                                  /**< 0..kSpeedMaxTenths. */
  uint8_t hour{0}, minute{0}, second{0};                    /**< MIB local time. */
  uint8_t day{0}, month{0}, year{0};                        /**< month 0 = time unknown; year since 2000. */
  std::string status_text{};              /**< Optional wording for the state label; "" = use the enum's name. */
  std::string error_message{"No error"};  /**< Human-readable error description, shown while ERROR. */
  std::string error_footer{};             /**< What to do about it, under error_message. */
};

/** The MIB's status topic, typed. */
inline constexpr rammp::Topic<MibStatus> kMibStatus{RAMMP_TOPIC_MIB_STATUS, RAMMP_TYPE_MIB_STATUS};

} // namespace MIB

#endif /* RAMMP_RTPS_MIB_MESSAGE_HPP */
