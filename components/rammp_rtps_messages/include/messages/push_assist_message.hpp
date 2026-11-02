/**
 * @file push_assist_message.hpp
 * @brief RAMMP push-assist handles -> MIB: the force on each push handle.
 *
 * - C++20. The struct IS the wire layout: XCDR1, fields in declaration order.
 * - The handles measure; the MIB decides. The board tares and zero-tracks each load
 *   cell and sends the forces as they are. The MIB turns them into a forward and a
 *   turn target for its trajectory planner while it is in MibSystemState::PUSH_ASSIST
 *   (entered and left by DriveCommand.push_assist). How the MIB should do that, with
 *   the reference conversion: rammp-org/space-bar-push-assist, docs/MIB-INPUT-DEVICES.md.
 * - Best-effort, latest wins, sent at the load cells' rate (10 or 80 Hz) whether or not
 *   anyone drives on it. The MIB stops the drive when it goes quiet (300 ms), when a
 *   force is NaN, or when a flag says a handle is not usable.
 *
 * Example (espp, `rtps` a started espp::RtpsParticipant): the MIB taking the forces
 *
 *   espp::Subscriber<rammp::HandleForce> force_sub(
 *       rtps, {.topic = rammp::kPushAssistHandleForce.name,
 *              .type_name = rammp::kPushAssistHandleForce.type,
 *              .on_message = [](const rammp::HandleForce &f) { push_assist(f); }});
 */

#ifndef RAMMP_RTPS_PUSH_ASSIST_MESSAGE_HPP
#define RAMMP_RTPS_PUSH_ASSIST_MESSAGE_HPP

#include <cstdint>

#include "messages/topic.hpp"

/* push_assist -> MIB */
#define RAMMP_TOPIC_PUSH_ASSIST_HANDLE_FORCE "rammp/push_assist/handle_force"
#define RAMMP_TYPE_HANDLE_FORCE "rammp/msg/HandleForce"

namespace rammp {

struct HandleForce;

inline constexpr Topic<HandleForce> kPushAssistHandleForce{RAMMP_TOPIC_PUSH_ASSIST_HANDLE_FORCE,
                                                           RAMMP_TYPE_HANDLE_FORCE};

/** HandleForce.flags bits. A handle is usable for driving only while its OK bit is set
    and its ZERO_AT_LIMIT bit is clear. */
inline constexpr uint8_t kHandleForceLeftOk = 1u << 0;          /**< left amp answering, left_n valid */
inline constexpr uint8_t kHandleForceRightOk = 1u << 1;         /**< right amp answering, right_n valid */
inline constexpr uint8_t kHandleForceLeftZeroAtLimit = 1u << 2;  /**< left zero tracking at its cap: tare */
inline constexpr uint8_t kHandleForceRightZeroAtLimit = 1u << 3; /**< right zero tracking at its cap: tare */
inline constexpr uint8_t kHandleForceUpdating = 1u << 4;         /**< firmware update running: do not drive */

/** Force on each push handle, newtons, push +, pull -. Left and right are as seen by the
    attendant standing behind the chair. Turn sense: left harder than right turns the
    chair clockwise seen from above (as XYTwist.twist); the MIB converts to REP 103. */
struct HandleForce {
  float left_n;   /**< N, tared and zero-tracked; NaN while the left amp is not answering */
  float right_n;  /**< N, same for the right handle */
  uint8_t flags;  /**< kHandleForce* bits */
  uint32_t seq;   /**< +1 per sample since boot, so a receiver can count drops */
};

} // namespace rammp

#endif // RAMMP_RTPS_PUSH_ASSIST_MESSAGE_HPP
