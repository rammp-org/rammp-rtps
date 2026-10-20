/**
 * @file estop_message.hpp
 * @brief RAMMP emergency stop: one broadcast topic, robot-wide. Any node may publish;
 * engaging is fail-safe by design, and only kEStopClearer can end the stop.
 *
 * - C++20. The struct IS the wire layout: XCDR1, fields in declaration order.
 * - The first four fields are the ROS EStop message (std_msgs/Header, engaged, source,
 *   reason). pub_seq, clear_target and clear_pub_seq are appended, so a decoder that
 *   knows only the ROS layout still reads the start correctly. Being ROS 2 compatible is
 *   not a goal: the topic and type names follow the rammp convention.
 * - The stamp is for the log only. Staleness is judged by receive time on the
 *   receiver's steady clock, never by the stamp, which depends on clock sync.
 *
 * Terms: a "clear attempt" is the clearer's request to end the stop; the stop is
 * "released" only when the attempt succeeds. Between the two the stop stays in effect.
 *
 * What an e-stop message means:
 *
 *   engaged  clear_target  meaning                                 who sends it
 *   true     (any)         engage; always acted on, never age-    any node
 *                          or pub_seq-checked
 *   false    ""            heartbeat: "alive, not engaging";      any node, every period
 *                          it never ends the stop
 *   false    "<src>"       clear attempt for <src>'s engagement   kEStopClearer only
 *
 * A plain engaged=false is not a clear attempt (the ROS comment says false "clears it"):
 * otherwise any node's normal heartbeat would end another node's stop. A decoder that
 * knows only the ROS layout reads pub_seq = 0 and clear_target = "", so its messages are
 * heartbeats or engages, never clear attempts.
 *
 * Clear attempt (the full rules and races: pace-hmi-fw docs/plans/estop.md):
 *   - Every node sends at least every kEStopPolicy.period: engaged=true while it wants
 *     the stop, engaged=false otherwise. A node that engaged keeps sending engaged=true
 *     until it accepts a clear attempt.
 *   - The clearer sets clear_pub_seq to the newest pub_seq it has received from the
 *     target and resends the clear attempt every period, never changing clear_pub_seq
 *     during the attempt.
 *   - The target accepts it only if clear_pub_seq >= its engaged_pub_seq (the pub_seq of
 *     its first engaged=true after its latest press), then sends engaged=false.
 *   - The clearer releases the stop once the target's newest message is engaged=false
 *     with pub_seq > clear_pub_seq, received within kEStopPolicy.timeout. If that does
 *     not happen within kEStopPolicy.clear_window, the attempt expires and the stop holds.
 *   - Deadman: a source silent longer than kEStopPolicy.timeout counts as engaged.
 *
 * Example (espp, `rtps` a started espp::RtpsParticipant): a node that watches the stop
 *
 *   espp::Subscriber<rammp::EStop> estop_sub(
 *       rtps, {.topic = rammp::kEStop.name, .type_name = rammp::kEStop.type,
 *              .on_message = [](const rammp::EStop &m) { estop_on_message(m, now()); }});
 */

#ifndef RAMMP_RTPS_ESTOP_MESSAGE_HPP
#define RAMMP_RTPS_ESTOP_MESSAGE_HPP

#include <chrono>
#include <cstdint>
#include <string>

#include "messages/topic.hpp"

/* Broadcast: no publisher segment, any node publishes. */
#define RAMMP_TOPIC_ESTOP "rammp/estop"
#define RAMMP_TYPE_ESTOP "rammp/msg/EStop"

namespace rammp {

/** builtin_interfaces/Time */
struct Time {
  int32_t sec;      /**< seconds since 1970-01-01 UTC */
  uint32_t nanosec; /**< 0..999'999'999 */
};

/** std_msgs/Header */
struct Header {
  Time stamp;           /**< for the log only; all-zero = unstamped, and accepted */
  std::string frame_id; /**< unused by the e-stop, sent empty */
};

/** Broadcast emergency stop. See the table at the top for what each combination means. */
struct EStop {
  Header header;            /**< stamp: for the log only, never for decisions */
  bool engaged;             /**< true = engage; false = heartbeat, or a clear attempt */
  std::string source;       /**< who published it (node name), for the log and the attempt */
  std::string reason;       /**< free text, for the log */
  uint32_t pub_seq;         /**< this publisher's message number: +1 per message, wraps */
  std::string clear_target; /**< clear attempts only: whose engagement; "" otherwise */
  uint32_t clear_pub_seq;   /**< clear attempts only: newest pub_seq the clearer had received
                                 from clear_target when the attempt started */
};

inline constexpr Topic<EStop> kEStop{RAMMP_TOPIC_ESTOP, RAMMP_TYPE_ESTOP};

/** The only source whose clear attempts count: the MIB owns the latch. */
inline constexpr char kEStopClearer[] = "mib";

/** Timing every e-stop node shares. Configurable here, in code: the MIB, the joystick
    and the bench sim all read these values. */
struct EStopPolicy {
  std::chrono::milliseconds period;       /**< every node sends at least this often */
  std::chrono::milliseconds timeout;      /**< a source silent this long counts as engaged
                                               (if deadman); also how fresh the
                                               releasing engaged=false must be */
  bool deadman;                           /**< false: only engaged=true stops; silence is
                                               just logged */
  std::chrono::milliseconds clear_window; /**< a clear attempt not completed within
                                               this expires; the stop holds */
};

inline constexpr EStopPolicy kEStopPolicy{
    .period = std::chrono::milliseconds{100},
    .timeout = std::chrono::milliseconds{300},
    .deadman = true,
    .clear_window = std::chrono::milliseconds{1000},
};
static_assert(kEStopPolicy.timeout >= 2 * kEStopPolicy.period,
              "one lost message must not trip the deadman");
static_assert(kEStopPolicy.clear_window > kEStopPolicy.timeout,
              "a clear attempt must outlast a round trip and a lost message");

/** Wrap-safe: is pub_seq `a` newer than pub_seq `b`? (RFC 1982 serial-number arithmetic.) */
constexpr bool estop_seq_newer(uint32_t a, uint32_t b) {
  return a != b && static_cast<uint32_t>(a - b) < 0x8000'0000u;
}
/** Wrap-safe a >= b. */
constexpr bool estop_seq_at_least(uint32_t a, uint32_t b) {
  return a == b || estop_seq_newer(a, b);
}
static_assert(estop_seq_newer(1, 0) && !estop_seq_newer(0, 1) && !estop_seq_newer(5, 5));
static_assert(estop_seq_newer(0, 0xFFFF'FFFFu), "wraps");
static_assert(estop_seq_at_least(7, 7) && estop_seq_at_least(0, 0xFFFF'FFF0u));

} // namespace rammp

#endif /* RAMMP_RTPS_ESTOP_MESSAGE_HPP */
