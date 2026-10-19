/**
 * @file estop_message.hpp
 * @brief RAMMP emergency stop: one broadcast topic, robot-wide. Any node may publish;
 * engaging is fail-safe by design, and only kEStopClearer can clear.
 *
 * - C++20. The struct IS the wire layout: XCDR1, fields in declaration order.
 * - The first four fields are the ROS EStop message (std_msgs/Header, engaged, source,
 *   reason). seq, ack_source and ack_seq are appended, so a decoder that knows only the
 *   ROS layout still reads the start correctly. Being ROS 2 compatible is not a goal:
 *   the topic and type names follow the rammp convention.
 * - The stamp is for the log only. Staleness is judged by receive time on the
 *   receiver's steady clock, never by the stamp, which depends on clock sync.
 *
 * What a message means:
 *
 *   engaged  ack_source  meaning                                  who sends it
 *   true     (any)       engage; always acted on, never age-     any node
 *                        or seq-checked
 *   false    ""          heartbeat: "alive, not engaging";       any node, every period
 *                        it clears nothing
 *   false    "<src>"     clears <src>'s engagement               kEStopClearer only
 *
 * A plain false is not a clear (the ROS comment says it is): otherwise any node's
 * normal heartbeat would release another node's stop. A decoder that knows only the
 * ROS layout reads seq = 0 and ack_source = "", so its messages are heartbeats or
 * engages, never clears.
 *
 * Clear handshake (the full rules and races: pace-hmi-fw docs/plans/estop.md):
 *   - Every node sends at least every kEStopPolicy.period: true while it wants the
 *     stop, false otherwise. A node that engaged keeps sending true until it accepts
 *     a clear.
 *   - The clearer fixes ack_seq to the newest seq it has seen from the source and
 *     resends the clear every period, never updating ack_seq during the attempt.
 *   - The source accepts it only if ack_seq >= the seq of its first true after its
 *     latest press, then sends false.
 *   - The clearer releases once that source's newest message is a false with
 *     seq > ack_seq, received within kEStopPolicy.timeout. If that does not happen
 *     within kEStopPolicy.clear_window, the attempt is dropped and the stop holds.
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
  Header header;          /**< stamp: for the log only, never for decisions */
  bool engaged;           /**< true = engage; false = heartbeat, or a clear with ack_source */
  std::string source;     /**< who published it (node name), for the log and the handshake */
  std::string reason;     /**< free text, for the log */
  uint32_t seq;           /**< +1 per message from this source, wraps; order, loss, the clear */
  std::string ack_source; /**< clears only: the source whose engagement this clears; "" if not */
  uint32_t ack_seq;       /**< clears only: newest seq the clearer had seen from ack_source */
};

inline constexpr Topic<EStop> kEStop{RAMMP_TOPIC_ESTOP, RAMMP_TYPE_ESTOP};

/** The only source whose clears count: the MIB owns the latch. */
inline constexpr char kEStopClearer[] = "mib";

/** Timing every e-stop node shares. Configurable here, in code: the MIB, the joystick
    and the bench sim all read these values. */
struct EStopPolicy {
  std::chrono::milliseconds period;       /**< every node sends at least this often */
  std::chrono::milliseconds timeout;      /**< a source silent this long counts as engaged
                                               (if deadman); also how fresh a clear's
                                               matching false must be */
  bool deadman;                           /**< false: only engaged=true stops; silence is
                                               just logged */
  std::chrono::milliseconds clear_window; /**< a clear not taken up within this is dropped;
                                               the stop holds */
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
              "a clear must outlast at least one round trip and a lost message");

/** Wrap-safe: is seq `a` newer than seq `b`? (RFC 1982 serial-number arithmetic.) */
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
