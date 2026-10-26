/**
 * @file header.hpp
 * @brief The ROS 2 stamp and header, as wire structs. Shared by every message that
 * carries a std_msgs/Header, so there is one definition of each.
 *
 * - C++20. Each struct IS the wire layout: XCDR1, fields in declaration order.
 */

#ifndef RAMMP_RTPS_HEADER_HPP
#define RAMMP_RTPS_HEADER_HPP

#include <cstdint>
#include <string>

namespace rammp {

/** builtin_interfaces/Time */
struct Time {
  int32_t sec;      /**< seconds since 1970-01-01 UTC */
  uint32_t nanosec; /**< 0..999'999'999 */
};

/** std_msgs/Header. What the stamp and frame mean is each message's own business. */
struct Header {
  Time stamp;           /**< all-zero = unstamped */
  std::string frame_id; /**< "" where the message has no frame */
};

} // namespace rammp

#endif /* RAMMP_RTPS_HEADER_HPP */
