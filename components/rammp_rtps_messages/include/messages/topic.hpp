/**
 * @file topic.hpp
 * @brief A DDS topic and type name, tied to the message it carries. Shared by every
 * message header so a publisher or subscriber for the wrong message does not compile.
 */

#ifndef RAMMP_RTPS_TOPIC_HPP
#define RAMMP_RTPS_TOPIC_HPP

namespace rammp {

/** A DDS topic and type name, tied to the message it carries. */
template <class Message> struct Topic {
  const char *name; /**< DDS topic name */
  const char *type; /**< DDS type name */
};

} // namespace rammp

#endif /* RAMMP_RTPS_TOPIC_HPP */
