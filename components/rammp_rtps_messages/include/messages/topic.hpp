/**
 * @file topic.hpp
 * @brief Ties a DDS topic name to the message type it carries.
 *
 * Its own header because both message headers need it and they include each
 * other's contents in one direction only: topic.hpp <- mib_message.hpp <-
 * joystick_message.hpp.
 */

#ifndef RAMMP_RTPS_TOPIC_HPP
#define RAMMP_RTPS_TOPIC_HPP

namespace rammp {

/** A DDS topic and type name, tied to the message it carries. Passing a topic to a
    publisher or subscriber of the wrong message does not compile. */
template <class Message> struct Topic {
  const char *name; /**< DDS topic name */
  const char *type; /**< DDS type name */
};

} // namespace rammp

#endif /* RAMMP_RTPS_TOPIC_HPP */
