/**
 * @file joystic_message.hpp
 * @brief RAMMP joystick RTPS message definition.
 * Defines the topic, type, and payload struct for joystick messages.
 * Any subscribe to the joystick messages should use this header so the publisher and subscriber could match the message definition.
 * New message type could be added later.
 *
 */

#ifndef RAMMP_RTPS_JOYSTIC_MESSAGE_HPP
#define RAMMP_RTPS_JOYSTIC_MESSAGE_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * XYTwist: joystick -> MCB normalized stick position, button state, and mode.
 * ---------------------------------------------------------------------- */

#define RAMMP_TOPIC_JOYSTICK_XY_TWIST "rammp/joystick/xy_twist"
#define RAMMP_TYPE_XY_TWIST "rammp/msg/XYTwist"

typedef struct rammp_xy_twist {
  float x;             /**< normalized -1.0..1.0 */
  float y;             /**< normalized -1.0..1.0 */
  float twist;         /**< normalized -1.0..1.0 */
  uint32_t buttons;    /**< bitfield; bit set = pressed */
  uint32_t drive_mode; /**< drive mode selected by the HMI */
} rammp_xy_twist_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RAMMP_RTPS_JOYSTIC_MESSAGE_HPP */