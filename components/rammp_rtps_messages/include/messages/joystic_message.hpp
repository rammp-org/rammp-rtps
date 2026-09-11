/**
 * @file joystic_message.hpp
 * @brief RAMMP RTPS topic, type, and message payload definitions.
 *
 * This header defines the plain-C topic/type names and payload structs shared
 * by RAMMP RTPS publishers and subscribers. Serialization/deserialization is
 * handled by espp/rtps, so this file intentionally contains no encoder or
 * decoder code.
 */

#ifndef RAMMP_RTPS_JOYSTIC_MESSAGE_HPP
#define RAMMP_RTPS_JOYSTIC_MESSAGE_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * McbStatus: MCB -> joystick status, published periodically.
 * ---------------------------------------------------------------------- */

#define RAMMP_TOPIC_MCB_STATUS "rammp/mcb/status"
#define RAMMP_TYPE_MCB_STATUS "rammp/msg/McbStatus"

/** Whether the chair is currently accepting drive commands from the stick. */
enum {
  RAMMP_DRIVE_STATUS_INACTIVE = 0,
  RAMMP_DRIVE_STATUS_ACTIVE = 1,
};

/** Overall health as judged by the MCB. */
enum {
  RAMMP_STATE_OK = 0,
  RAMMP_STATE_ERROR = 1,
};

/* -------------------------------------------------------------------------
 * RTPS message payloads
 * ---------------------------------------------------------------------- */

/**
 * Length of a label override, including the NUL terminator: 15 usable
 * characters. Printable ASCII only.
 */
#define RAMMP_MCB_TEXT_LEN 16

/**
 * Error banner body and footer, including the NUL. Sized for a couple of short
 * lines rather than a paragraph. Printable ASCII only.
 */
#define RAMMP_ERROR_TEXT_LEN 64
#define RAMMP_ERROR_FOOTER_LEN 32

typedef struct rammp_mcb_status {
  uint8_t drive_status;                /**< one of RAMMP_DRIVE_STATUS_* */
  uint8_t system_state;                /**< one of RAMMP_STATE_* */
  uint8_t flags;                       /**< reserved: drive inhibit, e-stop, ... (send 0) */
  uint8_t seq;                         /**< free-running, wraps; for staleness and debug */
  uint8_t speed_tenths;                /**< 0..99, shown as N.N */
  char drive_text[RAMMP_MCB_TEXT_LEN]; /**< "" = use the enum's name */
  char state_text[RAMMP_MCB_TEXT_LEN]; /**< "" = use the enum's name */
  char error_text[RAMMP_ERROR_TEXT_LEN];
  char error_footer[RAMMP_ERROR_FOOTER_LEN];
} rammp_mcb_status_t;

/* -------------------------------------------------------------------------
 * AdcXYTwist: joystick -> MCB stick position, button state, and drive mode.
 * ---------------------------------------------------------------------- */

#define RAMMP_TOPIC_JOYSTICK_ADC "rammp/joystick/adc"
#define RAMMP_TYPE_ADC_XY_TWIST "rammp/msg/AdcXYTwist"

/** How the chair should interpret stick deflection. */
enum {
  RAMMP_DRIVE_MODE_NORMAL = 0,
  RAMMP_DRIVE_MODE_HOLO = 1,
  RAMMP_DRIVE_MODE_AUTO = 2,
};

typedef struct rammp_adc_xy_twist {
  uint32_t x_mv;
  uint32_t y_mv;
  uint32_t twist_mv;
  uint32_t buttons;    /**< bitfield; bit set = pressed */
  uint32_t drive_mode; /**< one of RAMMP_DRIVE_MODE_* */
} rammp_adc_xy_twist_t;

/* -------------------------------------------------------------------------
 * UInt32 bench/bring-up topics use the stock ROS 2 UInt32 message.
 * ---------------------------------------------------------------------- */

#define RAMMP_TOPIC_HMI_COUNTER "rammp/hmi/counter"
#define RAMMP_TOPIC_HMI_COMMAND "rammp/hmi/command"
#define RAMMP_TOPIC_HMI_BRIGHTNESS "rammp/hmi/brightness"

#define RAMMP_TYPE_UINT32 "std_msgs/msg/UInt32"

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RAMMP_RTPS_JOYSTIC_MESSAGE_HPP */