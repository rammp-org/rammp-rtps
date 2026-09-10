#pragma once

#include <stdint.h>

/** Bench/bring-up topics: heartbeat counter, its echo, remote LCD brightness. */
#define RAMMP_TOPIC_HMI_COUNTER "rammp/hmi/counter"
#define RAMMP_TOPIC_HMI_COMMAND "rammp/hmi/command"
#define RAMMP_TOPIC_HMI_BRIGHTNESS "rammp/hmi/brightness"
#define RAMMP_TOPIC_MCB_STATUS "rammp/mcb/status"

/** Stock ROS 2 type for the single-uint32 bench topics above. */
#define RAMMP_TYPE_UINT32 "std_msgs/msg/UInt32"

/* -------------------------------------------------------------------------
 * Timing contract for RAMMP_TOPIC_MCB_STATUS
 *
 * A publisher of rammp_mcb_status_t must republish at least every
 * RAMMP_MCB_STATUS_TIMEOUT_MS, even when nothing has changed.
 * ---------------------------------------------------------------------- */
#define RAMMP_MCB_STATUS_PERIOD_MS 500
#define RAMMP_MCB_STATUS_TIMEOUT_MS 2000

/* -------------------------------------------------------------------------
 * Enumerations
 *
 * Values are explicit because the python scraper reads them.
 * ---------------------------------------------------------------------- */

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
 * Messages
 * ---------------------------------------------------------------------- */

/** Length including NUL terminator: 15 usable characters. */
#define RAMMP_MCB_TEXT_LEN 16

/** Error banner sizes including NUL terminator. */
#define RAMMP_ERROR_TEXT_LEN 64
#define RAMMP_ERROR_FOOTER_LEN 32

/** speed_tenths runs 0..99 and is displayed as N.N, so 0.0 to 9.9. */
#define RAMMP_SPEED_MAX_TENTHS 99

/** MCB -> joystick status, published periodically. */
typedef struct rammp_mcb_status {
  uint8_t drive_status;                /**< one of RAMMP_DRIVE_STATUS_* */
  uint8_t system_state;                /**< one of RAMMP_STATE_* */
  uint8_t flags;                       /**< reserved: send 0 for now */
  uint8_t seq;                         /**< free-running, wraps */
  uint8_t speed_tenths;                /**< 0..RAMMP_SPEED_MAX_TENTHS */
  char drive_text[RAMMP_MCB_TEXT_LEN]; /**< "" = use enum name */
  char state_text[RAMMP_MCB_TEXT_LEN]; /**< "" = use enum name */
  char error_text[RAMMP_ERROR_TEXT_LEN];
  char error_footer[RAMMP_ERROR_FOOTER_LEN];
} rammp_mcb_status_t;
