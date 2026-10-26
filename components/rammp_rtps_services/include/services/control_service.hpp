/**
 * @file control_service.hpp
 * @brief Token arbitration services for the base: acquire, release, and revoke control.
 * The status topic and the tokened commands are messages/control_message.hpp.
 *
 * - C++20. Each struct IS the wire layout: XCDR1, fields in declaration order.
 * - ROS 2 services, on purpose: see control_message.hpp. Each Service<> carries the
 *   service name and the BASE DDS type; espp derives the request and reply topics
 *   (rq/base/acquire_controlRequest, rr/base/acquire_controlReply) and the
 *   _Request_ / _Response_ type names from them, as ROS 2 does.
 *
 * Mirrors, field for field, rammp_common_interfaces/srv/ in rammp-interfaces-ros2:
 *   AcquireControl  (dev: the request carries an optional pre-supplied token)
 *   ReleaseControl
 *   RevokeControl
 *
 * Example (espp, `rtps` a started espp::RtpsParticipant, built with
 * CONFIG_RTPS_ENABLE_RPC): the MIB answering acquire requests
 *
 *   espp::ServiceServer<rammp::AcquireControlRequest, rammp::AcquireControlResponse>
 *       acquire(rtps, {.service = rammp::kBaseAcquireControl.name,
 *                      .type_name = rammp::kBaseAcquireControl.type,
 *                      .handler = [](const rammp::AcquireControlRequest &request) {
 *                        return grant(request.owner_id, request.token);
 *                      }});
 */

#ifndef RAMMP_RTPS_CONTROL_SERVICE_HPP
#define RAMMP_RTPS_CONTROL_SERVICE_HPP

#include <cstdint>
#include <string>

#include "messages/control_message.hpp" /* Token */

/* -------------------------------------------------------------------------
 * Services and types: ROS 2 names. The type is the base name, without the suffixes.
 * ---------------------------------------------------------------------- */

#define RAMMP_ROS_SERVICE_BASE_ACQUIRE_CONTROL "/base/acquire_control"
#define RAMMP_ROS_TYPE_ACQUIRE_CONTROL "rammp_common_interfaces::srv::dds_::AcquireControl"
#define RAMMP_ROS_SERVICE_BASE_RELEASE_CONTROL "/base/release_control"
#define RAMMP_ROS_TYPE_RELEASE_CONTROL "rammp_common_interfaces::srv::dds_::ReleaseControl"
#define RAMMP_ROS_SERVICE_BASE_REVOKE_CONTROL "/base/revoke_control"
#define RAMMP_ROS_TYPE_REVOKE_CONTROL "rammp_common_interfaces::srv::dds_::RevokeControl"

namespace rammp {

/** A service name and base DDS type, tied to the request and response it carries. */
template <class Request, class Response> struct Service {
  const char *name; /**< ROS 2 service name, e.g. "/base/acquire_control" */
  const char *type; /**< base DDS type, without _Request_ / _Response_ */
};

/** Acquire control of the base. SEIZES: if someone else holds it, they are halted first.
    By operational contract only the orchestrator calls this. */
struct AcquireControlRequest {
  std::string owner_id; /**< human-readable label, reported in ControlStatus */
  Token token;          /**< all-zero: mint one. Otherwise adopt this key, so one key can
                             cover the arm and the base. A server that predates the field
                             ignores it, so check the response's token. */
};
struct AcquireControlResponse {
  bool accepted;
  Token token;         /**< put it on every command to the base */
  uint64_t generation; /**< ControlStatus.generation for this grant */
  std::string message; /**< why it was refused; "" on success */
};

/** Release control you hold. Refused unless the token matches the current owner's. */
struct ReleaseControlRequest {
  Token token;
};
struct ReleaseControlResponse {
  bool released;
  std::string message;
};

/** Take the base from whoever holds it. Operator recovery for a crashed owner, so it
    carries no token: ownership has no lease. */
struct RevokeControlRequest {
  std::string reason; /**< for the log */
};
struct RevokeControlResponse {
  bool revoked; /**< true if someone held the base */
  std::string message;
};

inline constexpr Service<AcquireControlRequest, AcquireControlResponse> kBaseAcquireControl{
    RAMMP_ROS_SERVICE_BASE_ACQUIRE_CONTROL, RAMMP_ROS_TYPE_ACQUIRE_CONTROL};
inline constexpr Service<ReleaseControlRequest, ReleaseControlResponse> kBaseReleaseControl{
    RAMMP_ROS_SERVICE_BASE_RELEASE_CONTROL, RAMMP_ROS_TYPE_RELEASE_CONTROL};
inline constexpr Service<RevokeControlRequest, RevokeControlResponse> kBaseRevokeControl{
    RAMMP_ROS_SERVICE_BASE_REVOKE_CONTROL, RAMMP_ROS_TYPE_REVOKE_CONTROL};

} // namespace rammp

#endif /* RAMMP_RTPS_CONTROL_SERVICE_HPP */
