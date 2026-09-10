# rammp-rtps

Shared RTPS topic and type definitions for Rammp projects.

Any Rammp project that publishes/subscribes over RTPS should define topic names
and types in this repository, then consume this repository as a submodule.

## Repository layout

- [`main/rtps_topics.hpp`](./main/rtps_topics.hpp): central topic/type and
  message contract header.

## Add a new RTPS topic and type

1. Add topic/type macros (`RAMMP_TOPIC_*`, `RAMMP_TYPE_*`) in
   `main/rtps_topics.hpp`.
2. Add or extend message constants/enums used by all consumers.
3. Add/update shared `typedef struct` message contracts.

Keep topic names and type names stable once released, since they are consumed by
multiple projects.

## Use in another Rammp project

Add this repository as a submodule and include the topic registry header where
RTPS publishers/subscribers are configured.
