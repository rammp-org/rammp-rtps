# rammp-rtps

Shared RTPS topic and type definitions for Rammp projects.

Any Rammp project that publishes/subscribes over RTPS should define topic names
and types in this repository, then consume this repository as a submodule.

## Repository layout

- [`components/rammp_rtps_messages`](./components/rammp_rtps_messages): shared
  RTPS message topics, type names, constants, enums, and payload structs.
- [`components/rammp_rtps_actions`](./components/rammp_rtps_actions): shared
  RTPS action definitions.
- [`components/rammp_rtps_services`](./components/rammp_rtps_services): shared
  RTPS service definitions.
- [`components/rammp_rtps_interface`](./components/rammp_rtps_interface):
  aggregate header-only component that includes messages, actions, and services.

## Add a new RTPS topic and type

1. Add the message `struct` in
  `components/rammp_rtps_messages/include/messages/joystick_message.hpp` (C++20; the
  struct is the XCDR1 wire layout, fields in order).
2. Give its enums a fixed wire width: `enum class X : uint8_t` (no plain enums).
3. Add its `Topic<Message>` constant (topic + type name) right after the struct, so
  a publisher or subscriber for the wrong message does not compile.
4. Add action definitions under `components/rammp_rtps_actions/include/`.
5. Add service definitions under `components/rammp_rtps_services/include/`.

Keep topic names and type names stable once released, since they are consumed by
multiple projects.

## Use in another Rammp project

Add this repository as a submodule and include the topic registry header where
RTPS publishers/subscribers are configured.

Preferred includes:

```cpp
#include "rtps_interface.hpp" // messages, actions, and services
#include "messages.hpp"       // messages only
#include "actions.hpp"        // actions only
#include "services.hpp"       // services only
```
