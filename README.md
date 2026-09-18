# rammp-rtps

Shared RTPS topic and type definitions for Rammp projects.

Any Rammp project that publishes/subscribes over RTPS should define topic names
and types in this repository, then consume this repository as a submodule.

## Repository layout

- [`components/rammp_rtps_messages`](./components/rammp_rtps_messages): shared
  RTPS message topics, type names, constants, enums, and payload structs.
  - `messages/topic.hpp`: `Topic<Message>`, the typed topic/type handle every
    message header uses.
  - `messages/joystick_message.hpp`: joystick <-> MCB.
  - `messages/mib_message.hpp`: the MIB's state, which the joystick draws.
  - `messages/motor_message.hpp`: MIB <-> PACE RACER motor controllers, one
    topic pair per axis from `RAMMP_AXIS_TABLE`.
- [`components/rammp_rtps_actions`](./components/rammp_rtps_actions): shared
  RTPS action definitions.
- [`components/rammp_rtps_services`](./components/rammp_rtps_services): shared
  RTPS service definitions.
- [`components/rammp_rtps_interface`](./components/rammp_rtps_interface):
  aggregate header-only component that includes messages, actions, and services.

## Add a new RTPS topic and type

1. Add the message `struct` to the header for its publisher/subscriber pair under
  `components/rammp_rtps_messages/include/messages/` (C++20; the struct is the
  XCDR1 wire layout, fields in order), and include that header from `messages.hpp`.
2. Give its enums a fixed wire width: `enum class X : uint8_t` (no plain enums).
3. Name it with the pattern, in the "Topics and types" block at the top:
  `#define RAMMP_TOPIC_<PUBLISHER>_<MESSAGE> "rammp/<publisher>/<message>"`,
  `#define RAMMP_TYPE_<MESSAGE> "rammp/msg/<Message>"`, and the typed
  `inline constexpr Topic<Message> k<Publisher><Message>{...}` built from them, so a
  publisher or subscriber for the wrong message does not compile.
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
