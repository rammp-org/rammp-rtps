# rammp-rtps

Shared RTPS topic and type definitions for Rammp projects.

Any Rammp project that publishes/subscribes over RTPS should define topic names
and types in this repository, then consume this repository as a submodule.

## Repository layout

- [`main/rtps_topics.hpp`](./main/rtps_topics.hpp): central topic/type registry.

## Add a new RTPS topic and type

1. Define the payload type in `main/rtps_topics.hpp`.
2. Add a `TopicTraits<T>` specialization for that type.
3. Add an entry to `kTopicDefinitions`.

Keep topic names and type names stable once released, since they are consumed by
multiple projects.

## Use in another Rammp project

Add this repository as a submodule and include the topic registry header where
RTPS publishers/subscribers are configured.
