#pragma once

#include <array>
#include <cstdint>
#include <string_view>

namespace rammp::rtps {

struct TopicDefinition {
  std::string_view topic_name;
  std::string_view type_name;
};

template <typename T> struct TopicTraits;

struct Heartbeat {
  uint32_t sequence = 0;
};

template <> struct TopicTraits<Heartbeat> {
  static constexpr std::string_view kTopicName = "rammp/heartbeat";
  static constexpr std::string_view kTypeName = "rammp.rtps.Heartbeat";
};

inline constexpr std::array<TopicDefinition, 1> kTopicDefinitions{{
    {.topic_name = TopicTraits<Heartbeat>::kTopicName,
     .type_name = TopicTraits<Heartbeat>::kTypeName},
}};

} // namespace rammp::rtps
