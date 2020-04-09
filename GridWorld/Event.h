#pragma once

#include <variant>
#include <string>
#include <map>
#include <vector>

namespace GridWorld::Events
{
    struct EventData
    {
        using data_map = std::map<std::string, EventData>;
        using data_vector = std::vector<EventData>;
        using data_variant = std::variant<std::monostate, int, double, std::string, data_map, data_vector>;

        data_variant data;
    };

    struct Event
    {
        std::string name;
        EventData data;
    };
}
