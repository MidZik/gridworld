#pragma once

#include <variant>
#include <string>
#include <map>
#include <vector>

namespace GridWorld::Events
{
    struct Event
    {
        class variant;
        using variant_map = std::map<std::string, variant>;
        using variant_vector = std::vector<variant>;
        class variant : public std::variant<std::monostate, int, double, std::string, variant_map, variant_vector>
        {
            using std::variant<std::monostate, int, double, std::string, variant_map, variant_vector>::variant;
        };

        std::string name;
        variant data;
    };
}
