#pragma once

#include <string>
#include <vector>

#include "Registry.h"

namespace GridWorld
{
    class Simulation
    {
    public:
        Simulation();

        std::string get_state_json();

        void set_state_json(std::string json);

        uint64_t create_entity();

        void destroy_entity(uint64_t eid);

        std::vector<uint64_t> get_all_entities();
    private:
        registry reg;
    };
}
