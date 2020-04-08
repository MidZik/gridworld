#pragma once

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>

#include "Registry.h"

namespace GridWorld
{
    class Simulation
    {
    public:
        Simulation();

        std::string get_state_json() const;

        void set_state_json(std::string json);

        uint64_t create_entity();

        void destroy_entity(uint64_t eid);

        std::vector<uint64_t> get_all_entities() const;

        void start_simulation();

        void stop_simulation();

        void assign_component(uint64_t eid, std::string component_name);

        std::vector<std::string> get_component_names() const;
    private:
        registry reg;

        enum class SimulationState : uint32_t
        {
            running,
            stopped, // thread halts as soon as it is able to
            waiting // thread waits as soon as it is able to
        };
        mutable SimulationState requested_state;

        mutable std::mutex simulation_mutex;
        mutable std::condition_variable simulation_waiter;
        std::thread simulation_thread;

        class WaitGuard;


        void simulation_loop();
    };
}
