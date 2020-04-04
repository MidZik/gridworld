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

        std::string get_state_json();

        void set_state_json(std::string json);

        uint64_t create_entity();

        void destroy_entity(uint64_t eid);

        std::vector<uint64_t> get_all_entities();

        void start_simulation();

        void stop_simulation();
    private:
        registry reg;

        enum class SimulationState : uint32_t
        {
            running, // no requested_state
            stopped, // thread halts as soon as it is able to
            waiting // thread waits as soon as it is able to
        };
        SimulationState requested_state;

        std::mutex simulation_mutex;
        std::condition_variable simulation_waiter;
        std::thread simulation_thread;

        class WaitGuard;


        void simulation_loop();
    };
}
