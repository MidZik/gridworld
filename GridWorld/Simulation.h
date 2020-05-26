#pragma once

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <functional>

#include "Registry.h"

namespace GridWorld
{
    class Simulation
    {
    public:
        using event_callback_function = std::function<void(std::string)>;

        Simulation();

        uint64_t get_tick() const;

        std::string get_state_json() const;

        void set_state_json(std::string json);

        uint64_t create_entity();

        void destroy_entity(uint64_t eid);

        std::vector<uint64_t> get_all_entities() const;

        void start_simulation();

        void stop_simulation();

        bool is_running() const;

        void assign_component(uint64_t eid, std::string component_name);

        std::string get_component_json(uint64_t eid, std::string component_name) const;

        void remove_component(uint64_t eid, std::string component_name);

        void replace_component(uint64_t eid, std::string component_name, std::string component_json);

        std::vector<std::string> get_component_names() const;

        std::vector<std::string> get_entity_component_names(uint64_t eid) const;

        void set_event_callback(event_callback_function callback);
    private:
        registry reg;

        enum class SimulationState : uint32_t
        {
            running,
            stopped, // thread halts as soon as it is able to
            waiting // thread waits as soon as it is able to
        };
        mutable SimulationState requested_state;

        mutable std::recursive_mutex simulation_mutex;
        mutable std::condition_variable_any simulation_waiter;
        std::thread simulation_thread;

        event_callback_function event_callback;

        class WaitGuard;


        void simulation_loop();
    };
}
