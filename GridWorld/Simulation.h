#pragma once

#include <string>
#include <vector>
#include <thread>
#include <shared_mutex>
#include <condition_variable>
#include <memory>
#include <functional>

#include "Registry.h"

namespace GridWorld
{
    class Simulation
    {
    public:
        using event_callback_function = void(const char*, const char*);
        using tick_event_callback_function = void(uint64_t, uint64_t);
        using command_result_callback_function = void(const char*, const char*);

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

        std::string get_singleton_json(std::string singleton_name) const;

        void set_singleton_json(std::string singleton_name, std::string singleton_json);

        std::vector<std::string> get_singleton_names() const;

        void set_tick_event_callback(tick_event_callback_function callback);

        std::vector<char> get_state_binary() const;

        void set_state_binary(const char* binary, size_t size);

        void get_events_last_tick(event_callback_function callback);

        void run_command(int64_t argc, const char* argv[], command_result_callback_function callback);

        void request_stop();
    private:
        registry reg;

        mutable std::mutex control_mutex;
        mutable bool stop_requested;

        mutable std::shared_mutex simulation_mutex;
        std::thread simulation_thread;

        tick_event_callback_function* tick_event_callback;


        void simulation_loop();
    };
}
