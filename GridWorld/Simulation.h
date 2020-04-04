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

        enum class ThreadNotification : uint32_t 
        {
            none, // no notification
            stop, // thread halts as soon as it is able to
            wait // thread waits as soon as it is able to
        };
        ThreadNotification notification;

        std::mutex simulation_mutex;
        std::condition_variable simulation_waiter;
        std::thread simulation_thread;


        void simulation_loop();
    };
}
