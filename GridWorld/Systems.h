#pragma once

#include "Registry.h"

namespace GridWorld::Systems
{
    namespace Util
    {
        void rebuild_world(registry& reg);
    }

    void tick_increment(registry& reg);

    void movement(registry& reg);

    void simple_brain_calc(registry& reg);

    void simple_brain_seer(registry& reg);

    void simple_brain_mover(registry& reg);

    void random_movement(registry& reg);

    void predation(registry& reg);

    void evolution(registry& reg);

    void finalize_event_log(registry& reg);
}
