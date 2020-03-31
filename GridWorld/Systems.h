#pragma once

#include "Registry.h"

namespace GridWorld::Systems
{
    void movement(registry& reg);

    void simple_brain_calc(registry& reg);

    void simple_brain_seer(registry& reg);

    void simple_brain_mover(registry& reg);

    void random_movement(registry& reg);

    void predation(registry& reg);
}
