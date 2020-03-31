#pragma once

#include <stdint.h>
#include <entt/entt.hpp>

namespace GridWorld
{
    ENTT_OPAQUE_TYPE(EntityId, uint64_t);

    using registry = entt::basic_registry<EntityId>;
}
