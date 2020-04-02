#pragma once

#include <stdint.h>

namespace entt
{
    template<class>
    class basic_registry;
}

namespace GridWorld
{
    enum class EntityId : uint64_t { };
    constexpr auto to_integral(const EntityId eid) noexcept
    {
        return static_cast<std::underlying_type_t<EntityId>>(eid);
    }

    using registry = entt::basic_registry<EntityId>;
}
