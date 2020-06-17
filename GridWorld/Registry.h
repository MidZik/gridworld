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

    inline std::string to_string(const EntityId eid) noexcept
    {
        return std::to_string(to_integral(eid));
    }

    using registry = entt::basic_registry<EntityId>;
}
