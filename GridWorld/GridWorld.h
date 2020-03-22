#pragma once

#include <entt/entt.hpp>

#define DllExport __declspec(dllexport)
#define CDllExport extern "C" __declspec(dllexport)

ENTT_OPAQUE_TYPE(EntityId, uint64_t);
using registry = entt::basic_registry<EntityId>;

namespace GridWorld
{
    using namespace std;

    class EntityManager
    {
    public:
        registry reg;
        uint64_t tick = 0;

        EntityManager()
        {
        }

        template <typename C>
        auto view()
        {
            return reg.view<C>();
        }

        template <typename S>
        S& get_singleton()
        {
            S* singleton = reg.try_ctx<S>();

            if (singleton == nullptr)
            {
                throw pybind11::value_error("Given singleton does not exist.");
            }
            else
            {
                return *singleton;
            }
        }

        template<typename S>
        bool has_singleton()
        {
            return reg.try_ctx<S>() == nullptr;
        }

        template <typename S>
        S& set_singleton()
        {
            return reg.set<S>();
        }

        template<typename S>
        void unset_singleton()
        {
            reg.unset<S>();
        }

        auto get_matching_entities(vector<string> types)
        {
            vector<entt::hashed_string::hash_type> type_ids;
            for (string s : types)
            {
                auto id = entt::resolve(entt::hashed_string(s.c_str())).id();
                type_ids.push_back(id);
            }
            auto view = reg.runtime_view(type_ids.begin(), type_ids.end());
            return vector(view.begin(), view.end());
        }

        template<typename... C>
        auto get_components(EntityId eid)
        {
            if (!reg.valid(eid))
            {
                throw pybind11::key_error("The entity doesn't exist.");
            }

            auto components = reg.try_get<C...>(eid);

            if (components == nullptr)
            {
                throw pybind11::value_error("The entity doesn't have this component type.");
            }
            else
            {
                return components;
            }
        }

        template<typename... C>
        bool has_components(EntityId eid)
        {
            if (!reg.valid(eid))
            {
                throw pybind11::key_error("The entity doesn't exist.");
            }

            return reg.has<C...>(eid);
        }

        template<typename C>
        decltype(auto) assign_or_replace(EntityId eid)
        {
            return reg.assign_or_replace<C>(eid);
        }

        template<typename C>
        void remove(EntityId eid)
        {
            reg.remove<C>(eid);
        }

        void destroy(EntityId eid)
        {
            reg.destroy(eid);
        }

        EntityId create()
        {
            return reg.create();
        }
    };
}