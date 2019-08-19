#pragma once

#include <entt/entt.hpp>

#define DllExport __declspec(dllexport)
#define CDllExport extern "C" __declspec(dllexport)

using EntityId = uint64_t;
using registry = entt::basic_registry<EntityId>;

namespace GridWorld
{
    using namespace std;

    class EntityManager
    {
    private:
        EntityId singleton_id = -1;
    public:
        registry reg;
        uint64_t tick = 0;

        EntityManager()
        {
            singleton_id = reg.create();
        }

        template <typename C>
        auto view()
        {
            return reg.view<C>();
        }

        template <typename... S>
        auto get_singletons()
        {
            return reg.try_get<S...>(singleton_id);
        }

        template<typename... S>
        bool has_singletons()
        {
            return reg.has<S...>(singleton_id);
        }

        template <typename S>
        decltype(auto) assign_or_replace_singleton()
        {
            return reg.assign_or_replace<S>(singleton_id);
        }

        template<typename S>
        void remove_singleton()
        {
            reg.remove<S>(singleton_id);
        }

        auto get_matching_entities(vector<string> types)
        {
            vector<entt::hashed_string::hash_type> type_ids;
            for (string s : types)
            {
                type_ids.push_back(entt::hashed_string::to_value(s.c_str()));
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

    DllExport EntityManager& create_test_em();

    DllExport void run_test(EntityManager& em);
}