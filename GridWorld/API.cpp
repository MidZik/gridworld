#include "stdafx.h"
#include "Simulation.h"

#define API_EXPORT extern "C" __declspec(dllexport)

using namespace GridWorld;

using cstr_result_callback = void(const char*);
using uint64_result_callback = void(uint64_t);

template<typename Vt, typename C>
void vector_to_callback(const std::vector<Vt>&& vector, C callback)
{
    for (const Vt& item : vector)
    {
        callback(item);
    }
}

template<typename C>
void vector_to_callback(const std::vector<std::string>&& vector, C callback)
{
    for (const std::string& item : vector)
    {
        callback(item.c_str());
    }
}

Simulation* sim(void* ptr)
{
    return static_cast<Simulation*>(ptr);
}

API_EXPORT void* create_simulation()
{
    return new Simulation();
}

API_EXPORT void destroy_simulation(void* ptr)
{
    delete sim(ptr);
}

API_EXPORT uint64_t get_tick(void* ptr)
{
    return sim(ptr)->get_tick();
}

API_EXPORT void get_state_json(void* ptr, cstr_result_callback callback)
{
    callback(sim(ptr)->get_state_json().c_str());
}

API_EXPORT void set_state_json(void* ptr, const char* json)
{
    sim(ptr)->set_state_json(json);
}

API_EXPORT uint64_t create_entity(void* ptr)
{
    return sim(ptr)->create_entity();
}

API_EXPORT void destroy_entity(void* ptr, uint64_t eid)
{
    return sim(ptr)->destroy_entity(eid);
}

API_EXPORT void get_all_entities(void* ptr, uint64_result_callback callback)
{
    vector_to_callback(sim(ptr)->get_all_entities(), callback);
}

API_EXPORT void start_simulation(void* ptr)
{
    return sim(ptr)->start_simulation();
}

API_EXPORT void stop_simulation(void* ptr)
{
    return sim(ptr)->stop_simulation();
}

API_EXPORT bool is_running(void* ptr)
{
    return sim(ptr)->is_running();
}

API_EXPORT void assign_component(void* ptr, uint64_t eid, const char* component_name)
{
    sim(ptr)->assign_component(eid, component_name);
}

API_EXPORT void get_component_json(void* ptr,
    cstr_result_callback callback,
    uint64_t eid,
    const char* component_name)
{
    callback(sim(ptr)->get_component_json(eid, component_name).c_str());
}

API_EXPORT void remove_component(void* ptr, uint64_t eid, const char * component_name)
{
    sim(ptr)->remove_component(eid, component_name);
}

API_EXPORT void replace_component(void* ptr,
    uint64_t eid,
    const char* component_name,
    const char* component_json)
{
    sim(ptr)->replace_component(eid, component_name, component_json);
}

API_EXPORT void get_component_names(void* ptr, cstr_result_callback callback)
{
    vector_to_callback(sim(ptr)->get_component_names(), callback);
}

API_EXPORT void get_entity_component_names(void* ptr,
    cstr_result_callback callback,
    uint64_t eid)
{
    vector_to_callback(sim(ptr)->get_entity_component_names(eid), callback);
}

API_EXPORT void get_singleton_json(void* ptr,
    cstr_result_callback callback,
    const char* singleton_name)
{
    callback(sim(ptr)->get_singleton_json(singleton_name).c_str());
}

API_EXPORT void set_singleton_json(void* ptr,
    const char* singleton_name,
    const char* singleton_json)
{
    sim(ptr)->set_singleton_json(singleton_name, singleton_json);
}

API_EXPORT void get_singleton_names(void* ptr, cstr_result_callback callback)
{
    vector_to_callback(sim(ptr)->get_singleton_names(), callback);
}

API_EXPORT void set_event_callback(void* ptr, cstr_result_callback callback)
{
    auto event_callback = [callback](std::string s) {
        callback(s.c_str());
    };

    sim(ptr)->set_event_callback(event_callback);
}
