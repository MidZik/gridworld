#include "stdafx.h"

#define RAPIDJSON_NOMEMBERITERATORCLASS
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/document.h>
#include <rapidjson/schema.h>
#include <condition_variable>
#include <unordered_map>

#ifdef MEASURE_PERF_SIMULATION_LOOP
#include <chrono>
#endif

#include "Simulation.h"
#include "components.h"
#include "Systems.h"
#include "Event.h"

using unique_lock = std::unique_lock<std::recursive_mutex>;

/*
Create this object to ensure the simulation enters and exits the waiting
state appropriately. Constructor returns when the simulation is waiting,
and the destructor puts the simulation back into the running state.
*/
class GridWorld::Simulation::WaitGuard
{
public:
    WaitGuard(Simulation const& sim) : _sim(sim)
    {
        _sim.requested_state = SimulationState::waiting;
        _sim.simulation_mutex.lock();
    }

    ~WaitGuard() noexcept
    {
        _sim.requested_state = SimulationState::running;
        _sim.simulation_mutex.unlock();
        _sim.simulation_waiter.notify_one();
    }

    WaitGuard(const WaitGuard&) = delete;
    WaitGuard& operator=(const WaitGuard&) = delete;
private:
    Simulation const& _sim;
};

namespace Reflect
{
    using namespace GridWorld::Component;

#define REFLECT_COM_NAME(com_class) template<> constexpr auto com_name<com_class>() { return #com_class; }

    template<class C>
    constexpr auto com_name()
    {
        static_assert(false, "Component has no name.");
    };

    REFLECT_COM_NAME(Position);
    REFLECT_COM_NAME(Moveable);
    REFLECT_COM_NAME(Name);
    REFLECT_COM_NAME(RNG);
    REFLECT_COM_NAME(SimpleBrain);
    REFLECT_COM_NAME(SimpleBrainSeer);
    REFLECT_COM_NAME(SimpleBrainMover);
    REFLECT_COM_NAME(Predation);
    REFLECT_COM_NAME(RandomMover);
    REFLECT_COM_NAME(Scorable);

    REFLECT_COM_NAME(STickCounter);
    REFLECT_COM_NAME(SWorld);
    REFLECT_COM_NAME(SEventsLog);

    template<class C>
    constexpr std::pair<ENTT_ID_TYPE, const char*> id_name_pair()
    {
        return { entt::type_info<C>::id(), com_name<C>() };
    };

    static const std::unordered_map<ENTT_ID_TYPE, const char*> com_id_name_map
    {
        id_name_pair<Position>(),
        id_name_pair<Moveable>(),
        id_name_pair<Name>(),
        id_name_pair<RNG>(),
        id_name_pair<SimpleBrain>(),
        id_name_pair<SimpleBrainSeer>(),
        id_name_pair<SimpleBrainMover>(),
        id_name_pair<Predation>(),
        id_name_pair<RandomMover>(),
        id_name_pair<Scorable>()
    };

    const char* id_to_com_name(ENTT_ID_TYPE id)
    {
        return com_id_name_map.at(id);
    };
}

namespace GridWorld::JSON
{
    using namespace GridWorld::Component;
    using namespace rapidjson;


    template<typename C>
    void json_write(std::vector<C> const& vec, Writer<StringBuffer>& writer);

    template<typename C>
    void json_read(std::vector<C>& vec, Value const& value);

    template<typename C>
    void json_write(std::map<std::string, C> const& map, Writer<StringBuffer>& writer);

    template<typename C>
    void json_read(std::map<std::string, C>& map, Value const& value);

    void json_write(STickCounter const& com, Writer<StringBuffer>& writer)
    {
        writer.Uint64(com.tick);
    }

    void json_read(STickCounter& com, Value const& value)
    {
        com.tick = value.GetUint64();
    }

    void json_write(SWorld const& com, Writer<StringBuffer>& writer)
    {
        writer.StartObject();

        writer.Key("width");
        writer.Int(com.width);
        writer.Key("height");
        writer.Int(com.height);

        writer.EndObject();
    }

    void json_read(SWorld& com, Value const& value)
    {
        com.width = value["width"].GetInt();
        com.height = value["height"].GetInt();
        com.reset_world();
    }

    void json_write(SEventsLog const& com, Writer<StringBuffer>& writer)
    {
        writer.StartObject();

        writer.Key("events_last_tick");
        json_write(com.events_last_tick, writer);

        writer.EndObject();
    }

    void json_read(SEventsLog& com, Value const& value)
    {
        json_read(com.events_last_tick, value["events_last_tick"]);
    }

    void json_write(Position const& pos, Writer<StringBuffer>& writer)
    {
        writer.StartObject();

        writer.Key("x");
        writer.Int(pos.x);
        writer.Key("y");
        writer.Int(pos.y);

        writer.EndObject();
    }

    void json_read(Position& com, Value const& value)
    {
        com.x = value["x"].GetInt();
        com.y = value["y"].GetInt();
    }

    void json_write(Moveable const& mov, Writer<StringBuffer>& writer)
    {
        writer.StartObject();

        writer.Key("x_force");
        writer.Int(mov.x_force);
        writer.Key("y_force");
        writer.Int(mov.y_force);

        writer.EndObject();
    }

    void json_read(Moveable& com, Value const& value)
    {
        com.x_force = value["x_force"].GetInt();
        com.y_force = value["y_force"].GetInt();
    }

    void json_write(Name const& name, Writer<StringBuffer>& writer)
    {
        writer.StartObject();

        writer.Key("major_name");
        writer.String(name.major_name.c_str(), (SizeType)name.major_name.length());
        writer.Key("minor_name");
        writer.String(name.minor_name.c_str(), (SizeType)name.minor_name.length());

        writer.EndObject();
    }

    void json_read(Name& com, Value const& value)
    {
        com.major_name = value["major_name"].GetString();
        com.minor_name = value["minor_name"].GetString();
    }

    void json_write(RNG const& rng, Writer<StringBuffer>& writer)
    {
        writer.StartObject();

        std::stringstream ss;
        ss << rng;
        auto state = ss.str();

        writer.Key("state");
        writer.String(state.c_str(), (SizeType)state.length());

        writer.EndObject();
    }

    void json_read(RNG& com, Value const& value)
    {
        std::stringstream ss;
        ss << value["state"].GetString();
        ss >> com;
    }

    void json_write(SimpleBrainSeer const& seer, Writer<StringBuffer>& writer)
    {
        writer.StartObject();

        writer.Key("neuron_offset");
        writer.Int(seer.neuron_offset);
        writer.Key("sight_radius");
        writer.Int(seer.sight_radius);

        writer.EndObject();
    }

    void json_read(SimpleBrainSeer& com, Value const& value)
    {
        com.neuron_offset = value["neuron_offset"].GetInt();
        com.sight_radius = value["sight_radius"].GetInt();
    }

    void json_write(SimpleBrainMover const& mover, Writer<StringBuffer>& writer)
    {
        writer.StartObject();

        writer.Key("neuron_offset");
        writer.Int(mover.neuron_offset);

        writer.EndObject();
    }

    void json_read(SimpleBrainMover& com, Value const& value)
    {
        com.neuron_offset = value["neuron_offset"].GetInt();
    }

    void json_write(Predation const& pred, Writer<StringBuffer>& writer)
    {
        writer.StartObject();

        writer.Key("no_predation_until_tick");
        writer.Uint64(pred.no_predation_until_tick);
        writer.Key("ticks_between_predations");
        writer.Uint(pred.ticks_between_predations);
        writer.Key("predate_all");
        writer.Bool(pred.predate_all);

        writer.EndObject();
    }

    void json_read(Predation& com, Value const& value)
    {
        com.no_predation_until_tick = value["no_predation_until_tick"].GetUint64();
        com.ticks_between_predations = value["ticks_between_predations"].GetUint();
        com.predate_all = value["predate_all"].GetBool();
    }

    void json_write(SynapseMat const& mat, Writer<StringBuffer>& writer)
    {
        writer.StartArray();

        for (int r = 0; r < mat.rows(); r++)
        {
            writer.StartArray();
            for (int c = 0; c < mat.cols(); c++)
            {
                writer.Double(mat(r, c));
            }
            writer.EndArray();
        }

        writer.EndArray();
    }

    void json_read(SynapseMat& com, Value const& value)
    {
        int rows = value.Size();
        int cols = value[0].Size();

        com.resize(rows, cols);

        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                com(r, c) = (float)value[r][c].GetDouble();
            }
        }
    }

    void json_write(NeuronMat const& mat, Writer<StringBuffer>& writer)
    {
        writer.StartArray();

        for (int c = 0; c < mat.cols(); c++)
        {
            writer.Double(mat(c));
        }

        writer.EndArray();
    }

    void json_read(NeuronMat& com, Value const& value)
    {
        int cols = value.Size();

        com.resize(cols);

        for (int c = 0; c < cols; c++)
        {
            com(c) = (float)value[c].GetDouble();
        }
    }

    void json_write(SimpleBrain const& com, Writer<StringBuffer>& writer)
    {
        writer.StartObject();

        writer.Key("child_mutation_chance");
        writer.Double(com.child_mutation_chance);
        writer.Key("child_mutation_strength");
        writer.Double(com.child_mutation_strength);
        writer.Key("synapses");
        json_write(com.synapses, writer);
        writer.Key("neurons");
        json_write(com.neurons, writer);

        writer.EndObject();
    }

    void json_read(SimpleBrain& com, Value const& value)
    {
        com.child_mutation_chance = (float)value["child_mutation_chance"].GetDouble();
        com.child_mutation_strength = (float)value["child_mutation_strength"].GetDouble();
        json_read(com.synapses, value["synapses"]);
        json_read(com.neurons, value["neurons"]);
    }

    void json_write(Scorable const& scorable, Writer<StringBuffer>& writer)
    {
        writer.StartObject();

        writer.Key("score");
        writer.Int(scorable.score);

        writer.EndObject();
    }

    void json_read(Scorable& com, Value const& value)
    {
        com.score = value["score"].GetInt();
    }

    void json_write(Events::Event::variant const& data, Writer<StringBuffer>& writer)
    {
        using namespace Events;

        if (std::holds_alternative<int>(data))
        {
            writer.Int(std::get<int>(data));
        }
        else if (std::holds_alternative<double>(data))
        {
            writer.Double(std::get<double>(data));
        }
        else if (std::holds_alternative<std::string>(data))
        {
            const std::string& str_data = std::get<std::string>(data);
            writer.String(str_data.c_str(), str_data.length());
        }
        else if (std::holds_alternative<Event::variant_map>(data))
        {
            const Event::variant_map& map = std::get<Event::variant_map>(data);
            json_write(map, writer);
        }
        else if (std::holds_alternative<Event::variant_vector>(data))
        {
            const Event::variant_vector& vec = std::get<Event::variant_vector>(data);
            json_write(vec, writer);
        }
        else if (std::holds_alternative<std::monostate>(data))
        {
            writer.Null();
        }
    }

    void json_read(Events::Event::variant& data, Value const& value)
    {
        using namespace Events;
        if (value.IsInt())
        {
            data = value.GetInt();
        }
        else if (value.IsDouble())
        {
            data = value.GetDouble();
        }
        else if (value.IsString())
        {
            data = value.GetString();
        }
        else if (value.IsObject())
        {
            Event::variant_map map;
            json_read(map, value);
            data = std::move(map);
        }
        else if (value.IsArray())
        {
            Event::variant_vector vec;
            json_read(vec, value);
            data = std::move(vec);
        }
        else if (value.IsNull())
        {
            data = std::monostate();
        }
        else
        {
            throw std::exception("Error reading EventData JSON: invalid data format");
        }
    }

    void json_write(Events::Event const& event, Writer<StringBuffer>& writer)
    {
        writer.StartObject();

        writer.Key("name");
        writer.String(event.name.c_str(), event.name.length());
        writer.Key("data");
        json_write(event.data, writer);

        writer.EndObject();
    }

    void json_read(Events::Event& event, Value const& value)
    {
        event.name = value["name"].GetString();
        json_read(event.data, value["data"]);
    }

    template<typename C>
    void json_write(std::vector<C> const& vec, Writer<StringBuffer>& writer)
    {
        writer.StartArray();

        for (C const& com : vec)
        {
            json_write(com, writer);
        }

        writer.EndArray();
    }

    template<typename C>
    void json_read(std::vector<C>& vec, Value const& value)
    {
        vec.clear();
        vec.resize(value.Size());

        for (int i = 0; i < vec.size(); i++)
        {
            json_read(vec[i], value[i]);
        }
    }

    template<typename C>
    void json_write(std::map<std::string, C> const& map, Writer<StringBuffer>& writer)
    {
        writer.StartObject();

        for (auto const&[key, value] : map)
        {
            writer.Key(key.c_str(), key.length());
            json_write(value, writer);
        }

        writer.EndObject();
    }

    template<typename C>
    void json_read(std::map<std::string, C>& map, Value const& value)
    {
        map.clear();

        for (Value::ConstMemberIterator itr = value.MemberBegin();
             itr != value.MemberEnd(); ++itr)
        {
            json_read(map[itr->name.GetString()], itr->value);
        }
    }

    template<typename C>
    void json_write_components_array(registry const& reg, Writer<StringBuffer>& writer)
    {
        writer.StartArray();

        const EntityId* entities = reg.data<C>();
        const C* coms = reg.raw<C>();
        for (int i = 0; i < reg.size<C>(); i++)
        {
            writer.StartObject();
            writer.Key("EID");
            writer.Uint64(to_integral(entities[i]));
            writer.Key("Com");
            json_write(coms[i], writer);
            writer.EndObject();
        }

        writer.EndArray();
    }

    template<typename C>
    void json_read_components_array(registry& reg, Value const& value)
    {
        for (auto const& item : value.GetArray())
        {
            EntityId eid = (EntityId)item["EID"].GetUint64();
            json_read(reg.assign<C>(eid), item["Com"]);
        }
    }

    template<typename C>
    void json_write_tags_array(registry const& reg, Writer<StringBuffer>& writer)
    {
        writer.StartArray();

        const EntityId* entities = reg.data<C>();
        for (int i = 0; i < reg.size<C>(); i++)
        {
            writer.Uint64(to_integral(entities[i]));
        }

        writer.EndArray();
    }

    template<typename C>
    void json_read_tags_array(registry& reg, Value const& value)
    {
        for (auto const& item : value.GetArray())
        {
            EntityId eid = (EntityId)item.GetUint64();
            reg.assign<C>(eid);
        }
    }

    template<typename C>
    void json_read(C& com, std::string const& json)
    {
        // parse input
        Document doc;
        doc.Parse(json.c_str(), json.size());
        if (doc.HasParseError())
        {
            throw std::invalid_argument("Input is not valid JSON.");
        }

        json_read(com, doc);
    }
}

GridWorld::registry create_empty_simulation_registry()
{
    using namespace GridWorld::Component;
    GridWorld::registry reg;

    reg.ctx_or_set<STickCounter>();
    reg.ctx_or_set<SWorld>();
    reg.ctx_or_set<SEventsLog>();

    return reg;
}

GridWorld::Simulation::Simulation()
{
    reg = create_empty_simulation_registry();
    requested_state = SimulationState::running;
}

uint64_t GridWorld::Simulation::get_tick() const
{
    return reg.ctx<Component::STickCounter>().tick;
}

std::string GridWorld::Simulation::get_state_json() const
{
    using namespace GridWorld::JSON;
    using namespace rapidjson;

    WaitGuard wait_guard(*this);

    StringBuffer buf;
    Writer<StringBuffer> writer(buf);

    writer.StartObject();

    writer.Key("entities");
    {
        writer.StartArray();
        for (int i = 0; i < reg.size(); i++)
        {
            writer.Uint64(to_integral(reg.data()[i]));
        }
        writer.EndArray();
    } // entities

    writer.Key("singletons");
    {
        writer.StartObject();

        writer.Key("STickCounter");
        json_write(reg.ctx<STickCounter>(), writer);

        writer.Key("SWorld");
        json_write(reg.ctx<SWorld>(), writer);

        writer.Key("SEventsLog");
        json_write(reg.ctx<SEventsLog>(), writer);

        writer.EndObject();
    } // singletons

    writer.Key("components");
    {
        writer.StartObject();

        writer.Key("Position");
        json_write_components_array<Position>(reg, writer);

        writer.Key("Moveable");
        json_write_components_array<Moveable>(reg, writer);

        writer.Key("Name");
        json_write_components_array<Name>(reg, writer);

        writer.Key("RNG");
        json_write_components_array<RNG>(reg, writer);

        writer.Key("SimpleBrain");
        json_write_components_array<SimpleBrain>(reg, writer);

        writer.Key("SimpleBrainSeer");
        json_write_components_array<SimpleBrainSeer>(reg, writer);

        writer.Key("SimpleBrainMover");
        json_write_components_array<SimpleBrainMover>(reg, writer);

        writer.Key("Predation");
        json_write_components_array<Predation>(reg, writer);

        writer.Key("Scorable");
        json_write_components_array<Scorable>(reg, writer);

        writer.Key("RandomMover");
        json_write_tags_array<RandomMover>(reg, writer);

        writer.EndObject();
    } // components

    writer.EndObject(); // root

    return buf.GetString();
}

const char * state_schema = R"xx(
{
    "$schema": "http://json-schema.org/draft-04/schema#",
    "id": "GridWorld/StateJson",
    "type": "object",
    "properties": {
        "entities": {
            "type": "array",
            "uniqueItems": true,
            "items": {
                "type": "integer"
            }
        },
        "singletons": { "type": "object" },
        "components": {
            "type": "object",
            "additionalProperties": {
                "type": "array",
                "items": {
                    "oneOf": [
                        {
                            "type": "object",
                            "properties": {
                                "EID": { "type": "integer" },
                                "Com": true
                            },
                            "required": ["EID", "Com"]
                        },
                        {
                            "type": "integer"
                        }
                    ]
                }
            }
        }
    }
}
)xx";

void GridWorld::Simulation::set_state_json(std::string json)
{
    using namespace GridWorld::JSON;
    using namespace rapidjson;

    if (is_running())
    {
        throw std::exception("set_state_json cannot be used while simulation is running.");
    }

    registry tmp = create_empty_simulation_registry();

    // parse input
    Document doc;
    doc.Parse(json.c_str(), json.size());
    if (doc.HasParseError())
    {
        throw std::invalid_argument("Input is not valid JSON.");
    }

    // setup schema
    Document schema_doc;
    schema_doc.Parse(state_schema, strlen(state_schema));
    if (schema_doc.HasParseError())
    {
        // this is an internal error
        throw std::exception("Internal error. (Schema error)");
    }
    SchemaDocument schema(schema_doc);

    SchemaValidator validator(schema);
    if (!doc.Accept(validator))
    {
        // Input is invalid according to the schema.
        throw std::invalid_argument("Input failed schema validation.");
    }

    const Value& entities = doc["entities"];
    const Value& singletons = doc["singletons"];
    const Value& components = doc["components"];

    {
        // Entities
        std::vector<EntityId> ints;
        for (auto& v : entities.GetArray())
        {
            ints.push_back((EntityId)v.GetInt64());
        }

        tmp.assign(ints.begin(), ints.end());
    }

    {
        // Singletons
        json_read(tmp.ctx<STickCounter>(), singletons["STickCounter"]);

        json_read(tmp.ctx<SWorld>(), singletons["SWorld"]);

        json_read(tmp.ctx<SEventsLog>(), singletons["SEventsLog"]);
    }

    {
        // Components
        json_read_components_array<Position>(tmp, components["Position"]);

        json_read_components_array<Moveable>(tmp, components["Moveable"]);

        json_read_components_array<Name>(tmp, components["Name"]);

        json_read_components_array<RNG>(tmp, components["RNG"]);

        json_read_components_array<SimpleBrain>(tmp, components["SimpleBrain"]);

        json_read_components_array<SimpleBrainSeer>(tmp, components["SimpleBrainSeer"]);

        json_read_components_array<SimpleBrainMover>(tmp, components["SimpleBrainMover"]);

        json_read_components_array<Predation>(tmp, components["Predation"]);

        json_read_components_array<Scorable>(tmp, components["Scorable"]);

        json_read_tags_array<RandomMover>(tmp, components["RandomMover"]);
    }

    reg = std::move(tmp);
}

uint64_t GridWorld::Simulation::create_entity()
{
    if (is_running())
    {
        throw std::exception("create_entity cannot be used while simulation is running.");
    }

    return to_integral(reg.create());
}

void GridWorld::Simulation::destroy_entity(uint64_t eid)
{
    if (is_running())
    {
        throw std::exception("destroy_entity cannot be used while simulation is running.");
    }

    reg.destroy(EntityId{ eid });
}

std::vector<uint64_t> GridWorld::Simulation::get_all_entities() const
{
    WaitGuard wait_guard(*this);

    std::vector<uint64_t> result;
    result.reserve(reg.size());
    const GridWorld::EntityId* data = reg.data();
    for (int i = 0; i < reg.size(); i++)
    {
        if (reg.valid(data[i]))
        {
            result.push_back(to_integral(reg.data()[i]));
        }
    }

    return result;
}

void GridWorld::Simulation::start_simulation()
{
    if (!is_running())
    {
        // Since the state may have been changed externally while the simulation
        // wasn't running, ensure any hidden state is properly synced up
        Systems::Util::rebuild_world(reg);

        requested_state = SimulationState::running;
        simulation_thread = std::thread(&Simulation::simulation_loop, this);
    }
}

void GridWorld::Simulation::stop_simulation()
{
    if (is_running())
    {
        requested_state = SimulationState::stopped;
        simulation_thread.join();
    }
}

bool GridWorld::Simulation::is_running() const
{
    return simulation_thread.joinable();
}

void GridWorld::Simulation::assign_component(uint64_t eid_int, std::string component_name)
{
    using namespace Component;
    using namespace Reflect;

    if (is_running())
    {
        throw std::exception("assign_component cannot be used while simulation is running.");
    }

    EntityId eid = EntityId(eid_int);

    WaitGuard wait_guard(*this);

    if (component_name == com_name<Position>())
    {
        reg.assign<Position>(eid);
    }
    else if (component_name == com_name<Moveable>())
    {
        reg.assign<Moveable>(eid);
    }
    else if (component_name == com_name<Name>())
    {
        reg.assign<Name>(eid);
    }
    else if (component_name == com_name<RNG>())
    {
        reg.assign<RNG>(eid);
    }
    else if (component_name == com_name<SimpleBrain>())
    {
        reg.assign<SimpleBrain>(eid);
    }
    else if (component_name == com_name<SimpleBrainSeer>())
    {
        reg.assign<SimpleBrainSeer>(eid);
    }
    else if (component_name == com_name<SimpleBrainMover>())
    {
        reg.assign<SimpleBrainMover>(eid);
    }
    else if (component_name == com_name<Predation>())
    {
        reg.assign<Predation>(eid);
    }
    else if (component_name == com_name<RandomMover>())
    {
        reg.assign<RandomMover>(eid);
    }
    else if (component_name == com_name<Scorable>())
    {
        reg.assign<Scorable>(eid);
    }
    else
    {
        throw std::exception(("Unknown component type passed to assign_component: " + component_name).c_str());
    }
}

std::string GridWorld::Simulation::get_component_json(uint64_t eid_int, std::string component_name) const
{
    using namespace Component;
    using namespace Reflect;
    using namespace GridWorld::JSON;
    using namespace rapidjson;

    EntityId eid = EntityId(eid_int);
    StringBuffer buf;
    Writer<StringBuffer> writer(buf);

    WaitGuard wait_guard(*this);

    if (component_name == com_name<Position>())
    {
        json_write(reg.get<Position>(eid), writer);
    }
    else if (component_name == com_name<Moveable>())
    {
        json_write(reg.get<Moveable>(eid), writer);
    }
    else if (component_name == com_name<Name>())
    {
        json_write(reg.get<Name>(eid), writer);
    }
    else if (component_name == com_name<RNG>())
    {
        json_write(reg.get<RNG>(eid), writer);
    }
    else if (component_name == com_name<SimpleBrain>())
    {
        json_write(reg.get<SimpleBrain>(eid), writer);
    }
    else if (component_name == com_name<SimpleBrainSeer>())
    {
        json_write(reg.get<SimpleBrainSeer>(eid), writer);
    }
    else if (component_name == com_name<SimpleBrainMover>())
    {
        json_write(reg.get<SimpleBrainMover>(eid), writer);
    }
    else if (component_name == com_name<Predation>())
    {
        json_write(reg.get<Predation>(eid), writer);
    }
    else if (component_name == com_name<RandomMover>())
    {
        writer.Null(); // tags have no data
    }
    else if (component_name == com_name<Scorable>())
    {
        json_write(reg.get<Scorable>(eid), writer);
    }
    else
    {
        throw std::exception(("Unknown component type passed to get_component_json: " + component_name).c_str());
    }

    return buf.GetString();
}

void GridWorld::Simulation::remove_component(uint64_t eid_int, std::string component_name)
{
    using namespace Component;
    using namespace Reflect;

    if (is_running())
    {
        throw std::exception("remove_component cannot be used while simulation is running.");
    }

    EntityId eid = EntityId(eid_int);

    WaitGuard wait_guard(*this);

    if (component_name == com_name<Position>())
    {
        reg.remove<Position>(eid);
    }
    else if (component_name == com_name<Moveable>())
    {
        reg.remove<Moveable>(eid);
    }
    else if (component_name == com_name<Name>())
    {
        reg.remove<Name>(eid);
    }
    else if (component_name == com_name<RNG>())
    {
        reg.remove<RNG>(eid);
    }
    else if (component_name == com_name<SimpleBrain>())
    {
        reg.remove<SimpleBrain>(eid);
    }
    else if (component_name == com_name<SimpleBrainSeer>())
    {
        reg.remove<SimpleBrainSeer>(eid);
    }
    else if (component_name == com_name<SimpleBrainMover>())
    {
        reg.remove<SimpleBrainMover>(eid);
    }
    else if (component_name == com_name<Predation>())
    {
        reg.remove<Predation>(eid);
    }
    else if (component_name == com_name<RandomMover>())
    {
        reg.remove<RandomMover>(eid);
    }
    else if (component_name == com_name<Scorable>())
    {
        reg.remove<Scorable>(eid);
    }
    else
    {
        throw std::exception(("Unknown component type passed to remove_component: " + component_name).c_str());
    }
}

void GridWorld::Simulation::replace_component(uint64_t eid_int, std::string component_name, std::string component_json)
{
    using namespace Component;
    using namespace Reflect;

    if (is_running())
    {
        throw std::exception("replace_component cannot be used while simulation is running.");
    }

    EntityId eid = EntityId(eid_int);

    WaitGuard wait_guard(*this);

    if (component_name == com_name<Position>())
    {
        JSON::json_read(reg.get<Position>(eid), component_json);
    }
    else if (component_name == com_name<Moveable>())
    {
        JSON::json_read(reg.get<Moveable>(eid), component_json);
    }
    else if (component_name == com_name<Name>())
    {
        JSON::json_read(reg.get<Name>(eid), component_json);
    }
    else if (component_name == com_name<RNG>())
    {
        JSON::json_read(reg.get<RNG>(eid), component_json);
    }
    else if (component_name == com_name<SimpleBrain>())
    {
        JSON::json_read(reg.get<SimpleBrain>(eid), component_json);
    }
    else if (component_name == com_name<SimpleBrainSeer>())
    {
        JSON::json_read(reg.get<SimpleBrainSeer>(eid), component_json);
    }
    else if (component_name == com_name<SimpleBrainMover>())
    {
        JSON::json_read(reg.get<SimpleBrainMover>(eid), component_json);
    }
    else if (component_name == com_name<Predation>())
    {
        JSON::json_read(reg.get<Predation>(eid), component_json);
    }
    else if (component_name == com_name<Scorable>())
    {
        JSON::json_read(reg.get<Scorable>(eid), component_json);
    }
    else
    {
        throw std::exception(("Unknown component type passed to replace_component: " + component_name).c_str());
    }
}

std::vector<std::string> GridWorld::Simulation::get_component_names() const
{
    using namespace Reflect;
    return std::vector<std::string> {
        com_name<Position>(),
        com_name<Moveable>(),
        com_name<Name>(),
        com_name<RNG>(),
        com_name<SimpleBrain>(),
        com_name<SimpleBrainSeer>(),
        com_name<SimpleBrainMover>(),
        com_name<Predation>(),
        com_name<RandomMover>(),
        com_name<Scorable>()
    };
}

std::vector<std::string> GridWorld::Simulation::get_entity_component_names(uint64_t eid) const
{
    using namespace Reflect;
    std::vector<std::string> result;

    WaitGuard wait_guard(*this);

    reg.visit(EntityId(eid), [&result](ENTT_ID_TYPE com_id)
    {
        result.push_back(id_to_com_name(com_id));
    });
    return result;
}

std::string GridWorld::Simulation::get_singleton_json(std::string singleton_name) const
{
    using namespace Component;
    using namespace Reflect;
    using namespace GridWorld::JSON;
    using namespace rapidjson;

    StringBuffer buf;
    Writer<StringBuffer> writer(buf);

    WaitGuard wait_guard(*this);

    if (singleton_name == com_name<SWorld>())
    {
        json_write(reg.ctx<SWorld>(), writer);
    }
    else if (singleton_name == com_name<SEventsLog>())
    {
        json_write(reg.ctx<SEventsLog>(), writer);
    }
    else
    {
        throw std::exception(("Unknown component type passed to get_singleton_json: " + singleton_name).c_str());
    }

    return buf.GetString();
}

void GridWorld::Simulation::set_singleton_json(std::string singleton_name, std::string singleton_json)
{
    using namespace Component;
    using namespace Reflect;

    if (is_running())
    {
        throw std::exception("set_singleton_json cannot be used while simulation is running.");
    }

    WaitGuard wait_guard(*this);

    if (singleton_name == com_name<SWorld>())
    {
        JSON::json_read(reg.ctx<SWorld>(), singleton_json);
    }
    else if (singleton_name == com_name<SEventsLog>())
    {
        JSON::json_read(reg.ctx<SEventsLog>(), singleton_json);
    }
    else
    {
        throw std::exception(("Unknown component type passed to set_singleton_json: " + singleton_name).c_str());
    }
}

std::vector<std::string> GridWorld::Simulation::get_singleton_names() const
{
    using namespace Reflect;
    return std::vector<std::string> {
        com_name<SWorld>(),
        com_name<SEventsLog>()
    };
}

void GridWorld::Simulation::set_event_callback(event_callback_function callback)
{
    event_callback = callback;
}

void update_tick(GridWorld::registry& reg)
{
    using namespace GridWorld::Systems;

    tick_increment(reg);
    simple_brain_seer(reg);
    simple_brain_calc(reg);
    simple_brain_mover(reg);
    random_movement(reg);
    movement(reg);
    predation(reg);
    evolution(reg);
    finalize_event_log(reg);
}

void GridWorld::Simulation::simulation_loop()
{
#ifdef MEASURE_PERF_SIMULATION_LOOP
    using namespace std::chrono;

    high_resolution_clock::time_point last_event_time = high_resolution_clock::now();
#endif // MEASURE_PERF_SIMULATION_LOOP

    unique_lock simulation_lock(simulation_mutex);
    while (true)
    {
        switch (requested_state)
        {
            case SimulationState::running:
                break;
            case SimulationState::waiting:
                while (requested_state == SimulationState::waiting)
                {
                    simulation_waiter.wait(simulation_lock);
                }
                // restart the enclosing loop
                continue;
            case SimulationState::stopped:
            default:
                return;
        }

        update_tick(reg);

#ifdef MEASURE_PERF_SIMULATION_LOOP
        high_resolution_clock::time_point last_update_end_time = high_resolution_clock::now();
#endif

        // Publish events that occured last tick
        const auto& events_last_tick = reg.ctx<Component::SEventsLog>().events_last_tick;
        if (events_last_tick.size() > 0)
        {
            using namespace GridWorld::JSON;
            using namespace rapidjson;

            StringBuffer buf;
            Writer<StringBuffer> writer(buf);

            json_write(events_last_tick, writer);

            simulation_lock.unlock();
            event_callback(buf.GetString());
            simulation_lock.lock();

#ifdef MEASURE_PERF_SIMULATION_LOOP
            high_resolution_clock::time_point last_callback_end_time = high_resolution_clock::now();

            duration<double, std::milli> total_dur = last_callback_end_time - last_event_time;
            duration<double, std::milli> update_dur = last_update_end_time - last_event_time;
            duration<double, std::milli> callback_dur = last_callback_end_time - last_update_end_time;

            std::printf("Total time: %.2f, Update time: %.2f, Callback time: %.2f, Callback Ratio: %.2f%%\r\n",
                        total_dur.count(), update_dur.count(), callback_dur.count(),
                        (100 * callback_dur.count() / total_dur.count()));
            std::fflush(stdout);

            last_event_time = high_resolution_clock::now();
#endif // MEASURE_PERF_SIMULATION_LOOP
        }
    }
}
