#include "stdafx.h"

#define RAPIDJSON_NOMEMBERITERATORCLASS
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/document.h>
#include <rapidjson/schema.h>
#include <condition_variable>

#include "Simulation.h"
#include "components.h"
#include "Systems.h"

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

namespace GridWorld::JSON
{
    using namespace GridWorld::Component;
    using namespace rapidjson;

    template<typename C>
    void json_write(std::vector<C> const& vec, Writer<StringBuffer>& writer);

    template<typename C>
    void json_read(std::vector<C>& vec, Value const& value);

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
        writer.String(name.minor_name.c_str(), (SizeType)name.major_name.length());

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
        "tick": { "type": "integer" },
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

    if (simulation_thread.joinable())
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
    if (simulation_thread.joinable())
    {
        throw std::exception("create_entity cannot be used while simulation is running.");
    }

    return to_integral(reg.create());
}

void GridWorld::Simulation::destroy_entity(uint64_t eid)
{
    if (simulation_thread.joinable())
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
    if (!simulation_thread.joinable())
    {
        requested_state = SimulationState::running;
        simulation_thread = std::thread(&Simulation::simulation_loop, this);
    }
}

void GridWorld::Simulation::stop_simulation()
{
    if (simulation_thread.joinable())
    {
        requested_state = SimulationState::stopped;
        simulation_thread.join();
    }
}

void GridWorld::Simulation::assign_component(uint64_t eid_int, std::string component_name)
{
    using namespace Component;

    if (simulation_thread.joinable())
    {
        throw std::exception("assign_component cannot be used while simulation is running.");
    }

    EntityId eid = EntityId(eid_int);
    if (component_name == "Position")
    {
        reg.assign<Position>(eid);
    }
    else if (component_name == "Moveable")
    {
        reg.assign<Moveable>(eid);
    }
    else if (component_name == "Name")
    {
        reg.assign<Name>(eid);
    }
    else if (component_name == "RNG")
    {
        reg.assign<RNG>(eid);
    }
    else if (component_name == "SimpleBrain")
    {
        reg.assign<SimpleBrain>(eid);
    }
    else if (component_name == "SimpleBrainSeer")
    {
        reg.assign<SimpleBrainSeer>(eid);
    }
    else if (component_name == "SimpleBrainMover")
    {
        reg.assign<SimpleBrainMover>(eid);
    }
    else if (component_name == "Predation")
    {
        reg.assign<Predation>(eid);
    }
    else if (component_name == "RandomMover")
    {
        reg.assign<RandomMover>(eid);
    }
    else if (component_name == "Scorable")
    {
        reg.assign<Scorable>(eid);
    }
    else
    {
        throw std::exception(("Unknown component type passed to assign_component: " + component_name).c_str());
    }
}

std::vector<std::string> GridWorld::Simulation::get_component_names() const
{
    return std::vector<std::string> {
        "Position",
        "Moveable",
        "Name",
        "RNG",
        "SimpleBrain",
        "SimpleBrainSeer",
        "SimpleBrainMover",
        "Predation",
        "RandomMover",
        "Scorable"
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

        // Publish events that occured last tick
        for (Events::Event& e : reg.ctx<Component::SEventsLog>().events_last_tick)
        {
            event_callback(e.name);
        }
    }
}
