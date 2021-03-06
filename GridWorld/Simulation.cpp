#include "stdafx.h"

#define RAPIDJSON_NOMEMBERITERATORCLASS
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/document.h>
#include <rapidjson/schema.h>
#include <condition_variable>
#include <unordered_map>
#include <charconv>
#include <random>

#include "Simulation.h"
#include "components.h"
#include "Systems.h"
#include "Event.h"

using unique_lock = std::unique_lock<std::shared_mutex>;
using shared_lock = std::shared_lock<std::shared_mutex>;

class shared_pause_lock
{
public:
    shared_pause_lock(std::atomic<uint32_t>& pause_requests,
        std::condition_variable_any& no_pauses_requested,
        std::shared_mutex& shared_mutex) :
        pause_requests(pause_requests),
        no_pauses_requested(no_pauses_requested),
        shared_mutex(shared_mutex)
    {
        ++pause_requests;
        shared_mutex.lock_shared();
    };

    ~shared_pause_lock()
    {
        shared_mutex.unlock_shared();
        if (--pause_requests == 0)
        {
            no_pauses_requested.notify_one();
        }
    }
private:
    std::atomic<uint32_t>& pause_requests;
    std::condition_variable_any& no_pauses_requested;
    std::shared_mutex& shared_mutex;
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

    REFLECT_COM_NAME(SSimulationConfig);
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

    void json_write(SSimulationConfig const& com, Writer<StringBuffer>& writer)
    {
        writer.StartObject();

        writer.Key("evo_ticks_per_evolution");
        writer.Uint(com.evo_ticks_per_evolution);
        writer.Key("evo_winner_count");
        writer.Uint(com.evo_winner_count);
        writer.Key("evo_new_entity_count");
        writer.Uint(com.evo_new_entity_count);

        writer.EndObject();
    }

    void json_read(SSimulationConfig& com, Value const& value)
    {
        com.evo_ticks_per_evolution = value["evo_ticks_per_evolution"].GetUint();
        com.evo_winner_count = value["evo_winner_count"].GetUint();
        com.evo_new_entity_count = value["evo_new_entity_count"].GetUint();
    }

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

namespace GridWorld::Binary
{
    using namespace GridWorld::Component;
    using buffer = std::vector<char>;

    template<class T>
    void push_into_buffer(buffer& buf, const T& obj);

    template<class T>
    size_t copy_from_buffer(const char* buf, const char* buf_end, T& obj);

    template<class T>
    void push_into_buffer(buffer& buf, const T* obj_array, size_t count);

    template<class T>
    size_t copy_from_buffer(const char* buf, const char* buf_end, T* obj_array, size_t count);

    template<class T>
    void push_into_buffer(buffer& buf, const std::vector<T>& vec);

    template<class T>
    size_t copy_from_buffer(const char* buf, const char* buf_end, std::vector<T>& vec);

    template<class K, class V>
    void push_into_buffer(buffer& buf, const std::map<K, V>& map);

    template<class K, class V>
    size_t copy_from_buffer(const char* buf, const char* buf_end, std::map<K, V>& map);

    template<class T>
    void push_array_into_buffer(buffer& buf, const T* obj_array, size_t count);

    void push_into_buffer(buffer& buf, const char* data, size_t len)
    {
        buf.insert(buf.end(), data, data + len);
    }

    size_t copy_from_buffer(const char* buf, const char* buf_end, char* dest, size_t len)
    {
        if (buf + len > buf_end)
        {
            throw std::runtime_error("Failed to copy from buffer: source too small");
        }
        memcpy(dest, buf, len);
        return len;
    }

    void push_into_buffer(buffer& buf, const std::string& obj)
    {
        push_into_buffer(buf, obj.size());
        push_into_buffer(buf, obj.c_str(), obj.size());
    }

    size_t copy_from_buffer(const char* buf, const char* buf_end, std::string& obj)
    {
        size_t offset = 0;
        size_t count = 0;
        offset += copy_from_buffer(buf + offset, buf_end, count);

        if (buf + offset + count > buf_end)
        {
            throw std::runtime_error("Failed to copy from buffer: source too small");
        }

        obj = std::string(buf + offset, count);

        return offset + count;
    }

    void push_into_buffer(buffer& buf, const SWorld& obj)
    {
        push_into_buffer(buf, obj.width);
        push_into_buffer(buf, obj.height);
    }

    size_t copy_from_buffer(const char* buf, const char* buf_end, SWorld& obj)
    {
        size_t offset = 0;
        offset += copy_from_buffer(buf + offset, buf_end, obj.width);
        offset += copy_from_buffer(buf + offset, buf_end, obj.height);

        return offset;
    }

    void push_into_buffer(buffer& buf, const Events::Event::variant& obj)
    {
        using namespace Events;

        if (std::holds_alternative<int>(obj))
        {
            push_into_buffer(buf, (char)1);
            push_into_buffer(buf, std::get<int>(obj));
        }
        else if (std::holds_alternative<double>(obj))
        {
            push_into_buffer(buf, (char)2);
            push_into_buffer(buf, std::get<double>(obj));
        }
        else if (std::holds_alternative<std::string>(obj))
        {
            push_into_buffer(buf, (char)3);
            const std::string& str = std::get<std::string>(obj);
            push_into_buffer(buf, str);
        }
        else if (std::holds_alternative<Event::variant_map>(obj))
        {
            push_into_buffer(buf, (char)4);
            const Event::variant_map& map = std::get<Event::variant_map>(obj);
            push_into_buffer(buf, map);
        }
        else if (std::holds_alternative<Event::variant_vector>(obj))
        {
            push_into_buffer(buf, (char)5);
            const Event::variant_vector& vec = std::get<Event::variant_vector>(obj);
            push_into_buffer(buf, vec);
        }
        else if (std::holds_alternative<std::monostate>(obj))
        {
            push_into_buffer(buf, (char)0);
        }
    }

    size_t copy_from_buffer(const char* buf, const char* buf_end, Events::Event::variant& obj)
    {
        using namespace Events;

        size_t offset = 0;
        char type;
        offset += copy_from_buffer(buf + offset, buf_end, type);
        switch (type)
        {
        case 0:
        {
            obj = std::monostate();
            break;
        }
        case 1:
        {
            int result;
            offset += copy_from_buffer(buf + offset, buf_end, result);
            obj = result;
            break;
        }
        case 2:
        {
            double result;
            offset += copy_from_buffer(buf + offset, buf_end, result);
            obj = result;
            break;
        }
        case 3:
        {
            std::string result;
            offset += copy_from_buffer(buf + offset, buf_end, result);
            obj = std::move(result);
            break;
        }
        case 4:
        {
            Event::variant_map result;
            offset += copy_from_buffer(buf + offset, buf_end, result);
            obj = std::move(result);
            break;
        }
        case 5:
        {
            Event::variant_vector result;
            offset += copy_from_buffer(buf + offset, buf_end, result);
            obj = std::move(result);
            break;
        }
        default:
            throw std::out_of_range("Unknown event variant type encountered while copying from buffer.");
        }

        return offset;
    }

    void push_into_buffer(buffer& buf, const Events::Event& obj)
    {
        push_into_buffer(buf, obj.name);
        push_into_buffer(buf, obj.data);
    }

    size_t copy_from_buffer(const char* buf, const char* buf_end, Events::Event& obj)
    {
        size_t offset = 0;
        offset += copy_from_buffer(buf + offset, buf_end, obj.name);
        offset += copy_from_buffer(buf + offset, buf_end, obj.data);
        return offset;
    }

    void push_into_buffer(buffer& buf, const SEventsLog& obj)
    {
        push_into_buffer(buf, obj.events_last_tick);
    }

    size_t copy_from_buffer(const char* buf, const char* buf_end, SEventsLog& obj)
    {
        return copy_from_buffer(buf, buf_end, obj.events_last_tick);
    }

    void push_into_buffer(buffer& buf, const Name& obj)
    {
        push_into_buffer(buf, obj.major_name);
        push_into_buffer(buf, obj.minor_name);
    }

    size_t copy_from_buffer(const char* buf, const char* buf_end, Name& obj)
    {
        size_t offset = 0;
        offset += copy_from_buffer(buf + offset, buf_end, obj.major_name);
        offset += copy_from_buffer(buf + offset, buf_end, obj.minor_name);
        return offset;
    }

    void push_into_buffer(buffer& buf, const SynapseMat& obj)
    {
        push_into_buffer(buf, obj.rows());
        push_into_buffer(buf, obj.cols());
        push_into_buffer(buf, obj.data(), obj.size());
    }

    size_t copy_from_buffer(const char* buf, const char* buf_end, SynapseMat& obj)
    {
        size_t offset = 0;
        Eigen::Index rows, cols;
        offset += copy_from_buffer(buf + offset, buf_end, rows);
        offset += copy_from_buffer(buf + offset, buf_end, cols);
        obj.resize(rows, cols);
        offset += copy_from_buffer(buf + offset, buf_end, obj.data(), obj.size());
        return offset;
    }

    void push_into_buffer(buffer& buf, const NeuronMat& obj)
    {
        push_into_buffer(buf, obj.cols());
        push_into_buffer(buf, obj.data(), obj.size());
    }

    size_t copy_from_buffer(const char* buf, const char* buf_end, NeuronMat& obj)
    {
        size_t offset = 0;
        Eigen::Index cols;
        offset += copy_from_buffer(buf + offset, buf_end, cols);
        obj.resize(cols);
        offset += copy_from_buffer(buf + offset, buf_end, obj.data(), obj.size());
        return offset;
    }

    void push_into_buffer(buffer& buf, const SimpleBrain& obj)
    {
        push_into_buffer(buf, obj.child_mutation_chance);
        push_into_buffer(buf, obj.child_mutation_strength);
        push_into_buffer(buf, obj.synapses);
        push_into_buffer(buf, obj.neurons);
    }

    size_t copy_from_buffer(const char* buf, const char* buf_end, SimpleBrain& obj)
    {
        size_t offset = 0;
        offset += copy_from_buffer(buf + offset, buf_end, obj.child_mutation_chance);
        offset += copy_from_buffer(buf + offset, buf_end, obj.child_mutation_strength);
        offset += copy_from_buffer(buf + offset, buf_end, obj.synapses);
        offset += copy_from_buffer(buf + offset, buf_end, obj.neurons);
        return offset;
    }

    template<class S>
    void push_singleton_into_buffer(buffer& buf, const GridWorld::registry& reg)
    {
        push_into_buffer(buf, reg.ctx<S>());
    }

    template<class S>
    size_t copy_singleton_from_buffer(const char* buf, const char* buf_end, GridWorld::registry& reg)
    {
        return copy_from_buffer(buf, buf_end, reg.ctx<S>());
    }

    template<class C>
    void push_components_into_buffer(buffer& buf, const GridWorld::registry& reg)
    {
        size_t count = reg.size<C>();
        push_into_buffer(buf, count);
        push_into_buffer(buf, reg.data<C>(), count);
        push_into_buffer(buf, reg.raw<C>(), count);
    }

    template<class C>
    size_t copy_components_from_buffer(const char* buf, const char* buf_end, GridWorld::registry& reg)
    {
        size_t offset = 0;
        size_t count = 0;
        offset += copy_from_buffer(buf + offset, buf_end, count);

        for (int i = 0; i < count; ++i)
        {
            EntityId eid;
            offset += copy_from_buffer(buf + offset, buf_end, eid);
            reg.assign<C>(eid);
        }

        offset += copy_from_buffer(buf + offset, buf_end, reg.raw<C>(), count);

        return offset;
    }

    template<class T>
    void push_tags_into_buffer(buffer& buf, const GridWorld::registry& reg)
    {
        push_array_into_buffer(buf, reg.data<T>(), reg.size<T>());
    }

    template<class T>
    size_t copy_tags_from_buffer(const char* buf, const char* buf_end, GridWorld::registry& reg)
    {
        size_t offset = 0;
        size_t count = 0;
        offset += copy_from_buffer(buf + offset, buf_end, count);

        for (int i = 0; i < count; ++i)
        {
            EntityId eid;
            offset += copy_from_buffer(buf + offset, buf_end, eid);
            reg.assign<T>(eid);
        }

        return offset;
    }

    template<class T>
    void push_into_buffer(buffer& buf, const T& obj)
    {
        static_assert(std::is_trivially_copyable_v<T>, "Non trivially copyable type. " __FUNCSIG__);
        push_into_buffer(buf, reinterpret_cast<const char*>(&obj), sizeof(T));
    }

    template<class T>
    size_t copy_from_buffer(const char* buf, const char* buf_end, T& obj)
    {
        static_assert(std::is_trivially_copyable_v<T>, "Non trivially copyable type. " __FUNCSIG__);
        return copy_from_buffer(buf, buf_end, reinterpret_cast<char*>(&obj), sizeof(T));
    }

    template<class T>
    void push_into_buffer(buffer& buf, const T* obj_array, size_t count)
    {
        if constexpr (std::is_trivially_copyable_v<T>)
        {
            push_into_buffer(buf, reinterpret_cast<const char*>(obj_array), count * sizeof(T));
        }
        else
        {
            for (int i = 0; i < count; ++i)
            {
                push_into_buffer(buf, obj_array[i]);
            }
        }
    }

    template<class T>
    size_t copy_from_buffer(const char* buf, const char* buf_end, T* obj_array, size_t count)
    {
        if constexpr (std::is_trivially_copyable_v<T>)
        {
            return copy_from_buffer(buf, buf_end, reinterpret_cast<char*>(obj_array), count * sizeof(T));
        }
        else
        {
            size_t offset = 0;
            for (int i = 0; i < count; ++i)
            {
                offset += copy_from_buffer(buf + offset, buf_end, obj_array[i]);
            }
            return offset;
        }
    }

    template<class T>
    void push_into_buffer(buffer& buf, const std::vector<T>& vec)
    {
        push_array_into_buffer(buf, vec.data(), vec.size());
    }

    template<class T>
    size_t copy_from_buffer(const char* buf, const char* buf_end, std::vector<T>& vec)
    {
        size_t offset = 0;
        size_t count = 0;
        offset += copy_from_buffer(buf + offset, buf_end, count);
        vec = std::vector<T>(count);
        offset += copy_from_buffer(buf + offset, buf_end, vec.data(), count);
        return offset;
    }

    template<class K, class V>
    void push_into_buffer(buffer& buf, const std::map<K, V>& map)
    {
        push_into_buffer(buf, map.size());
        for (auto const& [key, value] : map)
        {
            push_into_buffer(buf, key);
            push_into_buffer(buf, value);
        }
    }

    template<class K, class V>
    size_t copy_from_buffer(const char* buf, const char* buf_end, std::map<K, V>& map)
    {
        size_t offset = 0;
        size_t count = 0;
        offset += copy_from_buffer(buf + offset, buf_end, count);
        map.clear();
        for (int i = 0; i < count; ++i)
        {
            K key;
            offset += copy_from_buffer(buf + offset, buf_end, key);
            offset += copy_from_buffer(buf + offset, buf_end, map[key]);
        }
        return offset;
    }

    template<class T>
    void push_array_into_buffer(buffer& buf, const T* obj_array, size_t count)
    {
        push_into_buffer(buf, count);
        push_into_buffer(buf, obj_array, count);
    }
}

GridWorld::registry create_empty_simulation_registry()
{
    using namespace GridWorld::Component;
    GridWorld::registry reg;

    reg.ctx_or_set<SSimulationConfig>();
    reg.ctx_or_set<STickCounter>();
    reg.ctx_or_set<SWorld>();
    reg.ctx_or_set<SEventsLog>();
    reg.ctx_or_set<RNG>();

    return reg;
}

GridWorld::Simulation::Simulation()
{
    reg = create_empty_simulation_registry();
    stop_requested = false;
}

uint64_t GridWorld::Simulation::get_tick() const
{
    return reg.ctx<Component::STickCounter>().tick;
}

std::tuple<std::string, uint64_t> GridWorld::Simulation::get_state_json() const
{
    using namespace GridWorld::JSON;
    using namespace rapidjson;

    //shared_lock sl(simulation_mutex);
    shared_pause_lock pl(pause_requests, no_pauses_requested, simulation_mutex);

    StringBuffer buf;
    buf.Reserve(1024 * 100);
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

        writer.Key("SSimulationConfig");
        json_write(reg.ctx<SSimulationConfig>(), writer);

        writer.Key("STickCounter");
        json_write(reg.ctx<STickCounter>(), writer);

        writer.Key("SWorld");
        json_write(reg.ctx<SWorld>(), writer);

        writer.Key("SEventsLog");
        json_write(reg.ctx<SEventsLog>(), writer);

        writer.Key("RNG");
        json_write(reg.ctx<RNG>(), writer);

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

    return std::make_tuple(buf.GetString(), get_tick());
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

    // Check if sim is running at the start before parsing occurs,
    // since it is unlikely that the simulation will no longer be running
    // when the parsing is finished.
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
        if (singletons.HasMember("SSimulationConfig"))
        {
            json_read(tmp.ctx<SSimulationConfig>(), singletons["SSimulationConfig"]);
        }

        json_read(tmp.ctx<STickCounter>(), singletons["STickCounter"]);

        json_read(tmp.ctx<SWorld>(), singletons["SWorld"]);

        json_read(tmp.ctx<SEventsLog>(), singletons["SEventsLog"]);

        if (singletons.HasMember("RNG"))
        {
            json_read(tmp.ctx<RNG>(), singletons["RNG"]);
        }
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

    // Do the proper write mutex/running check here, 
    // after the parsed registry is ready to be copied in.
    unique_lock ul(simulation_mutex);

    // check if the sim is running again, on the off chance
    // that its running state changed while the
    // json document was being parsed.
    if (is_running())
    {
        throw std::exception("set_state_json cannot be used while simulation is running.");
    }

    reg = std::move(tmp);
}

uint64_t GridWorld::Simulation::create_entity()
{
    unique_lock ul(simulation_mutex);

    if (is_running())
    {
        throw std::exception("create_entity cannot be used while simulation is running.");
    }

    return to_integral(reg.create());
}

void GridWorld::Simulation::destroy_entity(uint64_t eid)
{
    unique_lock ul(simulation_mutex);

    if (is_running())
    {
        throw std::exception("destroy_entity cannot be used while simulation is running.");
    }

    reg.destroy(EntityId{ eid });
}

std::tuple<std::vector<uint64_t>, uint64_t> GridWorld::Simulation::get_all_entities() const
{
    //shared_lock sl(simulation_mutex);
    shared_pause_lock pl(pause_requests, no_pauses_requested, simulation_mutex);

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

    return std::make_tuple(result, get_tick());
}

void GridWorld::Simulation::start_simulation()
{
    std::lock_guard control_guard(control_mutex);
    if (!is_running())
    {
        // Since the state may have been changed externally while the simulation
        // wasn't running, ensure any hidden state is properly synced up
        Systems::Util::rebuild_world(reg);

        stop_requested = false;
        simulation_thread = std::thread(&Simulation::simulation_loop, this);
    }
}

void GridWorld::Simulation::stop_simulation()
{
    std::lock_guard control_guard(control_mutex);
    if (is_running())
    {
        stop_requested = true;
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

    EntityId eid = EntityId(eid_int);

    unique_lock ul(simulation_mutex);

    if (is_running())
    {
        throw std::exception("assign_component cannot be used while simulation is running.");
    }

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

std::tuple<std::string, uint64_t> GridWorld::Simulation::get_component_json(uint64_t eid_int, std::string component_name) const
{
    using namespace Component;
    using namespace Reflect;
    using namespace GridWorld::JSON;
    using namespace rapidjson;

    EntityId eid = EntityId(eid_int);
    StringBuffer buf;
    Writer<StringBuffer> writer(buf);

    //shared_lock sl(simulation_mutex);
    shared_pause_lock pl(pause_requests, no_pauses_requested, simulation_mutex);

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

    return std::make_tuple(buf.GetString(), get_tick());
}

void GridWorld::Simulation::remove_component(uint64_t eid_int, std::string component_name)
{
    using namespace Component;
    using namespace Reflect;

    EntityId eid = EntityId(eid_int);

    unique_lock ul(simulation_mutex);

    if (is_running())
    {
        throw std::exception("remove_component cannot be used while simulation is running.");
    }

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

    EntityId eid = EntityId(eid_int);

    unique_lock ul(simulation_mutex);

    if (is_running())
    {
        throw std::exception("replace_component cannot be used while simulation is running.");
    }

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

std::tuple<std::vector<std::string>, uint64_t> GridWorld::Simulation::get_entity_component_names(uint64_t eid) const
{
    using namespace Reflect;
    std::vector<std::string> result;

    //shared_lock sl(simulation_mutex);
    shared_pause_lock pl(pause_requests, no_pauses_requested, simulation_mutex);

    reg.visit(EntityId(eid), [&result](ENTT_ID_TYPE com_id)
    {
        result.push_back(id_to_com_name(com_id));
    });
    return std::make_tuple(result, get_tick());
}

std::tuple<std::string, uint64_t> GridWorld::Simulation::get_singleton_json(std::string singleton_name) const
{
    using namespace Component;
    using namespace Reflect;
    using namespace GridWorld::JSON;
    using namespace rapidjson;

    StringBuffer buf;
    Writer<StringBuffer> writer(buf);

    //shared_lock sl(simulation_mutex);
    shared_pause_lock pl(pause_requests, no_pauses_requested, simulation_mutex);

    if (singleton_name == com_name<SWorld>())
    {
        json_write(reg.ctx<SWorld>(), writer);
    }
    else if (singleton_name == com_name<SEventsLog>())
    {
        json_write(reg.ctx<SEventsLog>(), writer);
    }
    else if (singleton_name == com_name<SSimulationConfig>())
    {
        json_write(reg.ctx<SSimulationConfig>(), writer);
    }
    else if (singleton_name == com_name<RNG>())
    {
        json_write(reg.ctx<RNG>(), writer);
    }
    else
    {
        throw std::exception(("Unknown component type passed to get_singleton_json: " + singleton_name).c_str());
    }

    return std::make_tuple(buf.GetString(), get_tick());
}

void GridWorld::Simulation::set_singleton_json(std::string singleton_name, std::string singleton_json)
{
    using namespace Component;
    using namespace Reflect;

    if (is_running())
    {
        throw std::exception("set_singleton_json cannot be used while simulation is running.");
    }

    unique_lock ul(simulation_mutex);

    if (singleton_name == com_name<SWorld>())
    {
        JSON::json_read(reg.ctx<SWorld>(), singleton_json);
    }
    else if (singleton_name == com_name<SEventsLog>())
    {
        JSON::json_read(reg.ctx<SEventsLog>(), singleton_json);
    }
    else if (singleton_name == com_name<SSimulationConfig>())
    {
        JSON::json_read(reg.ctx<SSimulationConfig>(), singleton_json);
    }
    else if (singleton_name == com_name<RNG>())
    {
        JSON::json_read(reg.ctx<RNG>(), singleton_json);
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
        com_name<SSimulationConfig>(),
        com_name<SWorld>(),
        com_name<SEventsLog>(),
        com_name<RNG>()
    };
}

void GridWorld::Simulation::set_tick_event_callback(tick_event_callback_function callback)
{
    unique_lock ul(simulation_mutex);
    tick_event_callback = callback;
}

std::tuple<std::vector<char>, uint64_t> GridWorld::Simulation::get_state_binary() const
{
    using namespace GridWorld::Component;
    using namespace GridWorld::Binary;

    buffer buf;
    buf.reserve(1024 * 30);

    //shared_lock sl(simulation_mutex);
    shared_pause_lock pl(pause_requests, no_pauses_requested, simulation_mutex);

    push_array_into_buffer(buf, reg.data(), reg.size());

    push_singleton_into_buffer<SSimulationConfig>(buf, reg);
    push_singleton_into_buffer<STickCounter>(buf, reg);
    push_singleton_into_buffer<SWorld>(buf, reg);
    push_singleton_into_buffer<SEventsLog>(buf, reg);
    push_singleton_into_buffer<RNG>(buf, reg);

    push_components_into_buffer<Position>(buf, reg);
    push_components_into_buffer<Moveable>(buf, reg);
    push_components_into_buffer<Name>(buf, reg);
    push_components_into_buffer<RNG>(buf, reg);
    push_components_into_buffer<SimpleBrain>(buf, reg);
    push_components_into_buffer<SimpleBrainSeer>(buf, reg);
    push_components_into_buffer<SimpleBrainMover>(buf, reg);
    push_components_into_buffer<Predation>(buf, reg);
    push_components_into_buffer<Scorable>(buf, reg);

    push_tags_into_buffer<RandomMover>(buf, reg);

    return std::make_tuple(buf, get_tick());
}

void GridWorld::Simulation::set_state_binary(const char* bin, size_t size)
{
    //using namespace GridWorld::JSON;
    //using namespace rapidjson;
    using namespace GridWorld::Binary;

    unique_lock ul(simulation_mutex);

    if (is_running())
    {
        throw std::exception("set_state_binary cannot be used while simulation is running.");
    }

    const char* bin_end = bin + size;
    size_t offset = 0;

    registry tmp = create_empty_simulation_registry();

    {
        std::vector<EntityId> eids;
        offset += copy_from_buffer(bin + offset, bin_end, eids);

        tmp.assign(eids.begin(), eids.end());
    }

    {
        offset += copy_singleton_from_buffer<SSimulationConfig>(bin + offset, bin_end, tmp);
        offset += copy_singleton_from_buffer<STickCounter>(bin + offset, bin_end, tmp);
        offset += copy_singleton_from_buffer<SWorld>(bin + offset, bin_end, tmp);
        offset += copy_singleton_from_buffer<SEventsLog>(bin + offset, bin_end, tmp);
        offset += copy_singleton_from_buffer<RNG>(bin + offset, bin_end, tmp);
    }

    {
        offset += copy_components_from_buffer<Position>(bin + offset, bin_end, tmp);
        offset += copy_components_from_buffer<Moveable>(bin + offset, bin_end, tmp);
        offset += copy_components_from_buffer<Name>(bin + offset, bin_end, tmp);
        offset += copy_components_from_buffer<RNG>(bin + offset, bin_end, tmp);
        offset += copy_components_from_buffer<SimpleBrain>(bin + offset, bin_end, tmp);
        offset += copy_components_from_buffer<SimpleBrainSeer>(bin + offset, bin_end, tmp);
        offset += copy_components_from_buffer<SimpleBrainMover>(bin + offset, bin_end, tmp);
        offset += copy_components_from_buffer<Predation>(bin + offset, bin_end, tmp);
        offset += copy_components_from_buffer<Scorable>(bin + offset, bin_end, tmp);

        offset += copy_tags_from_buffer<RandomMover>(bin + offset, bin_end, tmp);
    }

    reg = std::move(tmp);
}

uint64_t GridWorld::Simulation::get_events_last_tick(event_callback_function callback)
{
    using namespace GridWorld::JSON;
    using namespace rapidjson;

    StringBuffer buf;
    Writer<StringBuffer> writer(buf);

    //shared_lock sl(simulation_mutex);
    shared_pause_lock pl(pause_requests, no_pauses_requested, simulation_mutex);

    for (const Events::Event& e : reg.ctx<Component::SEventsLog>().events_last_tick)
    {
        json_write(e.data, writer);
        callback(e.name.c_str(), buf.GetString());
        buf.Clear();
    }

    return get_tick();
}

void GridWorld::Simulation::run_command(int64_t argc, const char* argv[], command_result_callback_function callback)
{
    using namespace GridWorld::Component;

    try
    {
        if (argc <= 0)
        {
            throw std::exception("No command specified.");
        }

        std::vector<std::string_view> args;
        args.reserve(argc);

        for (int i = 0; i < argc; ++i)
        {
            args.push_back(std::string_view(argv[i]));
        }

        auto command = args[0];

        if (command == "randomize")
        {
            unique_lock ul(simulation_mutex);

            if (is_running())
            {
                throw std::exception("Command 'randomize' cannot be used while simulation is running.");
            }

            if (argc == 1)
            {
                // no other args, randomize all RNG components + singleton
                auto rng_view = reg.view<RNG>();
                for (EntityId eid : rng_view)
                {
                    RNG& rng = rng_view.get(eid);
                    rng.seed(pcg_extras::seed_seq_from<std::random_device>());
                }

                RNG& srng = reg.ctx<RNG>();
                srng.seed(pcg_extras::seed_seq_from<std::random_device>());
            }
            else if (argc == 2)
            {
                auto eid_view = args[1];
                uint64_t raw_eid = 0;
                auto [p, ec] = std::from_chars(eid_view.data(), eid_view.data() + eid_view.size(), raw_eid);
                EntityId eid = (EntityId)raw_eid;
                if (ec != std::errc())
                {
                    throw std::exception("Provided EID does not have a valid format.");
                }

                RNG& rng = reg.get<RNG>(eid);
                rng.seed(pcg_extras::seed_seq_from<std::random_device>());
            }
            else
            {
                throw std::exception("Command 'randomize' can only accept up to 1 arguments.");
            }
        }
        else
        {
            throw std::exception("Unknown sim command provided.");
        }
    }
    catch (const std::exception& e)
    {
        callback(e.what(), nullptr);
    }

}

void GridWorld::Simulation::request_stop()
{
    stop_requested = true;
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
    unique_lock ul(simulation_mutex);
    while (!stop_requested)
    {
        // For the update, aquire an exclusive lock to prevent reads during the sim update.
        // However, after the write is done, we do not need to (and should not) keep a
        // shared lock afterwards, because the tick event handler might need to make its
        // own reads, and shared mutexes do not allow recursion, so it would likely 
        // cause deadlocks.

        // This is safe only as long as ALL methods that require write access only allow
        // a write to continue when the simulation loop is not running. That way, as long as
        // a thread is inside this method, no writes will succeed, and we have a de-facto
        // read safe state inside this method without a shared lock.

        while (pause_requests != 0)
        {
            no_pauses_requested.wait(simulation_mutex);
        }

        update_tick(reg);

        if (tick_event_callback != nullptr)
        {
            ul.unlock();
            const auto& events_last_tick = reg.ctx<Component::SEventsLog>().events_last_tick;
            uint64_t flags =
                // BIT 0: 1 if events have occured last tick, 0 otherwise
                1 * (events_last_tick.size() > 0);

            tick_event_callback(get_tick(), flags);
            ul.lock();
        }
    }
}
