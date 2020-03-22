#include "stdafx.h"
#include "components.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

PYBIND11_MAKE_OPAQUE(std::vector<GridWorld::Component::SynapseMat>)
PYBIND11_MAKE_OPAQUE(std::vector<GridWorld::Component::NeuronMat>)

namespace py = pybind11;
using namespace GridWorld;

void setup_components_meta()
{
    entt::meta<Component::Moveable>().alias("Moveable"_hs);
    entt::meta<Component::Name>().alias("Name"_hs);
    entt::meta<Component::Position>().alias("Position"_hs);
    entt::meta<Component::Predation>().alias("Predation"_hs);
    entt::meta<Component::PyMeta>().alias("PyMeta"_hs);
    entt::meta<Component::RandomMover>().alias("RandomMover"_hs);
    entt::meta<Component::RNG>().alias("RNG"_hs);
    entt::meta<Component::Scorable>().alias("Scorable"_hs);
    entt::meta<Component::SimpleBrain>().alias("SimpleBrain"_hs);
    entt::meta<Component::SimpleBrainMover>().alias("SimpleBrainMover"_hs);
    entt::meta<Component::SimpleBrainSeer>().alias("SimpleBrainSeer"_hs);
    entt::meta<Component::SNewEntityQueue>().alias("SNewEntityQueue"_hs);
    entt::meta<Component::SWorld>().alias("SWorld"_hs);
}

void bind_components_to_python_module(py::module& m)
{
    py::bind_vector<vector<GridWorld::Component::SynapseMat>>(m, "VectorSynapseMat", py::module_local(false));
    py::bind_vector<vector<GridWorld::Component::NeuronMat>>(m, "VectorNeuronMat", py::module_local(false));

    py::class_<Component::Position>(m, "Position")
        .def_readwrite("x", &Component::Position::x)
        .def_readwrite("y", &Component::Position::y)
        ;
    py::class_<Component::Moveable>(m, "Moveable")
        .def_readwrite("x_force", &Component::Moveable::x_force)
        .def_readwrite("y_force", &Component::Moveable::x_force)
        ;
    py::class_<Component::Name>(m, "Name")
        .def_readwrite("major_name", &Component::Name::major_name)
        .def_readwrite("minor_name", &Component::Name::minor_name)
        ;

    auto get_rng_state = [](const Component::RNG& rng)
    {
        std::stringstream ss;
        ss << rng;
        return ss.str();
    };

    auto load_rng_state = [](Component::RNG& rng, std::string& state)
    {
        std::stringstream ss;
        ss << state;
        ss >> rng;
    };

    auto randi = [](Component::RNG& rng)
    {
        return rng();
    };

    // Simple function to return a random double [0, 1)
    auto randd = [](Component::RNG& rng)
    {
        return ldexp(rng(), -32);
    };

    py::class_<Component::RNG>(m, "RNG")
        .def(py::init<>())
        .def("get_state", get_rng_state)
        .def("set_state", load_rng_state)
        .def("seed", &Component::RNG::seed<uint64_t&, uint64_t&>)
        .def("randi", randi)
        .def("randd", randd)
        ;

    py::class_<Component::Scorable>(m, "Scorable")
        .def_readwrite("score", &Component::Scorable::score)
        ;

    py::class_<Component::RandomMover>(m, "RandomMover")
        ;

    py::class_<Component::SimpleBrain>(m, "SimpleBrain")
        .def_readwrite("synapses", &Component::SimpleBrain::synapses)
        .def_readwrite("neurons", &Component::SimpleBrain::neurons)
        .def_readwrite("child_mutation_chance", &Component::SimpleBrain::child_mutation_chance)
        .def_readwrite("child_mutation_strength", &Component::SimpleBrain::child_mutation_strength)
        ;

    py::class_<Component::SimpleBrainSeer>(m, "SimpleBrainSeer")
        .def_readwrite("neuron_offset", &Component::SimpleBrainSeer::neuron_offset)
        .def_readwrite("sight_radius", &Component::SimpleBrainSeer::sight_radius)
        ;

    py::class_<Component::SimpleBrainMover>(m, "SimpleBrainMover")
        .def_readwrite("neuron_offset", &Component::SimpleBrainMover::neuron_offset)
        ;

    py::class_<Component::Predation>(m, "Predation")
        .def_readwrite("no_predation_until_tick", &Component::Predation::no_predation_until_tick)
        .def_readwrite("ticks_between_predations", &Component::Predation::ticks_between_predations)
        .def_readwrite("predate_all", &Component::Predation::predate_all)
        ;

    py::class_<Component::SWorld>(m, "SWorld")
        .def_readonly("width", &Component::SWorld::width)
        .def_readonly("height", &Component::SWorld::height)
        .def("get_map_data", &Component::SWorld::get_map_data)
        .def("set_map_data", &Component::SWorld::set_map_data)
        .def("reset_world", py::overload_cast<int, int>(&Component::SWorld::reset_world))
        ;
}

#define GRIDWORLD_EM_COMPONENT_FUNCTIONS(com) \
        .def("get_" #com, &EntityManager::get_components<Component::com>, py::return_value_policy::reference_internal)                  \
        .def("has_" #com, &EntityManager::has_components<Component::com>)                                                               \
        .def("assign_or_replace_" #com, &EntityManager::assign_or_replace<Component::com>, py::return_value_policy::reference_internal) \
        .def("remove_" #com, &EntityManager::remove<Component::com>)
#define GRIDWORLD_EM_SINGLETON_COMPONENT_FUNCTIONS(scom) \
        .def("get_singleton_" #scom, &EntityManager::get_singleton<Component::scom>, py::return_value_policy::reference_internal)                              \
        .def("has_singleton_" #scom, &EntityManager::has_singleton<Component::scom>)                                                                           \
        .def("set_singleton_" #scom, &EntityManager::set_singleton<Component::scom>, py::return_value_policy::reference_internal)   \
        .def("unset_singleton_" #scom, &EntityManager::unset_singleton<Component::scom>)
#define GRIDWORLD_EM_TAG_FUNCTIONS(tag) \
        .def("has_" #tag, &EntityManager::has_components<Component::tag>)                                                               \
        .def("assign_or_replace_" #tag, &EntityManager::assign_or_replace<Component::tag>, py::return_value_policy::reference_internal) \
        .def("remove_" #tag, &EntityManager::remove<Component::tag>)

void bind_components_to_entity_manager(py::class_<EntityManager>& c)
{
    c
        GRIDWORLD_EM_COMPONENT_FUNCTIONS(Moveable)
        GRIDWORLD_EM_COMPONENT_FUNCTIONS(Position)
        GRIDWORLD_EM_COMPONENT_FUNCTIONS(Name)
        GRIDWORLD_EM_COMPONENT_FUNCTIONS(RNG)
        GRIDWORLD_EM_COMPONENT_FUNCTIONS(Scorable)
        GRIDWORLD_EM_COMPONENT_FUNCTIONS(SimpleBrain)
        GRIDWORLD_EM_COMPONENT_FUNCTIONS(SimpleBrainSeer)
        GRIDWORLD_EM_COMPONENT_FUNCTIONS(SimpleBrainSeer)
        GRIDWORLD_EM_COMPONENT_FUNCTIONS(SimpleBrainMover)
        GRIDWORLD_EM_COMPONENT_FUNCTIONS(Predation)
        GRIDWORLD_EM_COMPONENT_FUNCTIONS(PyMeta)
        GRIDWORLD_EM_SINGLETON_COMPONENT_FUNCTIONS(SWorld)
        GRIDWORLD_EM_SINGLETON_COMPONENT_FUNCTIONS(RNG)
        GRIDWORLD_EM_TAG_FUNCTIONS(RandomMover)
        ;
}
