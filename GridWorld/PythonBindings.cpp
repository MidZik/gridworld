#include "stdafx.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/functional.h>

#include <entt/entity/entity.hpp>

#include "Registry.h"
#include "Simulation.h"

namespace py = pybind11;

// The simulation manager expects all simulations to export a "simulation" module,
// with a defined API.
PYBIND11_MODULE(simulation, m)
{
    using namespace py::literals;
    using namespace GridWorld;

    m.doc() = "GridWorld module.";

    auto simulation_class = py::class_<Simulation>(m, "Simulation")
        .def(py::init<>())
        .def("get_tick", &Simulation::get_tick)
        .def("get_state_json", &Simulation::get_state_json)
        .def("set_state_json", &Simulation::set_state_json)
        .def("create_entity", &Simulation::create_entity)
        .def("destroy_entity", &Simulation::destroy_entity)
        .def("get_all_entities", &Simulation::get_all_entities)
        .def("start_simulation", &Simulation::start_simulation)
        .def("stop_simulation", &Simulation::stop_simulation)
        .def("is_running", &Simulation::is_running)
        .def("assign_component", &Simulation::assign_component)
        .def("remove_component", &Simulation::remove_component)
        .def("replace_component", &Simulation::replace_component)
        .def("get_component_names", &Simulation::get_component_names)
        .def("set_event_callback", &Simulation::set_event_callback)
        ;

    m.attr("null") = to_integral((EntityId)entt::null);
}
