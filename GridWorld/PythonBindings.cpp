#include "stdafx.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

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
        .def("get_state_json", &Simulation::get_state_json)
        .def("set_state_json", &Simulation::set_state_json)
        .def("create_entity", &Simulation::create_entity)
        .def("destroy_entity", &Simulation::destroy_entity)
        .def("get_all_entities", &Simulation::get_all_entities)
        ;

    m.attr("null") = to_integral((EntityId)entt::null);
}
