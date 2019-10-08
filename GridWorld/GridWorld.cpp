// GridWorld.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "GridWorld.h"
#include "components.h"
#include <iostream>
#include <set>
#include <algorithm>
#include <queue>

#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

using Eigen::MatrixXd;
using namespace std;

namespace GridWorld
{
    namespace System
    {
        using namespace Component;

        //// MOVEMENT

        /*
        Returns -1 if x is negative, 0 if x is 0, 1 if x is positive.
        */
        int sign(int x)
        {
            return (x > 0) - (x < 0);
        }

        struct _MovementInfo
        {
            int map_index = -1;
            std::vector<_MovementInfo*> child_nodes;
            _MovementInfo* parent_node = NULL;
            bool is_entry_node = false;
            EntityId eid = entt::null;
            Position* entity_position = NULL;
            int net_force = 0;
            bool finalized = false;
            _MovementInfo* accepted_child = NULL;
        };

        using movement_node_map = std::unordered_map<int, _MovementInfo>;
        using movement_node_set = std::set<_MovementInfo*>;

        thread_local movement_node_map movement_nodes; // Declared globally to keep in memory
        thread_local movement_node_set movement_entry_nodes;

        void _add_movement_info(EntityId eid, SWorld& world, Moveable& moveable, Position& position)
        {
            int abs_x_force = abs(moveable.x_force);
            int abs_y_force = abs(moveable.y_force);

            if (abs_x_force - abs_y_force == 0)
            {
                return;
            }

            int cancellation = min(abs_x_force, abs_y_force);

            int true_x_force = (abs_x_force - cancellation) * sign(moveable.x_force);
            int true_y_force = (abs_y_force - cancellation) * sign(moveable.y_force);

            int new_x = position.x;
            int new_y = position.y;
            int net_force = 0;
            if (true_x_force > 0)
            {
                new_x += 1;
                net_force = true_x_force;
            }
            else if (true_x_force < 0)
            {
                new_x -= 1;
                net_force = -true_x_force;
            }
            else if (true_y_force > 0)
            {
                new_y += 1;
                net_force = true_y_force;
            }
            else
            {
                new_y -= 1;
                net_force = -true_y_force;
            }

            int cur_map_index = world.get_map_index(position.x, position.y);
            int new_map_index = world.get_map_index(new_x, new_y);

            _MovementInfo* cur_movement_info = NULL;
            auto cur_iter = movement_nodes.find(cur_map_index);
            if (cur_iter != movement_nodes.end())
            {
                cur_movement_info = &cur_iter->second;
            }
            else
            {
                _MovementInfo temp;
                temp.map_index = cur_map_index;
                temp.eid = eid;
                cur_movement_info = &movement_nodes.emplace(cur_map_index, temp).first->second;
            }

            _MovementInfo* new_movement_info = NULL;
            auto new_iter = movement_nodes.find(new_map_index);
            if (new_iter != movement_nodes.end())
            {
                new_movement_info = &new_iter->second;
            }
            else
            {
                _MovementInfo temp;
                temp.map_index = new_map_index;
                temp.eid = world.map[new_map_index];
                new_movement_info = &movement_nodes.emplace(new_map_index, temp).first->second;
                movement_entry_nodes.insert(new_movement_info);
                new_movement_info->is_entry_node = true;
            }

            cur_movement_info->net_force = net_force;
            cur_movement_info->entity_position = &position;

            if (cur_movement_info->parent_node != new_movement_info)
            {
                // erase self from old parent if we had one
                if (cur_movement_info->parent_node != NULL)
                {
                    auto& parent_children = cur_movement_info->parent_node->child_nodes;
                    parent_children.erase(remove(parent_children.begin(), parent_children.end(), cur_movement_info));
                }

                cur_movement_info->parent_node = new_movement_info;
                new_movement_info->child_nodes.push_back(cur_movement_info);
            }

            _MovementInfo* search_node = cur_movement_info->parent_node;

            // Verify that our graph has just one entry node by searching for an entry node among our parents.
            while (!search_node->is_entry_node && search_node != cur_movement_info)
            {
                search_node = search_node->parent_node;
            }

            if (search_node != cur_movement_info && cur_movement_info->is_entry_node)
            {
                // Found an entry node that is not ourselves, and we used to be an entry node,
                // so we remove ourselves as an entry node
                cur_movement_info->is_entry_node = false;
                movement_entry_nodes.erase(cur_movement_info);
            }
            else if (search_node == cur_movement_info && !cur_movement_info->is_entry_node)
            {
                // We did not find an entry node among our parents and arrived back to ourselves.
                // That means there is no entry node in our loop, so make ourselves an entry node.
                cur_movement_info->is_entry_node = true;
                movement_entry_nodes.insert(cur_movement_info);
            }
        }

        void _traverse_and_resolve_movement(_MovementInfo& entry_node)
        {
            assert(entry_node.is_entry_node);

            queue<_MovementInfo*> traversal_queue;

            /*
            Special handling for the entry node.
            If entry node has a parent, that indicates a cycle. Let all nodes in the cycle move.
            If the entry node has an entity but has no parent, it is not moving. Reject all children.
            Otherwise, the node is empty and should accept the child with highest force (or no child if a tie exists).
            */

            if (entry_node.parent_node != NULL)
            {
                // cycle case, the entry node has a parent it wants to move to.
                auto* previous_cycle_node = &entry_node;
                auto* current_cycle_node = entry_node.parent_node;

                while (!current_cycle_node->finalized)
                {
                    current_cycle_node->accepted_child = previous_cycle_node;
                    current_cycle_node->finalized = true;

                    for (auto* child : current_cycle_node->child_nodes)
                    {
                        if (child != previous_cycle_node)
                        {
                            traversal_queue.push(child);
                        }
                    }

                    previous_cycle_node = current_cycle_node;
                    current_cycle_node = current_cycle_node->parent_node;
                }
            }
            else if (entry_node.eid != entt::null)
            {
                // reject children case (entity exists and is not moving)
                entry_node.accepted_child = NULL;
                entry_node.finalized = true;

                for (auto* child : entry_node.child_nodes)
                {
                    traversal_queue.push(child);
                }
            }
            else
            {
                // Accept most forceful child case.
                int highest_force = -1;
                _MovementInfo* highest_child = NULL;

                for (auto* child : entry_node.child_nodes)
                {
                    if (child->net_force > highest_force)
                    {
                        highest_child = child;
                        highest_force = child->net_force;
                    }
                    else if (child->net_force == highest_force)
                    {
                        highest_child = NULL;
                    }

                    traversal_queue.push(child);
                }

                entry_node.accepted_child = highest_child;
                entry_node.finalized = true;
            }

            /*
            Normal handling for non-cycle, non-entry nodes.
            If the parent node accepted me, accept the child with the highest force (If tied, accept none).
            If the parent node rejected me, reject all children.
            */

            while (!traversal_queue.empty())
            {
                auto* cur_node = traversal_queue.front();
                traversal_queue.pop();

                assert(!cur_node->finalized);

                if (cur_node->parent_node->accepted_child == cur_node)
                {
                    // Accept most forceful child case.
                    int highest_force = -1;
                    _MovementInfo* highest_child = NULL;

                    for (auto* child : cur_node->child_nodes)
                    {
                        if (child->net_force > highest_force)
                        {
                            highest_child = child;
                            highest_force = child->net_force;
                        }
                        else if (child->net_force == highest_force)
                        {
                            highest_child = NULL;
                        }

                        traversal_queue.push(child);
                    }

                    cur_node->accepted_child = highest_child;
                    cur_node->finalized = true;
                }
                else
                {
                    cur_node->accepted_child = NULL;
                    cur_node->finalized = true;

                    for (auto* child : cur_node->child_nodes)
                    {
                        traversal_queue.push(child);
                    }
                }
            }
        }

        void _traverse_and_execute_movement(SWorld& world, _MovementInfo& entry_node)
        {
            assert(entry_node.is_entry_node);

            auto* cur_node = &entry_node;
            int cur_map_index = cur_node->map_index;

            while (cur_node->accepted_child != NULL && world.map[cur_map_index] != cur_node->accepted_child->eid)
            {
                world.map[cur_map_index] = cur_node->accepted_child->eid;
                cur_node->accepted_child->entity_position->x = world.get_map_index_x(cur_map_index);
                cur_node->accepted_child->entity_position->y = world.get_map_index_y(cur_map_index);

                cur_node = cur_node->accepted_child;
                cur_map_index = cur_node->map_index;
            }

            // special case: if any nodes were moved, we need to make sure the last node of the tree clears out its position if necessary 
            // (since it wasn't iterated over)
            if (cur_node->accepted_child == NULL && cur_node != &entry_node)
            {
                world.map[cur_map_index] = entt::null;
            }
        }

        void movement(EntityManager& em)
        {
            auto& world = em.get_singleton<SWorld>();

            auto view = em.reg.view<Moveable, Position>();

            view.each([&world](EntityId eid, Moveable& moveable, Position& position)
            {
                _add_movement_info(eid, world, moveable, position);

                moveable.x_force = 0;
                moveable.y_force = 0;
            });

            for (auto entry_node : movement_entry_nodes)
            {
                _traverse_and_resolve_movement(*entry_node);
            }

            for (auto entry_node : movement_entry_nodes)
            {
                _traverse_and_execute_movement(world, *entry_node);
            }

            movement_nodes.clear();
            movement_entry_nodes.clear();
        }


        //// SIMPLE BRAIN CALC

        void _relu(NeuronMat& mat)
        {
            auto data = mat.data();
            for (Eigen::Index i = 0, size = mat.size(); i < size; i++)
            {
                float& element = *(data + i);
                if (element < 0)
                {
                    element = 0;
                }
            }
        }

        void simple_brain_calc(EntityManager& em)
        {
            auto brain_view = em.reg.view<SimpleBrain>();
            for (EntityId eid : brain_view)
            {
                auto& brain = brain_view.get(eid);
                
                for (auto i = 0; i < brain.synapses.size(); i++)
                {
                    bool has_bias = !(i == brain.synapses.size() - 1);
                    SynapseMat& synapse_mat = brain.synapses[i];
                    NeuronMat& input = brain.neurons[i];
                    NeuronMat& output = brain.neurons[i + 1];

                    // relu first
                    _relu(input);

                    if (has_bias)
                    {
                        output.rightCols(output.cols() - 1) = input * synapse_mat;
                    }
                    else
                    {
                        output = input * synapse_mat;
                    }
                }

                // finally, for now we also apply relu to the final output
                _relu(brain.neurons.back());
            }
        }

        //// RANDOM MOVEMENT

        void random_movement(EntityManager& em)
        {
            auto random_mover_view = em.reg.view<RandomMover, Moveable, RNG>();

            random_mover_view.each([](EntityId eid, RandomMover, Moveable& moveable, RNG& rng)
            {
                if (rng() % 2 == 0)
                {
                    moveable.y_force += rng() % 7 - 3;
                }
                else
                {
                    moveable.x_force += rng() % 7 - 3;
                }
            });
        }

        //// PREDATION
        struct map_lookup_result
        {
            int x_offset = 0;
            int y_offset = 0;
            EntityId eid = entt::null;
        };
        void _get_entities_in_radius(SWorld& world, int x, int y, int radius, std::vector<map_lookup_result>& result)
        {
            result.clear();

            for (int cur_y_offset = -radius; cur_y_offset <= radius; cur_y_offset++)
            {
                int cur_x_radius = radius - abs(cur_y_offset);
                for (int cur_x_offset = -cur_x_radius; cur_x_offset <= cur_x_radius; cur_x_offset++)
                {
                    auto map_data = world.get_map_data(x + cur_x_offset, y + cur_y_offset);
                    if (map_data != entt::null)
                    {
                        result.push_back({ cur_x_offset, cur_y_offset, map_data });
                    }
                }
            }
        }

        void predation(EntityManager& em)
        {
            SWorld& world = em.get_singleton<SWorld>();

            auto predator_view = em.reg.view<Predation, Position, RNG>();
            auto scorable_view = em.reg.view<Scorable>();

            predator_view.each([&em, &world, scorable_view](EntityId eid, Predation& predation, Position& position, RNG& rng)
            {
                if (em.tick < predation.no_predation_until_tick)
                {
                    return;
                }

                std::vector<Scorable*> scorables_found;
                std::vector<map_lookup_result> nearby_entities;

                _get_entities_in_radius(world, position.x, position.y, 1, nearby_entities);

                for (auto result : nearby_entities)
                {
                    if (scorable_view.contains(result.eid))
                    {
                        scorables_found.push_back(&scorable_view.get(result.eid));
                    }
                }

                auto scorables_found_size = scorables_found.size();
                if (scorables_found_size > 0)
                {
                    // At least one scorable has been found, reduce a random scorable's score
                    int random_index = rng() % scorables_found_size;
                    auto& scorable = *scorables_found[random_index];
                    scorable.score -= 1;
                    predation.no_predation_until_tick = em.tick + 5;
                }
            });
        }

        //// SIMPLE BRAIN SEER SYSTEM
        void _get_map_data_in_radius(SWorld& world, int x, int y, int radius, std::vector<map_lookup_result>& result)
        {
            result.clear();

            for (int cur_y_offset = -radius; cur_y_offset <= radius; cur_y_offset++)
            {
                int cur_x_radius = radius - abs(cur_y_offset);
                for (int cur_x_offset = -cur_x_radius; cur_x_offset <= cur_x_radius; cur_x_offset++)
                {
                    auto map_data = world.get_map_data(x + cur_x_offset, y + cur_y_offset);
                    result.push_back({ cur_x_offset, cur_y_offset, map_data });
                }
            }
        }
        void simple_brain_seer(EntityManager& em)
        {
            SWorld& world = em.get_singleton<SWorld>();

            auto simple_brain_view = em.reg.view<SimpleBrain, SimpleBrainSeer, Position>();
            auto predator_view = em.reg.view<Predation>();

            simple_brain_view.each([&em, &world, predator_view](SimpleBrain& brain, SimpleBrainSeer& seer, Position& position) {
                NeuronMat& input_neurons = brain.neurons[0];

                int cur_neuron_offset = seer.neuron_offset;
                std::vector<map_lookup_result> map_data;

                _get_map_data_in_radius(world, position.x, position.y, seer.sight_radius, map_data);

                for (auto result : map_data)
                {
                    if (result.eid == entt::null)
                    {
                        // nothing seen
                        input_neurons(cur_neuron_offset) = 0;
                        input_neurons(cur_neuron_offset + 1) = 0;
                    }
                    else if (predator_view.contains(result.eid))
                    {
                        // predator seen
                        input_neurons(cur_neuron_offset) = 1;
                        input_neurons(cur_neuron_offset + 1) = 0;
                    }
                    else
                    {
                        // non-predator seen
                        input_neurons(cur_neuron_offset) = 0;
                        input_neurons(cur_neuron_offset + 1) = 1;
                    }
                    cur_neuron_offset += 2; // iterate in sets of 2 (predator neuron + nonpredator neuron)
                }
            });
        }

        //// SIMPLE BRAIN MOVER SYSTEM
        void simple_brain_mover(EntityManager& em)
        {
            auto simple_brain_view = em.reg.view<SimpleBrain, SimpleBrainMover, Moveable>();

            simple_brain_view.each([&em](SimpleBrain& brain, SimpleBrainMover& mover, Moveable& moveable) {
                int neuron_offset = mover.neuron_offset;
                auto& output_neurons = brain.neurons.back();

                moveable.x_force += 4 * int(output_neurons(0, neuron_offset));
                moveable.x_force -= 4 * int(output_neurons(0, neuron_offset + 1));
                moveable.y_force += 4 * int(output_neurons(0, neuron_offset + 2));
                moveable.y_force -= 4 * int(output_neurons(0, neuron_offset + 3));
            });
        }
    }

    using namespace Component;

    void rebuild_world(EntityManager& em)
    {
        auto position_view = em.reg.view<Position>();
        SWorld& world = em.get_singleton<SWorld>();
        world.reset_world();
        for (EntityId eid : position_view)
        {
            auto& pos = position_view.get(eid);

            EntityId existing_data = world.get_map_data(pos.x, pos.y);
            if (existing_data != entt::null)
            {
                throw exception("Failed to rebuild world, multiple entities share the same position.");
            }
            else
            {
                world.set_map_data(pos.x, pos.y, eid);
            }
        }
    }

    void update(EntityManager& em)
    {
        em.tick += 1;
        System::simple_brain_seer(em);
        System::simple_brain_calc(em);
        System::simple_brain_mover(em);
        System::random_movement(em);
        System::movement(em);
        System::predation(em);
    }

    void multiupdate(EntityManager& em, int n)
    {
        for (; n > 0; n--)
        {
            update(em);
        }
    }

#define GRIDWORLD_DUP(com) \
if (em.reg.has<Component::com>(eid))\
{\
    auto& c = em.reg.get<Component::com>(eid);\
    em.reg.assign<Component::com>(dup_eid, c);\
};
#define GRIDWORLD_DUP_TAG(tag) \
if (em.reg.has<Component::tag>(eid))\
{\
    em.reg.assign<Component::tag>(dup_eid);\
};

    EntityId duplicate_entity(EntityManager &em, EntityId eid)
    {
        EntityId dup_eid = em.reg.create();

        GRIDWORLD_DUP(Moveable)
        GRIDWORLD_DUP(Position)
        GRIDWORLD_DUP(Name)
        GRIDWORLD_DUP(RNG)
        GRIDWORLD_DUP(Scorable)
        GRIDWORLD_DUP(SimpleBrain)
        GRIDWORLD_DUP(SimpleBrainSeer)
        GRIDWORLD_DUP(SimpleBrainSeer)
        GRIDWORLD_DUP(SimpleBrainMover)
        GRIDWORLD_DUP(Predation)
        GRIDWORLD_DUP(PyMeta)
        GRIDWORLD_DUP_TAG(RandomMover)

        return dup_eid;
    }
}

// Forward declaration of python binding functions
namespace py = pybind11;
void bind_components_to_python_module(py::module&);
void bind_components_to_entity_manager(py::class_<GridWorld::EntityManager>&);

PYBIND11_MODULE(gridworld, m)
{
    using namespace py::literals;
    using namespace GridWorld;

    m.doc() = "GridWorld module.";

    auto entity_manager_class = py::class_<EntityManager>(m, "EntityManager")
        .def(py::init<>())
        .def_readwrite("tick", &EntityManager::tick)
        .def("get_matching_entities", &EntityManager::get_matching_entities)
        .def("create", &EntityManager::create)
        .def("destroy", &EntityManager::destroy)
        ;

    bind_components_to_python_module(m);
    bind_components_to_entity_manager(entity_manager_class);

    m.def("multiupdate", &multiupdate, py::call_guard<py::gil_scoped_release>());
    m.def("rebuild_world", &rebuild_world);
    m.def("duplicate_entity", &duplicate_entity);
    m.attr("null") = py::int_((EntityId)entt::null);
}
