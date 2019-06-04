// GridWorld.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "GridWorld.h"
#include <iostream>
#include <set>
#include <algorithm>
#include <queue>

using Eigen::MatrixXd;
using namespace std;

using EntityId = uint64_t;
using registry = entt::basic_registry<EntityId>;

int wrapi(int i, int lower_bound, int upper_bound)
{
    int range = upper_bound - lower_bound;
    i = ((i - lower_bound) % range);
    if (i < 0)
        return upper_bound + i;
    else
        return lower_bound + i;
}

namespace GridWorld
{
    class EntityManager
    {
    private:
        EntityId singleton_id = -1;
    public:
        registry reg;

        template <typename C>
        auto view()
        {
            return reg.view<C>();
        }

        template <typename... S>
        auto get_singletons()
        {
            assert(singleton_id != -1);

            return reg.try_get<S...>(singleton_id);
        }

        template <typename S>
        S* add_singleton()
        {
            assert(singleton_id != -1);
            if (!reg.has<S>(singleton_id))
            {
                return &reg.assign<S>(singleton_id);
            }
            else
            {
                cerr << "Error adding singleton, singleton already exists." << endl;
                return NULL;
            }
        }

        /*
        Creates 
        */
        void setup_singleton_entity()
        {
            if (singleton_id == -1)
            {
                singleton_id = reg.create();
            }
            else
            {
                cerr << "Err: Tried to create a singleton entity while one already exists." << endl;
            }
        }
    };

    namespace Component
    {
        struct SWorld
        {
            int width = 20;
            int height = 20;
            EntityId* map = new EntityId[width * height];

            void reset_world(int p_width, int p_height)
            {
                width = p_width;
                height = p_height;
                delete[] map;
                map = new EntityId[width * height];
                for (auto i = 0; i < width * height; i++)
                {
                    map[i] = -1;
                }
            }

            void reset_world()
            {
                reset_world(width, height);
            }

            EntityId get_map_data(int x, int y)
            {
                return map[get_map_index(x, y)];
            }

            void set_map_data(int x, int y, EntityId data)
            {
                map[get_map_index(x, y)] = data;
            }

            int get_map_index(int x, int y)
            {
                return normalize_y(y) * width + normalize_x(x);
            }

            int get_map_index_x(int map_index)
            {
                return map_index % width;
            }

            int get_map_index_y(int map_index)
            {
                return map_index / width;
            }

            int normalize_x(int x)
            {
                return wrapi(x, 0, width);
            }

            int normalize_y(int y)
            {
                return wrapi(y, 0, height);
            }
        };

        class NewEntityDef
        {
            virtual void create_entity() = 0;
        };
        struct SNewEntityQueue
        {
            std::vector<NewEntityDef*> queue = std::vector<NewEntityDef*>();
        };

        struct SJudge
        {
            uint64_t next_judgement_tick = 50000;
            uint64_t ticks_between_judgements = 50000;
        };

        struct Position
        {
            int x = 0;
            int y = 0;
        };

        struct Moveable
        {
            int x_force = 0;
            int y_force = 0;
        };

        struct Name
        {
            std::string name = "";
        };

        typedef pcg32 RNG;

        using SynapseMat = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
        using NeuronMat = Eigen::Matrix<float, 1, Eigen::Dynamic>;
        struct SimpleBrain
        {
            std::vector<SynapseMat> synapses;
            std::vector<NeuronMat> neurons;
            float child_mutation_chance = 0.5f;
            float child_mutation_strength = 0.2f;

            SimpleBrain()
            {
                neurons.push_back(NeuronMat::Ones(27));
                neurons.push_back(NeuronMat::Ones(9));
                neurons.push_back(NeuronMat::Ones(4));

                synapses.push_back(SynapseMat::Zero(27, 8));
                synapses.push_back(SynapseMat::Zero(9, 4));
            }

            void init_brain(int neuron_counts[], int layers)
            {
                for (int i = 0; i < layers - 1; i++)
                {
                    int in = neuron_counts[i] + 1;
                    int out = neuron_counts[i + 1];
                    neurons.push_back(NeuronMat::Ones(in));
                    synapses.push_back(SynapseMat::Zero(in, out));
                }
                // The final neuron count is output only, no bias neuron
                neurons.push_back(NeuronMat::Ones(neuron_counts[layers - 1]));
            }
        };

        struct SimpleBrainSeer
        {
            int neuron_offset = 1;
            int sight_radius = 2;
        };

        struct SimpleBrainMover
        {
            int neuron_offset = 0;
        };

        struct Predation
        {
            int no_predation_until_tick = 0;
        };

        struct RandomMover
        {
        };

        struct Scorable
        {
            int score = 0;
        };
    }

    

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
            EntityId eid = -1;
            Position* entity_position = NULL;
            int net_force = 0;
            bool finalized = false;
            _MovementInfo* accepted_child = NULL;
        };

        std::unordered_map<int, _MovementInfo> movement_nodes; // Declared globally to keep in memory
        std::set<_MovementInfo*> movement_entry_nodes;

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
                    auto parent_children = cur_movement_info->parent_node->child_nodes;
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
            If entry node has a parent, that indicates a cycle. Let all nodes in the cycle mode.
            If the entry node has an entity but has no parent, it is not moving. Reject all children.
            Otherwise, the node is empty and should accept the child with highest force (or no child if a tie exists).
            */

            if (entry_node.parent_node != NULL)
            {
                // cycle case, the entry node has a parent it wants to move to.
                auto& previous_cycle_node = entry_node;
                auto& current_cycle_node = *entry_node.parent_node;

                while (!current_cycle_node.finalized)
                {
                    current_cycle_node.accepted_child = &previous_cycle_node;
                    current_cycle_node.finalized = true;

                    for (auto child : current_cycle_node.child_nodes)
                    {
                        if (child != &previous_cycle_node)
                        {
                            traversal_queue.push(child);
                        }
                    }

                    previous_cycle_node = current_cycle_node;
                    current_cycle_node = *current_cycle_node.parent_node;
                }
            }
            else if (entry_node.eid != -1)
            {
                // reject children case (entity exists and is not moving)
                entry_node.accepted_child = NULL;
                entry_node.finalized = true;

                for (auto child : entry_node.child_nodes)
                {
                    traversal_queue.push(child);
                }
            }
            else
            {
                // Accept most forceful child case.
                int highest_force = -1;
                _MovementInfo* highest_child = NULL;

                for (auto child : entry_node.child_nodes)
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
                auto cur_node = traversal_queue.front();
                traversal_queue.pop();

                assert(cur_node->finalized);

                if (cur_node->parent_node->accepted_child == cur_node)
                {
                    // Accept most forceful child case.
                    int highest_force = -1;
                    _MovementInfo* highest_child = NULL;

                    for (auto child : cur_node->child_nodes)
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

                    for (auto child : cur_node->child_nodes)
                    {
                        traversal_queue.push(child);
                    }
                }
            }
        }

        void _traverse_and_execute_movement(SWorld& world, _MovementInfo& entry_node)
        {
            assert(entry_node.is_entry_node);

            auto cur_node = entry_node;
            int cur_map_index = cur_node.map_index;

            while (cur_node.accepted_child != NULL && world.map[cur_map_index] != cur_node.accepted_child->eid)
            {
                world.map[cur_map_index] = cur_node.accepted_child->eid;
                cur_node.accepted_child->entity_position->x = world.get_map_index_x(cur_map_index);
                cur_node.accepted_child->entity_position->y = world.get_map_index_y(cur_map_index);

                cur_node = *cur_node.accepted_child;
                cur_map_index = cur_node.map_index;
            }

            // special case: if any nodes were moved, we need to make sure the last node of the tree clears out its position if necessary 
            // (since it wasn't iterated over)
            if (cur_node.accepted_child == NULL && &cur_node != &entry_node)
            {
                world.map[cur_map_index] = -1;
            }
        }

        void movement(EntityManager& em)
        {
            auto world = em.get_singletons<SWorld>();

            auto view = em.reg.view<Moveable, Position>();

            view.each([world](EntityId eid, auto& moveable, auto& position)
            {
                _add_movement_info(eid, *world, moveable, position);
            });

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

        
    }

    using namespace Component;

    void print_coms(registry &r)
    {
        auto view = r.view<Component::Position>();

        for (auto eid : view)
        {
            auto &pos = view.get(eid);
            cout << pos.x << "," << pos.y << endl;
        }
    }

    EntityId create_brain_entity(EntityManager& em, int x, int y, uint64_t seed, uint64_t seq)
    {
        registry &reg = em.reg;

        EntityId eid = reg.create();

        reg.assign<Position>(eid, x, y);

        reg.assign<Moveable>(eid);

        reg.assign<Scorable>(eid);

        reg.assign<SimpleBrain>(eid);

        reg.assign<SimpleBrainMover>(eid);

        reg.assign<SimpleBrainSeer>(eid);

        auto rng = reg.assign<RNG>(eid);
        rng.seed(seed, seq);

        reg.assign<Name>(eid, "x" + std::to_string(x) + "y" + std::to_string(y));

        return eid;
    }

    EntityId create_predator(EntityManager& em, int x, int y, uint64_t seed, uint64_t seq)
    {
        registry &reg = em.reg;

        EntityId eid = reg.create();

        reg.assign<Position>(eid, x, y);

        reg.assign<Moveable>(eid);

        reg.assign<RandomMover>(eid);

        reg.assign<Predation>(eid);

        auto rng = reg.assign<RNG>(eid);
        rng.seed(seed, seq);

        reg.assign<Name>(eid, "x" + std::to_string(x) + "y" + std::to_string(y));

        return eid;
    }

    void rebuild_world(EntityManager& em)
    {
        auto position_view = em.reg.view<Position>();
        SWorld* world = em.get_singletons<SWorld>();
        world->reset_world();
        for (EntityId eid : position_view)
        {
            auto& pos = position_view.get(eid);

            EntityId existing_data = world->get_map_data(pos.x, pos.y);
            if (existing_data != -1)
            {
                cerr << "Failed to rebuild world, multiple entities share the same position." << endl;
                break;
            }
            else
            {
                world->set_map_data(pos.x, pos.y, eid);
            }
        }
    }

    EntityManager& create_test_em()
    {
        EntityManager* em = new EntityManager;
        registry& reg = em->reg;

        em->setup_singleton_entity();

        em->add_singleton<SWorld>();

        auto* judge = em->add_singleton<SJudge>();
        judge->next_judgement_tick = 50000;
        judge->ticks_between_judgements = 50000;

        auto* rng = em->add_singleton<pcg32>();
        rng->seed(123456789, 987654321);

        for (auto i = 0; i < 10; i++)
        {
            create_brain_entity(*em, i, i, 13579 + i, 97531 + i);
        }

        for (auto i = 0; i < 10; i++)
        {
            create_predator(*em, i + 1, i - 1, 246893 + 13 * i, 975869 + 17 * i);
            create_predator(*em, i + 4, i - 4, 7774569 + 39 * i, 3882451 + 51 * i);
        }

        rebuild_world(*em);

        print_coms(reg);
        return *em;
    }

    void run_test(EntityManager& em)
    {
        for (auto i = 0; i < 1000000; i++)
        {
            System::simple_brain_calc(em);
            System::random_movement(em);
            System::movement(em);
            if (i % 100000 == 0)
            {
                cout << i << endl;
            }
        }
        cout << 1000000 << endl;
    }
}