#pragma once

#include "GridWorld.h"
#include "pcg_random.hpp"
#include <Eigen/Dense>
#include <pybind11/pybind11.h>

static int wrapi(int i, int lower_bound, int upper_bound)
{
    int range = upper_bound - lower_bound;
    i = ((i - lower_bound) % range);
    if (i < 0)
        return upper_bound + i;
    else
        return lower_bound + i;
}

namespace GridWorld::Component
{
    struct SWorld
    {
        int width = 20;
        int height = 20;
        std::vector<EntityId> map;

        void reset_world(int p_width, int p_height)
        {
            width = p_width;
            height = p_height;
            map.resize(width * height);
            for (auto i = 0; i < width * height; i++)
            {
                map[i] = entt::null;
            }
        }

        void reset_world()
        {
            reset_world(width, height);
        }

        EntityId get_map_data(int x, int y) const
        {
            return map[get_map_index(x, y)];
        }

        void set_map_data(int x, int y, EntityId data)
        {
            map[get_map_index(x, y)] = data;
        }

        int get_map_index(int x, int y) const
        {
            return normalize_y(y) * width + normalize_x(x);
        }

        int get_map_index_x(int map_index) const
        {
            return map_index % width;
        }

        int get_map_index_y(int map_index) const
        {
            return map_index / width;
        }

        int normalize_x(int x) const
        {
            return wrapi(x, 0, width);
        }

        int normalize_y(int y) const
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
        std::string major_name = ""; // "family name"
        std::string minor_name = ""; // "personal name"
    };

    using RNG = pcg32;

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
        uint64_t no_predation_until_tick = 0;
        uint32_t ticks_between_predations = 1;
        bool predate_all = true;
    };

    struct RandomMover
    {
    };

    struct Scorable
    {
        int score = 0;
    };

    typedef pybind11::dict PyMeta;
}

template<>
struct entt::is_equality_comparable<GridWorld::Component::PyMeta> : std::false_type
{
};