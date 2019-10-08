# GridWorld
GridWorld is a high-performance 2D world simulator created to study neural network behavior and evolution in a deterministic and controlled environment. Its functionality is currently exposed through python bindings as a python module.

GridWorld defines the components that make up the world, and the systems that update the world on every update tick. Modifying and analysing the simulation's state outside of the defined systems (such as to evolve the neural networks) is left to its clients (namely, [PyGridWorld](https://github.com/MidZik/pygridworld), its sibling project).

## Dependencies
+ pybind11
+ Eigen
+ EnTT 3.0
