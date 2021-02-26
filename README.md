# GridWorld
GridWorld is an example simulation created for
[PyGridWorld](https://github.com/MidZik/pygridworld).

It simulates a simple 2D "grid world", where AI agents can move around
to avoid dangerous obstacles. It features a small and simple neural
network system that evolves over time, by duplicating and mutating
agents that perform the best in a given time period. To this end, it
provides some tweakable parameters in its state, so that a user can
modify how the simulation behaves, and examine differences in
performance based on the tweaks.

## Dependencies
+ Eigen
+ EnTT 3.0
