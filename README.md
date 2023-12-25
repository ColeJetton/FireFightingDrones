# FireFightingDrones
 
This Agent Based Model (ABM) of a homogeneous fleet of Fire Fighting Drones. The purpose of this research is to explore design parameter tradeoffs of drone capabilities and fleet sizes. While a more capable drone fleet will be able to accomplish more, the cost tradeoff means that (generally) these fleets will have less vehicles. There is also the need to incorporate realistic design parameters from engineers that will be designing these drones.


There are three main agents: 
1. Patches: These agents represent the forest. The current setup works with a probabilistic fire propagation model.
    
    * Ideally it will be rebuilt later with the patches being built into the environment (to increase the speed) however the use of continuous space requires them to be agents at this time. The other option is to give agent the id of its neighbors.

2. UAVs: These are the fire fighting drones. They get assigned a patch from the coordinator agent but act autonomously to travel to the patch, put out the fire, and head back to the base when need.


3. Coordinator: This takes in a global view of the system to assign UAV to specific tasks/locations. Current setup weighs the importance of certain patches and "auctions" them o


This is a work in progress. Model tuning is still in progress and the initial model is designed to be an approximation of fire spreading based on growth rates. Future versions will be more realistic and incorporate actual forest fire modeling methods, such as ABwise.

Packages Required to Run:
```
Agents.jl 
Random.jl
InteractiveDynamics.jl
CairoMakie.jl
LazySets.jl
Statistics.jl
CurveFit.jl
```