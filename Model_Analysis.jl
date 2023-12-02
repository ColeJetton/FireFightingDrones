using Agents, Random
using InteractiveDynamics
using CairoMakie
using LazySets
using Statistics
using CurveFit

include("Main_Model.jl")
include("Model_Agents.jl")
include("Model_Plotting.jl")

forest = forest_fire(first_burn = :lowerleft,n_uav = 2)
@time step!(forest, agent_step!,1100)

#fig = call_fig(forest)

#save("test.png", fig, px_per_unit = 6)