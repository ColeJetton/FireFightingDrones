using Agents, Random
using InteractiveDynamics
using CairoMakie
using LazySets
using Statistics
using CurveFit

include("Main_Model.jl")
include("Model_Agents.jl")
include("Model_Plotting.jl")

forest = forest_fire()
step!(forest, agent_step!,1)

fig = call_fig(forest)
