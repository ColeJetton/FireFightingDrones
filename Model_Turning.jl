using Agents, Random
using InteractiveDynamics
using CairoMakie
using LazySets
using Statistics
using CurveFit

include("Main_Model.jl")
include("Model_Agents.jl")
include("Model_Plotting.jl")


forest = forest_fire(first_burn = :left, tune_model = true)#, n_x_cells = 5, radius_burn = 5)

@time step!(forest, agent_step!, 200)

#@time call_video(forest, "tune.mp4", 10, 100, 5)