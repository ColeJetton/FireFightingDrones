using Agents, Random
using InteractiveDynamics
using CairoMakie
using LazySets
using Statistics
using CurveFit

include("Main_Model.jl")
include("Model_Agents.jl")
include("Model_Plotting.jl")

forest2 = forest_fire(first_burn = :center,n_uav = 10, suppressant_max = 300, fire_delay = 100, uav_speed = 20)#, n_x_cells = 100, radius_burn = 5)
#@time step!(forest, agent_step!, 1000)

@time call_video(forest2, "test_4.mp4", 10, 50, 10)

#fig = call_fig(forest)

#save("test.png", fig, px_per_unit = 6)