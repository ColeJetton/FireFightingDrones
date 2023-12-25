using Agents, Random
using InteractiveDynamics
using CairoMakie
using LazySets
using Statistics
using CurveFit
using DelimitedFiles

include("Main_Model.jl")
include("Model_Agents.jl")
include("Model_Plotting.jl")
include("Tracking_Functions.jl")

#forest2 = forest_fire(first_burn = :left,n_uav = 10, suppressant_max = 300, fire_delay = 100, uav_speed = 20)#, n_x_cells = 100, radius_burn = 5)
forest2 = forest_fire(tune_model = false, first_burn = :center, seed = 5,n_uav = 105, suppressant_max = 270, battery_max = 50)
#@time step!(forest2, agent_step!, 1000)

seconds = 5
spf = 8
framerate = 12
frames = framerate*seconds

#@time call_video(forest2, "test_design.mp4", framerate, frames, spf)

#fig = call_fig(forest2)

#save("test.png", fig, px_per_unit = 6)

# matrices for recording data on the runs
n_samples = 30
n_designs = 25

n_trees_end = zeros(n_designs, n_samples)
contained_tf = zeros(n_designs, n_samples)
time_to_contain = zeros(n_designs, n_samples)

#_, mdata = run!(forest2, agent_step!, 1000; mdata = assets_analysis)


n_burn(a) = a isa Patch && a.status == :burnt
is_contain(a) = a isa Coord && a.fire_out == true
n_steps(a) = a isa Coord && a.steps_out > 0 #work around for now, just use proportion of steps where it is out

agent_assets = [(n_burn, count), (is_contain, count), (n_steps, sum)]

#adata, _ = run!(forest2, agent_step!, 1000, adata = agent_assets)

#Int( floor((1 - mean(adata.sum_n_steps))*1000) )
#create the sample designs 

designs = zeros(25,3)
suppresent = [10, 50, 90, 130, 170]#[20, 100, 180, 260, 340]#[30,150,270,390,510]
battery_range =  [10, 30, 50, 70, 90]#[10,50,90,130,170]
n_drones = [99 87 75 63 51 87 75 63 51 39 75 63 51 39 27 63 51 39 27 15 51 39 27 15 3]
#n_drones = [165, 145, 125, 105, 85, 145, 125, 105, 85, 65, 125, 105, 85, 65, 45, 105, 85, 65, 45, 25, 85, 65, 45, 25, 5]

for i in 1:1:5
    for j in 1:1:5
        designs[(i-1)*5 + j,1] = suppresent[i]
        designs[(i-1)*5 + j,2] = battery_range[j]
        designs[(i-1)*5 + j,3] = n_drones[(i-1)*5 + j]
    end

end



for i in 1:1:n_designs
    for j in 1:1:n_samples
        #create the model with the specific design and random variable setup
        forest_n = forest_fire(suppressant_max = designs[i,1], battery_max = designs[i,2], seed = j, n_uav = designs[i,3])
        #run the model for 1000 steps
        adata, _ = run!(forest_n, agent_step!, 1000, adata = agent_assets)
        #record the data
        n_trees_end[i,j] = adata.count_n_burn[end]
        contained_tf[i,j] = adata.count_is_contain[end]
        time_to_contain[i,j] = Int( floor((1 - mean(adata.sum_n_steps))*1000) )
    end
    print(i, "\n")
end


writedlm("was_contained_C.csv", contained_tf, ',')
writedlm("trees_burned_C.csv", n_trees_end, ',')
writedlm("steps_taken_C.csv", time_to_contain, ',')


