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

#forest = forest_fire(first_burn = :left, tune_model = true, n_x_cells = 5, radius_burn = 5)

#@time step!(forest, agent_step!, 200)

#@time call_video(forest, "tune.mp4", 10, 100, 5)



tree_burn_test = [60, 90,120]

tree_UQ_test = [5, 10, 15]

p_burn_test = [0.0075, 0.01, 0.0125]

p_UQ_test = [0.003, 0.005, 0.007]


let n_test = 0

#set up a matrix for storing the data
n_tests = 3
advancement_data = zeros(length(tree_burn_test)*length(tree_UQ_test)*length(p_burn_test)*length(p_UQ_test), (n_tests + 2))

#=
    test_n:1 Tree Burn: 30 Tree UQ: 5 Prob Burn: 0.01 Prob UQ: 0.005
    test_n:2 Tree Burn: 30 Tree UQ: 5 Prob Burn: 0.01 Prob UQ: 0.0075
    test_n:3 Tree Burn: 30 Tree UQ: 5 Prob Burn: 0.01 Prob UQ: 0.01
    test_n:4 Tree Burn: 30 Tree UQ: 5 Prob Burn: 0.02 Prob UQ: 0.005
    test_n:5 Tree Burn: 30 Tree UQ: 5 Prob Burn: 0.02 Prob UQ: 0.0075
    test_n:6 Tree Burn: 30 Tree UQ: 5 Prob Burn: 0.02 Prob UQ: 0.01
    test_n:7 Tree Burn: 30 Tree UQ: 5 Prob Burn: 0.03 Prob UQ: 0.005
    test_n:8 Tree Burn: 30 Tree UQ: 5 Prob Burn: 0.03 Prob UQ: 0.0075
    test_n:9 Tree Burn: 30 Tree UQ: 5 Prob Burn: 0.03 Prob UQ: 0.01
    test_n:10 Tree Burn: 30 Tree UQ: 10 Prob Burn: 0.01 Prob UQ: 0.005
    test_n:11 Tree Burn: 30 Tree UQ: 10 Prob Burn: 0.01 Prob UQ: 0.0075
    test_n:12 Tree Burn: 30 Tree UQ: 10 Prob Burn: 0.01 Prob UQ: 0.01
    test_n:13 Tree Burn: 30 Tree UQ: 10 Prob Burn: 0.02 Prob UQ: 0.005
    test_n:14 Tree Burn: 30 Tree UQ: 10 Prob Burn: 0.02 Prob UQ: 0.0075
    test_n:15 Tree Burn: 30 Tree UQ: 10 Prob Burn: 0.02 Prob UQ: 0.01
    test_n:16 Tree Burn: 30 Tree UQ: 10 Prob Burn: 0.03 Prob UQ: 0.005
    test_n:17 Tree Burn: 30 Tree UQ: 10 Prob Burn: 0.03 Prob UQ: 0.0075
    test_n:18 Tree Burn: 30 Tree UQ: 10 Prob Burn: 0.03 Prob UQ: 0.01
    test_n:19 Tree Burn: 30 Tree UQ: 20 Prob Burn: 0.01 Prob UQ: 0.005
    test_n:20 Tree Burn: 30 Tree UQ: 20 Prob Burn: 0.01 Prob UQ: 0.0075
    test_n:21 Tree Burn: 30 Tree UQ: 20 Prob Burn: 0.01 Prob UQ: 0.01
    test_n:22 Tree Burn: 30 Tree UQ: 20 Prob Burn: 0.02 Prob UQ: 0.005
    test_n:23 Tree Burn: 30 Tree UQ: 20 Prob Burn: 0.02 Prob UQ: 0.0075
    test_n:24 Tree Burn: 30 Tree UQ: 20 Prob Burn: 0.02 Prob UQ: 0.01
    test_n:25 Tree Burn: 30 Tree UQ: 20 Prob Burn: 0.03 Prob UQ: 0.005
    test_n:26 Tree Burn: 30 Tree UQ: 20 Prob Burn: 0.03 Prob UQ: 0.0075
    test_n:27 Tree Burn: 30 Tree UQ: 20 Prob Burn: 0.03 Prob UQ: 0.01
    test_n:28 Tree Burn: 60 Tree UQ: 5 Prob Burn: 0.01 Prob UQ: 0.005
    test_n:29 Tree Burn: 60 Tree UQ: 5 Prob Burn: 0.01 Prob UQ: 0.0075
    test_n:30 Tree Burn: 60 Tree UQ: 5 Prob Burn: 0.01 Prob UQ: 0.01
    test_n:31 Tree Burn: 60 Tree UQ: 5 Prob Burn: 0.02 Prob UQ: 0.005
    test_n:32 Tree Burn: 60 Tree UQ: 5 Prob Burn: 0.02 Prob UQ: 0.0075
    test_n:33 Tree Burn: 60 Tree UQ: 5 Prob Burn: 0.02 Prob UQ: 0.01
    test_n:34 Tree Burn: 60 Tree UQ: 5 Prob Burn: 0.03 Prob UQ: 0.005
    test_n:35 Tree Burn: 60 Tree UQ: 5 Prob Burn: 0.03 Prob UQ: 0.0075
    test_n:36 Tree Burn: 60 Tree UQ: 5 Prob Burn: 0.03 Prob UQ: 0.01
    test_n:37 Tree Burn: 60 Tree UQ: 10 Prob Burn: 0.01 Prob UQ: 0.005
    test_n:38 Tree Burn: 60 Tree UQ: 10 Prob Burn: 0.01 Prob UQ: 0.0075
    test_n:39 Tree Burn: 60 Tree UQ: 10 Prob Burn: 0.01 Prob UQ: 0.01
    test_n:40 Tree Burn: 60 Tree UQ: 10 Prob Burn: 0.02 Prob UQ: 0.005
    test_n:41 Tree Burn: 60 Tree UQ: 10 Prob Burn: 0.02 Prob UQ: 0.0075
    test_n:42 Tree Burn: 60 Tree UQ: 10 Prob Burn: 0.02 Prob UQ: 0.01
    test_n:43 Tree Burn: 60 Tree UQ: 10 Prob Burn: 0.03 Prob UQ: 0.005
    test_n:44 Tree Burn: 60 Tree UQ: 10 Prob Burn: 0.03 Prob UQ: 0.0075
    test_n:45 Tree Burn: 60 Tree UQ: 10 Prob Burn: 0.03 Prob UQ: 0.01
    test_n:46 Tree Burn: 60 Tree UQ: 20 Prob Burn: 0.01 Prob UQ: 0.005
    test_n:47 Tree Burn: 60 Tree UQ: 20 Prob Burn: 0.01 Prob UQ: 0.0075
    test_n:48 Tree Burn: 60 Tree UQ: 20 Prob Burn: 0.01 Prob UQ: 0.01
    test_n:49 Tree Burn: 60 Tree UQ: 20 Prob Burn: 0.02 Prob UQ: 0.005
    test_n:50 Tree Burn: 60 Tree UQ: 20 Prob Burn: 0.02 Prob UQ: 0.0075
    test_n:51 Tree Burn: 60 Tree UQ: 20 Prob Burn: 0.02 Prob UQ: 0.01
    test_n:52 Tree Burn: 60 Tree UQ: 20 Prob Burn: 0.03 Prob UQ: 0.005
    test_n:53 Tree Burn: 60 Tree UQ: 20 Prob Burn: 0.03 Prob UQ: 0.0075
    test_n:54 Tree Burn: 60 Tree UQ: 20 Prob Burn: 0.03 Prob UQ: 0.01
    test_n:55 Tree Burn: 90 Tree UQ: 5 Prob Burn: 0.01 Prob UQ: 0.005
    test_n:56 Tree Burn: 90 Tree UQ: 5 Prob Burn: 0.01 Prob UQ: 0.0075
    test_n:57 Tree Burn: 90 Tree UQ: 5 Prob Burn: 0.01 Prob UQ: 0.01
    test_n:58 Tree Burn: 90 Tree UQ: 5 Prob Burn: 0.02 Prob UQ: 0.005
    test_n:59 Tree Burn: 90 Tree UQ: 5 Prob Burn: 0.02 Prob UQ: 0.0075
    test_n:60 Tree Burn: 90 Tree UQ: 5 Prob Burn: 0.02 Prob UQ: 0.01
    test_n:61 Tree Burn: 90 Tree UQ: 5 Prob Burn: 0.03 Prob UQ: 0.005
    test_n:62 Tree Burn: 90 Tree UQ: 5 Prob Burn: 0.03 Prob UQ: 0.0075
    test_n:63 Tree Burn: 90 Tree UQ: 5 Prob Burn: 0.03 Prob UQ: 0.01
    test_n:64 Tree Burn: 90 Tree UQ: 10 Prob Burn: 0.01 Prob UQ: 0.005
    test_n:65 Tree Burn: 90 Tree UQ: 10 Prob Burn: 0.01 Prob UQ: 0.0075
    test_n:66 Tree Burn: 90 Tree UQ: 10 Prob Burn: 0.01 Prob UQ: 0.01
    test_n:67 Tree Burn: 90 Tree UQ: 10 Prob Burn: 0.02 Prob UQ: 0.005
    test_n:68 Tree Burn: 90 Tree UQ: 10 Prob Burn: 0.02 Prob UQ: 0.0075
    test_n:69 Tree Burn: 90 Tree UQ: 10 Prob Burn: 0.02 Prob UQ: 0.01
    test_n:70 Tree Burn: 90 Tree UQ: 10 Prob Burn: 0.03 Prob UQ: 0.005
    test_n:71 Tree Burn: 90 Tree UQ: 10 Prob Burn: 0.03 Prob UQ: 0.0075
    test_n:72 Tree Burn: 90 Tree UQ: 10 Prob Burn: 0.03 Prob UQ: 0.01
    test_n:73 Tree Burn: 90 Tree UQ: 20 Prob Burn: 0.01 Prob UQ: 0.005
    test_n:74 Tree Burn: 90 Tree UQ: 20 Prob Burn: 0.01 Prob UQ: 0.0075
    test_n:75 Tree Burn: 90 Tree UQ: 20 Prob Burn: 0.01 Prob UQ: 0.01
    test_n:76 Tree Burn: 90 Tree UQ: 20 Prob Burn: 0.02 Prob UQ: 0.005
    test_n:77 Tree Burn: 90 Tree UQ: 20 Prob Burn: 0.02 Prob UQ: 0.0075
    test_n:78 Tree Burn: 90 Tree UQ: 20 Prob Burn: 0.02 Prob UQ: 0.01
    test_n:79 Tree Burn: 90 Tree UQ: 20 Prob Burn: 0.03 Prob UQ: 0.005
    test_n:80 Tree Burn: 90 Tree UQ: 20 Prob Burn: 0.03 Prob UQ: 0.0075
    test_n:81 Tree Burn: 90 Tree UQ: 20 Prob Burn: 0.03 Prob UQ: 0.01
=#



for i = 1:1:length(tree_burn_test)
    for j = 1:1:length(tree_UQ_test)
        for k = 1:1:length(p_burn_test)
            for l = 1:1:length(p_UQ_test)
                #=
                
                forest = forest_fire(first_burn = :center, tune_model = true, n_x_cells = 100, radius_burn = 5, prob_burn = p_burn_test[k], tree_UQ = (tree_burn_test[i], p_UQ_test[l]))
                @time step!(forest, agent_step!, 1000)
                mdataf = model_data(forest)
                t_adv = track_advancement(mdataf)
                println("Tree Burn: ", tree_burn_test[i], " Tree UQ: ", tree_UQ[j], " Prob Burn: ", p_burn_test[k], " Prob UQ: ", p_UQ_test[l], " Advancement: ", mean(t_adv))
                =#
                n_test += 1 #increment test number, also use for storing data

                for q = 1:1:n_tests #30 tests for calculating the average 
                    #just use q as the seed too, easiest way to do it

                    forest = forest_fire(seed = q, first_burn = :left, tune_model = true, n_x_cells = 100, 
                    burn_time = tree_burn_test[i], radius_burn = 5, prob_burn = p_burn_test[k], 
                    tree_UQ = (tree_UQ_test[j], p_UQ_test[l]))
                    
                    #run the model, no need to run till end
                    adataf, mdataf = run!(forest, agent_step!, 2000; adata = to_collect, mdata = assets_tune);
                    
                    #fit line to advancement data. The
                    intercept, slope = linear_fit(mdataf[:,1], mdataf[:,2])
                    #store the slope in table
                    advancement_data[n_test,q] = slope
                end
                
                advancement_data[n_test, (n_tests + 1)] = mean(advancement_data[n_test, 1:n_tests])
                advancement_data[n_test, (n_tests + 2)] = std(advancement_data[n_test, 1:n_tests])

                

                println("test_n:", n_test," Tree Burn: ", tree_burn_test[i], " Tree UQ: ", tree_UQ_test[j], " Prob Burn: ", p_burn_test[k], " Prob UQ: ", p_UQ_test[l], " slope: ",advancement_data[n_test, (n_tests + 1)])
            end
        end
    end
end

writedlm("advancement_data_test_2.csv", advancement_data, ',')
end

