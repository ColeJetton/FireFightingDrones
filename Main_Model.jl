# This file initializes the model using the
# It can then be called by other files for analysis
# includes logic to allow for the model to be initialized for the purpose of tuning only (IE only patch agents)

using Agents, Random

include("Model_Agents.jl")


function forest_fire(; 
    n_uav = 25,        
    n_x_cells = 50,
    radius_burn = 10.0,
    burn_time = 1, 
    prob_burn = 1.0, 
    first_burn = :center,
    tree_UQ = (0,0), #(20, 0.005),
    seed = 2,
    tune_model = false 
    )

    #initialize based on dimensions and
    rng = MersenneTwister(seed)    
    dims = (Int(n_x_cells*radius_burn), Int(n_x_cells*radius_burn))    
    space = ContinuousSpace(dims; spacing = radius_burn, periodic = false)

    #note: status method for scheduling is needed 
    order = Dict("green" => 1, "burning" => 2, "burnt" => 3, "coord" => 4, "uav" => 5)

    if tune_model
        forest = ABM(Patch, space; rng, scheduler = Schedulers.ByProperty(:sched), container = Vector)
    else
        forest = ABM(Union{Patch,UAV,Coord}, space; rng, scheduler = Schedulers.ByProperty(:sched), container = Vector)
    end
    


    # Establish  scenarios, need to change this and add more.
    if first_burn == :center
        x_min = dims[1]*0.49
        x_max = dims[1]*0.50
        y_min = x_min
        y_max = x_max
        
    elseif first_burn == :left
        x_min = 0
        x_max = dims[1]*0.05
        y_min = 0
        y_max = dims[2]
        
    else
        error("Invalid Scenario, :center or :left only at this moment")
    end

    #add in cells, odd vs even used for putting in hexagonal "cells"
    n_y_cells = floor(Integer, n_x_cells/(sqrt(3)/2))        
    x_p = []
    y_p = []

    for row in 1:n_y_cells
        Y = radius_burn * ( 0.5 + (row - 1)*sqrt(3)/2)
        if isodd(row)
            for col in 1:n_x_cells
                push!(x_p, Float64(radius_burn*(col-0.5)))
                push!(y_p, Y)
            end           
        else
            for col in 1:(n_x_cells-1)
                push!(x_p, Float64(radius_burn*col))
                push!(y_p, Y)
            end
        end       
    end

    #if the goal isn't to tune the model, then make room for home base and add in other agents
    if !tune_model
        in_r = []
        
        for ind in 1:length(x_p)
            if x_p[ind] > dims[1]*0.8 && y_p[ind] > dims[2]*0.8
                push!(in_r,ind)
            end
        end

        deleteat!(x_p, in_r)
        deleteat!(y_p, in_r)

    
    
    end

    #add in the forest patches
    #note that for mixed agent models, position has to be given first for some reason...
    for ind in 1:size(x_p)[1]
        pos = (x_p[ind], y_p[ind]) 

        if x_min <= pos[1] <= x_max && y_min <= pos[2] <= y_max
            add_agent!(pos, Patch, forest, (0,0), order["burning"], burn_time, prob_burn, radius_burn, :burning)
        else
            add_agent!(pos, Patch, forest, (0,0), order["green"], 
            burn_time + rand(forest.rng,(-tree_UQ[1]:1:tree_UQ[1])),
            prob_burn + rand(forest.rng,(-tree_UQ[2]:0.001:tree_UQ[2])),
            radius_burn, :green)   
        end
    end
    
    if !tune_model


        test_velo = (1,1)
        test_targ = (50,50)

        for _ in 1:n_uav
            add_agent!(UAV, forest, test_velo, order["uav"], 100, 100,100,:busy, test_targ)
        end

        add_agent!((dims[1]*0.9, dims[2]*0.9), Coord, forest, (0,0), order["coord"]) 
       
    end

    return forest
end
