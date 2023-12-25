using Agents, Random

#Create the agents, all of which are continuous in the simulation
#note that '@agent macro for ContinuousAgent gives position and velocity parameters automatically


@agent Patch ContinuousAgent{2} begin
    #= Tree agent as a continuous agent to be placed hexagonally in the model. 
    It includes parts for agent order calling and parameters for tuning=#
    sched::Int
    burn_time::Int
    prob_burn::Float64
    radius_burn::Float64
    status::Symbol
    dist::Float64 #distance from home, for prioritization in sorting
    edge::Bool #if it's on the edge of the forest
end


@agent UAV ContinuousAgent{2} begin
    #= UAV agent to receive signals to follow orders from the Coord. 
    It includes parts for agent order calling and design parameters=#
    sched::Int
    battery::Int
    suppressant::Int
    speed::Float64
    status::Symbol  #idle, assigned, refilling, returning
    target_pos::Tuple
    target_id::Int
end


@agent Coord ContinuousAgent{2} begin
    #= Coordinator Agent that has a global view of the simulation. 
    =#
    sched::Int
    fire_delay::Int #built in delay for corrinator to notice the file
    internal_step_counter::Int #ensures it's not constantly reassigning things
    fire_out::Bool
    steps_out::Int
end


#Create the steps for the agents. 

function agent_step!(patch::Patch, model)
    #if current tree is burning, possibly light others on fire
    if patch.status == :burning 
        check_edge = 0 #count number of neighbors that are burnt, if it's 6, then it's on the edge
        for neighbor in nearby_agents(patch, model, patch.radius_burn)
            #not sure why I need to do a euclidean distance check, but this is the only way I could get it work!
            #future versions will add in neighbor IDs to make this faster
            if euclidean_distance(patch, neighbor, model) < patch.radius_burn * 1.05
                if neighbor.status == :green  && rand(model.rng) < neighbor.prob_burn
                    neighbor.status = :burning
                    neighbor.sched = 2
                end

                if neighbor.status == :burning || neighbor.status == :burnt
                    check_edge += 1
                end

            end
        end

        #continue burning
        patch.burn_time -= 1
        if patch.burn_time == 0
            patch.status = :burnt
            patch.edge = false
        end

        #update edge status if needed
        if patch.status == :burning && check_edge != 6
            patch.edge = true
        else 
            patch.edge = false
        end


    end
    
end


#placeholder agent steps for testing, will add in more later

function agent_step!(uav::UAV, model)
    if uav.status == :idle
        #do nothing, waits for the target to be assigned from the coordinator

    elseif uav.status == :assigned
        #if at target, find that ID and then start using suppressant
        #if not at target, move towards target

        if uav.pos == uav.target_pos
            #find patch with that id and decrease it's burn time, assuming it can't go below 0
            patch = model.agents[uav.target_id] 
            patch.burn_time = max(0, patch.burn_time - model.suppressant_rate) 
           
            #reduce intenral suppressant
            uav.suppressant = max(0, uav.suppressant - model.suppressant_rate)

            #if the patch is out, change status to idle and patch status to burnt
            if patch.burn_time == 0 || patch.edge == false
                uav.vel = (0, 0)
                uav.status = :idle
                patch.status = :burnt
                patch.edge = :false
            end

            #if out of suppressant (regardless of state), change status to returning
            if uav.suppressant == 0
                uav.status = :returning
                uav.target_pos = model.base_location
            end
            #print("sup", uav.suppressant,"\n")

        else
            #move towards target, but check if it overshot and reassign based on it being "close enough")
            if euclidean_distance(uav, model.agents[uav.target_id], model) <= sqrt(uav.vel[1]^2 + uav.vel[2]^2)
                uav.pos = uav.target_pos
                uav.vel = (0,0)
            else
                #move_agent!(uav, model, 1) #THIS SHOULD WORK
                #I have no clue what is going on here, but I built an exception to get it to work
                #In the mean time, this is a dumb work around that seems to be fine

                try
                    move_agent!(uav, model, 1)
                catch MethodError
                    #uav.pos = uav.target_pos
                    #uav.vel = (0,0)
                    uav.pos = (uav.pos[1] + uav.vel[1], uav.pos[2] + uav.vel[2])
                end

            end


        end


        #reduce it's battery, check to see if it needs to return and refill
        uav.battery -= 1
        
        if (euclidean_distance(uav,  model.agents[end], model) > uav.speed*uav.battery) || uav.suppressant == 0 #if it can't make it back            
            uav.battery = 0 #set battery to 0, even though that's not what happens (for simplicity)
            uav.status = :returning
            uav.target_pos = model.base_location
            ratio = uav.speed/euclidean_distance(uav, model.agents[1], model)
            uav.vel = ((model.base_location[1] - uav.pos[1])*ratio, (model.base_location[2] - uav.pos[2])*ratio)
        end

    elseif uav.status == :returning
        #move towards home
        #if at the home, change the status to refilling

        if uav.pos == model.base_location
            uav.vel = (0,0)
            uav.status = :refilling
        else
            #move towards base, but check if it overshot
            if euclidean_distance(uav, model.agents[end], model) <= sqrt(uav.vel[1]^2 + uav.vel[2]^2)
                uav.pos = model.base_location
                uav.vel = (0,0)
            else
                #see above about weird issue with move_agent!
                try
                    move_agent!(uav, model, 1)
                catch MethodError
                    uav.pos = (uav.pos[1] + uav.vel[1], uav.pos[2] + uav.vel[2])
                end
            end
        end

    elseif uav.status == :refilling
        #refill the battery and suppressant
        #if full, change status to idle

        uav.battery = min(uav.battery + model.battery_recharge, model.battery_max)
        uav.suppressant = min(uav.suppressant + model.suppressant_recharge, model.suppressant_max) 

        if uav.battery == model.battery_max && uav.suppressant == model.suppressant_max
            uav.status = :idle
        end

    else
        print("Error: UAV status not recognized")
    end


end

function agent_step!(coord::Coord, model)
    #= Coordinates the agents. Unsure how yet
    Get the positions and statuses of the patches and the uavs
    Use positions of burning patches to determine the targets (may be more or less than n_uav)
    Assign targets to drones =#

    coord.internal_step_counter += 1
    #NOTE: New plan. assign by patch. No need for dynamics yet.
    #auction based on location and business etc.
    #dynamically do so since you can cite that. IE mix of FIFO etc
    #don't send out new plans all the time. slows things down

    if rem(coord.internal_step_counter, 10) == 0 && coord.internal_step_counter >= coord.fire_delay
        #=
        patches = [p for p in allagents(model) if p isa Patch]
        patches_burning = [p for p in patches if p.status == :burning]
        patches_burning = sort(patches_burning, by=p -> p.dist)
        patches_burnt = [p for p in patches if p.status == :burnt] 
        patches = [patches_burning; patches_burnt]
        centroid = (mean([p.pos[1] for p in patches]), mean([p.pos[2] for p in patches]))
        =#

        prioritized_patches = patch_prioritization(model)

        if check_out_of_control(model) == false && coord.fire_out == true
            coord.fire_out = false
        end

        if length(prioritized_patches) == 0
            #fires are put out! simulation can end
            #note that I can't figure out how to get the simulation to end on its own
            #print("Fires are out! Done in ", coord.internal_step_counter - 1, " steps. \n")
            if  coord.steps_out == 0
                coord.steps_out = coord.internal_step_counter - 1
            end
        end



        free_uavs = [u for u in allagents(model) if u isa UAV]
        free_uavs = [u for u in free_uavs if u.status == :idle]

        #assign patch targets to uavs based on an auction system

        if length(free_uavs) > 0
            #find distance between uav and each patch based on the priority of the patch
            #note that this a bit of a hack since a priority queue would be better           
            for i in 1:length(prioritized_patches)

                if length(free_uavs) == 0
                    break
                end

                free_uavs = sort(free_uavs, by=u -> euclidean_distance(u, prioritized_patches[i], model))

                assign_drone = popfirst!(free_uavs)
                assign_drone.target_pos = prioritized_patches[i].pos
                assign_drone.target_id = prioritized_patches[i].id
                
                ratio =assign_drone.speed/euclidean_distance(assign_drone, prioritized_patches[i], model)
                new_vel = ((prioritized_patches[i].pos[1] - assign_drone.pos[1])*ratio, (prioritized_patches[i].pos[2] - assign_drone.pos[2])*ratio)
                assign_drone.vel = new_vel
                assign_drone.status = :assigned

            end
        end
    end
end

# Support functions for the agent steps

function patch_prioritization(model)
    #=

    =#
        patches = [p for p in allagents(model) if p isa Patch]
        patches_burning_on_edge = [p for p in patches if p.edge == true]
        patches_burnt = [p for p in patches if p.status == :burnt]
        patches = [patches_burning_on_edge; patches_burnt]
        centroid = (mean([p.pos[1] for p in patches]), mean([p.pos[2] for p in patches]))
        weights = [2, 4] #weighting for distance and time to burn
        prioritized_patches = sort(patches_burning_on_edge, by=p -> weights[1] * p.dist + weights[2] * sqrt( (p.pos[1] - centroid[1])^2 + (p.pos[2] - centroid[2])^2))

    return prioritized_patches
end

function check_out_of_control(model)
    #checks the edge patches to explore if the fire got "out of hand" and reached the edge of the map
    patches = [p for p in allagents(model) if p isa Patch]

    for p in patches
        if p.pos[1] <= 0.05*model.dims[1] || p.pos[1] >= 0.95*model.dims[1] || p.pos[2] <= 0.05*model.dims[2] || p.pos[2] >= 0.95*model.dims[2]
            if p.status == :burnt
                return false
                break
            else 
                return true
            end
        end
    end
end


