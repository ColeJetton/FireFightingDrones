using Agents, Random, Clustering

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
end


#Create the steps for the agents. 

function agent_step!(patch::Patch, model)
    #if current tree is burning, possibly light others on fire
    if patch.status == :burning
        for neighbor in nearby_agents(patch, model, patch.radius_burn)
            #not sure why I need to do a euclidean distance check, but this is the only way I could get it work!
            if neighbor.status == :green && euclidean_distance(patch, neighbor, model) < patch.radius_burn * 1.05 && rand(model.rng) < neighbor.prob_burn
                neighbor.status = :burning
                neighbor.sched = 2
            end
        end

        #continue burning
        patch.burn_time -= 1
        if patch.burn_time == 0
            patch.status = :burnt
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
            #find patch with that id
            patch = model.agents[uav.target_id] #this may not work
            patch.burn_time = max(0, patch.burn_time - model.suppressant_rate)

            #reduce suppressant
            uav.suppressant = max(0, uav.suppressant - model.suppressant_rate)

            #if the patch is out, change status to idle
            if patch.burn_time == 0
                print(uav, "??? \n")
                uav.status = :idle
                patch.status = :burnt
            end

            #if out of suppressant, change status to returning
            if uav.suppressant == 0
                uav.status = :returning
                uav.target_pos = model.base_location
            end
            #print("sup", uav.suppressant,"\n")

        else
            #move towards target, but check if it overshot
            d_i = euclidean_distance(uav, model.agents[uav.target_id], model)
            print(uav,"\n")
            move_agent!(uav, model, 1)
            print(uav,"\n\n")
            if d_i > euclidean_distance(uav, model.agents[uav.target_id], model)
                uav.pos = uav.target_pos
            end

        end


        #reduce it's battery, check to see if it needs to return and refill
        uav.battery -= 1

        if euclidean_distance(uav, model.agents[1], model) > uav.speed*uav.battery #if it can't make it back            
            uav.status = :returning
            uav.target_pos = model.base_location
            ratio = uav.speed/euclidean_distance(assign_drone, model.agents[1], model)
            uav.vel = ((model.base_location[1] - uav.pos[1])*ratio, (model.base_location[2] - uav.pos[2])*ratio)
        end

    elseif uav.status == :returning
        #move towards home
        #if at the home, change the status to refilling

        if uav.pos == model.base_location
            uav.status = :refilling
        else
            #move towards base, but check if it overshot
            d_i = euclidean_distance(uav, model.agents[1], model)
            
            print("moving back",uav,"\n")
            move_agent!(uav, model, 1)

            if d_i > euclidean_distance(uav, model.agents[1], model)
                uav.pos = model.base_location
            end
        end

    elseif uav.status == :refilling
        #refill the battery and suppressant
        #if full, change status to idle

        uav.battery = min(uav.battery + model.battery_recharge, model.battery_max)
        uav.suppressant = min(uav.suppressant + model.suppressant_recharge, model.suppressant_max) 

        if uav.battery == model.battery_max && uav.suppressant == model.suppressant_max
            uav.status = :idle
            uav.vel = (0,0)
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
        patches = [p for p in allagents(model) if p isa Patch]
        patches_burning = [p for p in patches if p.status == :burning]
        patches_burning = sort(patches_burning, by=p -> p.dist)

        if length(patches_burning) == 0
            #fires are put out! simulation can end
            #note that I can't figure out how to get the simulation to end on its own
            print("Fires are out! Done in ", coord.internal_step_counter - 1, " steps. \n")
        end

        free_uavs = [u for u in allagents(model) if u isa UAV]
        free_uavs = [u for u in free_uavs if u.status == :idle]

        #assign patch targets to uavs based on an auction system

        if length(free_uavs) > 0
            #find distance between uav and each patch based on the priority of the patch
            #note that this a bit of a hack since a priority queue would be better           
            for i in 1:length(patches_burning)

                if length(free_uavs) == 0
                    break
                end

                free_uavs = sort(free_uavs, by=u -> euclidean_distance(u, patches_burning[i], model))

                assign_drone = popfirst!(free_uavs)
                assign_drone.target_pos = patches_burning[i].pos
                assign_drone.target_id = patches_burning[i].id
                
                ratio =assign_drone.speed/euclidean_distance(assign_drone, patches_burning[i], model)
                print(euclidean_distance(assign_drone, patches_burning[i], model),"\n")
                new_vel = ((patches_burning[i].pos[1] - assign_drone.pos[1])*ratio, (patches_burning[i].pos[2] - assign_drone.pos[2])*ratio)
                assign_drone.vel = new_vel
                print(assign_drone,"\n")
                assign_drone.status = :assigned

            end
        end
    end
end

# Support functions for the agent steps
