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
end


@agent UAV ContinuousAgent{2} begin
    #= UAV agent to receive signals (plan::) to follow orders from the Coord. 
    It includes parts for agent order calling and design parameters=#
    sched::Int
    battery::Float64 
    suppressant::Int
    speed::Float64 
    status::Symbol
    target::Tuple 
end


@agent Coord ContinuousAgent{2} begin
    #=
    =#
   sched::Int #work around for scheduling the agents  
end


#Create the steps for the agent. 

function agent_step!(patch::Patch, model)
    #if current tree is burning, possibly light others on fire
     if patch.status == :burning
         for neighbor in nearby_agents(patch, model, patch.radius_burn)
             #not sure why I need to do a euclidean distance check, but this is the only way I could get it work!
             if neighbor.status == :green && euclidean_distance(patch, neighbor, model) < patch.radius_burn*1.05 && rand(model.rng) < neighbor.prob_burn
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
    move_agent!(uav, model, 10)
end

function agent_step!(coord::Coord, model)

end
