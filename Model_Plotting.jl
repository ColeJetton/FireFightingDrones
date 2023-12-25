#support functions for plotting model in the analysis 


function patch_color(pf)
    # quick work around for a scale based on probability of catching fire...
    # I wish I could figure out how to scale this better
    colors = ("#0A7F00", "#0C9300", "#0EAF00","#10C900")
    pr = [0, 0.025, 0.03, 0.035]

    if pr[1] < pf <= pr[2]
        color = colors[1]

    elseif pr[2] < pf <= pr[3]
        color = colors[2]

    elseif pr[3]< pf <= pr[4]
        color = colors[3]

    else 
        color = colors[4]
        
    end
    
    return color
end

function agent_color(a)
    if a isa Patch
        if a.status == :burning
            color = :crimson
        elseif a.status == :burnt
            color = :gray75
        else
            color = patch_color(a.prob_burn)            
        end
    elseif a isa UAV
        color = :purple4
    else
        color = :blue
    end
    color
end

function agent_size(a)
    if a isa Patch
        if a.status == :burning
            sz = 20
            #sz = 14
        elseif a.status == :green
            sz = 20
            #sz = 10
        else
            sz = 16
            #sz = 10
        end
        
    elseif a isa UAV
        sz = 32

    else 
        sz = 25
    end

end


const coord_polygon = Makie.Polygon(Point2f[(-.75,-1), (-.75,0), (-1,0), (0,1), (1, 0), (0.75, 0), (0.75, -1)])

const uav_polygon = Makie.Polygon(Point2f[(-1,0), (0,1), (1, 0), (0, -1)])

function agent_shape(a)
    if a isa Patch
        shape = :hexagon
        if a.status == :burning
            shape = :star8
        elseif a.status == :burnt
            shape = :rect
        end
        
    elseif a isa UAV
        shape = :xcross 

    else
        shape = coord_polygon
    end
    shape
end


function call_fig(model)
    figure, _ = Agents.abmplot(model; ac = agent_color,as = agent_size, am = agent_shape, scatterkwargs = (strokewidth = 0.,), figure = (;resolution = (750,750)))
    return figure

end

function call_video(model, filename,  framerate, frames, spf)
    Agents.abmvideo(filename, model, agent_step!;
     ac = agent_color,as = agent_size, am = agent_shape, scatterkwargs = (strokewidth = 0.,), 
     spf = spf, framerate = framerate, frames = frames)

end