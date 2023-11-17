#support functions for plotting model in the analysis 


function patch_color(pf)
    # quick work around for a scale based on probability of catching fire...
    # I wish I could figure out how to scale this better
    colors = ("#0A7F00", "#0C9300", "#0EAF00","#10C900")
    pr = [0, 0.0075, 0.01, 0.0125]

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
        color = :blue
    else
        color = :purple
    end
    color
end

function agent_size(a)
    if a isa Patch
        if a.status == :burning
            sz = 16
        elseif a.status == :green
            sz = 20
        else
            sz = 14
        end
        
    else
        sz = 20

    end

end

const uav_polygon = Makie.Polygon(Point2f[(-.5,-.5), (1,0), (-.5,.5)])

const coord_polygon = Makie.Polygon(Point2f[(-.75,-1), (-.75,0), (-1,0), (0,1), (1, 0), (0.75, 0), (0.75, -1)])

function uav_marker(u::UAV)
    t = atan(u.vel[2], u.vel[1])
    shape = rotate_polygon(uav_polygon, t)
    return shape
end

function agent_shape(a)
    if a isa Patch
        shape = :hexagon
        if a.status == :burning
            shape = :circle
        elseif a.status == :burnt
            shape = :rect
        end
        
    elseif a isa UAV
        shape = uav_marker(a)

    else
        shape = coord_polygon
    end
    shape
end


function call_fig(model)
    figure, _ = Agents.abmplot(model; ac = agent_color,as = agent_size, am = agent_shape, scatterkwargs = (strokewidth = 0.,), figure = (;resolution = (750,750)))
    return figure

end