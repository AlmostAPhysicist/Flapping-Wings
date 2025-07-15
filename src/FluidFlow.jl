using LinearAlgebra
using GLMakie, GeometryBasics
include("Shapes.jl")




function velocity(r::AbstractVector{<:Real}, R1::AbstractVector{<:Real}, R2::AbstractVector{<:Real}, Gamma::Real)
    #https://www.tfd.chalmers.se/~lada/postscript_files/Licentiate_thesis_Hamid.pdf
    @assert length(r) == 3 "Position vector r must be a 3D vector."
    @assert length(R1) == 3 "Position vector R1 must be a 3D vector."
    @assert length(R2) == 3 "Position vector R2 must be a 3D vector."

    R1r = r - R1
    R2r = r - R2

    K12 = dot(-Gamma * (R2 - R1) / (4 * pi * norm(cross(R1r, R2r))^2), (normalize(R2r) - normalize(R1r)))
    v = K12 * cross(R1r, R2r)

end

function velocity2(r::AbstractVector{<:Real}, R1::AbstractVector{<:Real}, R2::AbstractVector{<:Real}, Gamma::Real)
    # A different function definition
    @assert length(r) == 3 "Position vector r must be a 3D vector."
    @assert length(R1) == 3 "Position vector R1 must be a 3D vector."
    @assert length(R2) == 3 "Position vector R2 must be a 3D vector."

    h = r - R1
    L = R2 - R1
    h_dot_L = dot(h, L)
    h_cross_L = cross(h, L)

    prefactor = Gamma / (4 * pi) * (-h_cross_L) * (1/norm(h_cross_L)^2)
    term1 = (norm(L)^2 - h_dot_L)/norm(h-L) 
    term2 = h_dot_L/norm(h)
    v = prefactor * (term1 + term2)
    return v
end


mutable struct VortexLine
    R1::AbstractVector{<:Real}  # Position of the first vortex
    R2::AbstractVector{<:Real}  # Position of the last vortex
    Gamma::Real                 # Circulation strength
end

mutable struct VortexLoop
    vortices::Vector{VortexLine}  # Vector of VortexLine objects
end

function VortexLoop(centre::AbstractVector{<:Real}, radius::Real, Gamma::Real, upvector::AbstractVector{<:Real}, ngon::Int=4, num_lines::Int=4)
    # each vertex of the ngon will create a line along which there will be equally spaced num_lines number of vortices
    @assert length(centre) == 3 "Centre must be a 3D vector."
    @assert radius > 0 "Radius must be a positive number."
    @assert length(upvector) == 3 "Upvector must be a 3D vector."
    @assert ngon >= 3 "Number of sides (ngon) must be at least 3."
    @assert num_lines >= 1 "Number of loops must be at least 1."

    # For each vortex pairs generate a linear point space of num_lines+1 points
    # for those points on a linear line, pair them as R1 and R2 for VortexLine
    # So for each vortrex loop, say you have 4 vertices, and you divide each in 4 parts.
    # You would then have 4 parpendicular lines, each with 4 VortexLine objects
    # Each VortexLine object will have R1 and R2 as the start and end points of the line segment
    # 2 Adjacent VortexLine objects will have R2 of the first as R1 of the second

    # Normalize the upvector
    up = normalize(upvector)

    # Generate ngon vertices in the plane defined by the upvector
    temp = abs(up[1]) < 0.9 ? Vec3{Float64}(1, 0, 0) : Vec3{Float64}(0, 1, 0)
    u = normalize(cross(up, temp))
    v = cross(up, u)  # This is already normalized since up and u are orthonormal
    theta = range(0, stop=2 * pi, length=ngon + 1)[1:end-1]  # Exclude the duplicate end point
    vertices = [centre + radius * (cos(t) * u + sin(t) * v) for t in theta]

    # Create ordered vortex lines
    vortices = Vector{VortexLine}()
    
    for i in 1:ngon
        R1 = vertices[i]
        R2 = vertices[mod1(i + 1, ngon)]  # Wrap around to the first vertex

        # Generate num_lines equally spaced segments between R1 and R2
        for j in 1:num_lines
            t1 = (j - 1) / num_lines  # Start parameter for this segment
            t2 = j / num_lines        # End parameter for this segment
            
            point_R1 = R1 + t1 * (R2 - R1)
            point_R2 = R1 + t2 * (R2 - R1)

            push!(vortices, VortexLine(Point3{Float64}(point_R1...), Point3{Float64}(point_R2...), Gamma))
        end
    end

    return VortexLoop(vortices)
end

mutable struct VortexRibbon
    loops::Vector{VortexLoop}      # Vector of VortexLoop objects
end
function VortexRibbon(centre::AbstractVector{<:Real}, radius::Real, Gamma::Real, upvector::AbstractVector{<:Real}, ngon::Int=4, num_loops::Int=4, loop_attributes::Dict{Symbol, <:Any} = Dict())
    @assert length(centre) == 3 "Centre must be a 3D vector."
    @assert radius > 0 "Radius must be a positive number."
    @assert length(upvector) == 3 "Upvector must be a 3D vector."
    @assert ngon >= 3 "Number of sides (ngon) must be at least 3."
    @assert num_loops >= 1 "Number of loops must be at least 1."

    loop_ngon = get(loop_attributes, :ngon, 4)
    loop_num_segments = get(loop_attributes, :num_segments, 1)
    
    up = normalize(upvector)

    temp = abs(up[1]) < 0.9 ? Vec3{Float64}(1, 0, 0) : Vec3{Float64}(0, 1, 0)
    u = normalize(cross(up, temp))
    v = cross(up, u)
    theta = range(0, stop=2 * pi, length=ngon + 1)[1:end-1]
    vertices = [centre + radius * (cos(t) * u + sin(t) * v) for t in theta]
    
    loops = Vector{VortexLoop}()
    
    for i in 1:ngon
        R1 = vertices[i]
        R2 = vertices[mod1(i + 1, ngon)]
        
        # Calculate segment length
        segment_length = norm(R2 - R1)
        
        # Calculate the subtracted distance d based on ngon and num_loops
        alpha = pi / ngon  # Half of the exterior angle in radians
        d = (segment_length * (1 - cos(alpha))) / (2 * num_loops * cos(alpha) + 2 * (1 - cos(alpha)))
        
        # Adjust R1 and R2 by subtracting d along the segment direction
        segment_direction = normalize(R2 - R1)
        adjusted_R1 = R1 + d * segment_direction
        adjusted_R2 = R2 - d * segment_direction
        
        # Calculate new segment length and radius
        new_segment_length = norm(adjusted_R2 - adjusted_R1)
        loop_radius = new_segment_length / (2 * num_loops)
        
        # Adjust center positions to prevent diagonal overlap at corners
        adjusted_centers = Vector{Vector{Float64}}()
        for j in 1:num_loops
            t = (j - 0.5) / num_loops  # Center the loop along the adjusted segment
            loop_centre = adjusted_R1 + t * (adjusted_R2 - adjusted_R1)
            
            # Check distance to previous corner center only if there is a previous center
            if !isempty(adjusted_centers) && (j > 1 || i > 1)
                prev_center = adjusted_centers[end]
                dist = norm(loop_centre - prev_center)
                min_dist = 2 * loop_radius  # Ensure distance is at least 2r for diagonal separation
                if dist < min_dist
                    adjustment = (loop_centre - prev_center) / norm(loop_centre - prev_center) * (min_dist - dist) / 2
                    loop_centre += adjustment
                end
            end
            push!(adjusted_centers, loop_centre)
            
            # Create the vortex loop at this center with calculated radius
            loop = VortexLoop(loop_centre, loop_radius, Gamma, up, loop_ngon, loop_num_segments)
            push!(loops, loop)
        end
    end
    
    return VortexRibbon(loops)
end

# Test
a = Dict(:ngon => 10, :num_segments => 1)
a isa Dict{Symbol, <:Any} || error("Attributes must be a Dict with Symbol keys and Any values.")
v = VortexRibbon([0.0, 1.0, 10.0], 1.0, 1.0, [0.0, 0.0, 1.0], 20, 10, a)


#Plotting with line!
s = Scene(camera=cam3d!)



for loop in v.loops
    for vortex in loop.vortices
        R1 = vortex.R1
        R2 = vortex.R2
        lines!(s, [R1[1], R2[1]], [R1[2], R2[2]], [R1[3], R2[3]], linewidth=2)
    end
end

velocity(r::AbstractVector{<:Real},v::VortexLine) = velocity(r, v.R1, v.R2, v.Gamma)
velocity(r::AbstractVector{<:Real},v::VortexLoop) = sum(velocity(r, vortex) for vortex in v.vortices)
velocity(r::AbstractVector{<:Real},v::VortexRibbon) = sum(velocity(r, loop) for loop in v.loops)


function flux(n::AbstractVector{<:Real}, F::AbstractVector{<:Real})
    @assert length(n) == 3 "Normal vector n must be a 3D vector."
    @assert length(F) == 3 "Vector field F must be a 3D vector."

    return dot(n, F)
end

function approx_flux(F::Function, pos::AbstractVector{<:Real}, radius::Real)
    @assert length(pos) == 3 "Position must be a 3D vector."
    @assert radius > 0 "Radius must be a positive number."

    d = Dodecagon(pos, radius)
    face_centroids = d.face_centroids
    face_areas = d.face_areas

    net_flux = 0.0
    for i in eachindex(face_centroids)
        # For a sphere approximation, the outward normal at each face centroid
        # is simply the normalized vector from center to face centroid
        outward_normal = normalize(face_centroids[i] - pos)

        # Evaluate the vector field at the face centroid
        field_value = F(face_centroids[i]...)

        # Compute flux contribution
        net_flux += dot(outward_normal, field_value) * face_areas[i]
    end
    return net_flux
end

function divergence(F::Function, pos::AbstractVector{<:Real}; h=1e-7)
    # F: function taking (x,y,z) and returning a 3-element vector (Fx,Fy,Fz)
    # pos: 3D position as vector
    # h: small step for finite difference

    @assert length(pos) == 3 "Position must be a 3D vector."

    # Helper function for partial derivative of component i w.r.t axis i at pos
    partial_derivative(F, pos, i, h) = (
        F(ntuple(k -> k == i ? pos[k] + h : pos[k], 3)...)[i] -
        F(ntuple(k -> k == i ? pos[k] - h : pos[k], 3)...)[i]
    ) / (2h)

    # Divergence is the sum of partial derivatives: ∂Fx/∂x + ∂Fy/∂y + ∂Fz/∂z
    div = sum(partial_derivative(F, pos, i, h) for i in 1:3)

    return div
end

function curl(F::Function, pos::AbstractVector{<:Real}; h=1e-7)
    # F: function taking (x,y,z) and returning a 3-element vector (Fx,Fy,Fz)
    # pos: 3D position as vector
    # h: small step for finite difference

    # Compute partial derivatives needed for curl:
    @assert length(pos) == 3 "Position must be a 3D vector."

    # Helper function for partial derivative of component j w.r.t axis i at pos
    partial_derivative(F, pos, i, j, h) = (
        F(ntuple(k -> k == i ? pos[k] + h : pos[k], 3)...)[j] -
        F(ntuple(k -> k == i ? pos[k] - h : pos[k], 3)...)[j]
    ) / (2h)

    # Curl components:
    curl_x = partial_derivative(F, pos, 2, 3, h) - partial_derivative(F, pos, 3, 2, h)  # dFz/dy - dFy/dz
    curl_y = partial_derivative(F, pos, 3, 1, h) - partial_derivative(F, pos, 1, 3, h)  # dFx/dz - dFz/dx
    curl_z = partial_derivative(F, pos, 1, 2, h) - partial_derivative(F, pos, 2, 1, h)  # dFy/dx - dFx/dy

    return Vec3{Float64}(curl_x, curl_y, curl_z)
end




function path_integral(F::Function, curve::AbstractVector{<:AbstractVector{<:Real}})
    # F: function taking (x,y,z) and returning a 3-element vector (Fx,Fy,Fz)
    # curve: vector of 3D points defining the curve

    @assert length(curve) > 1 "Curve must have at least two points."
    @assert all(length.(curve) .== 3) "Each point in the curve must be a 3D vector."

    integral = 0.0
    for i in 1:(length(curve) - 1)
        p1 = curve[i]
        p2 = curve[i + 1]
        
        # Midpoint for better approximation
        midpoint = (p1 + p2) / 2
        field_value = F(midpoint...)
        
        # dr is the differential path element (p2 - p1)
        dr = p2 - p1
        integral += dot(field_value, dr)
    end

    return integral
end

function path_integral(F::Function, curve::Function, t_start::Real, t_end::Real; num_points::Int=100)
    @assert t_start < t_end "Start time must be less than end time."
    @assert num_points > 0 "Number of points must be a positive integer."

    # Generate points along the curve
    t_values = range(t_start, stop=t_end, length=num_points)
    curve_points = [curve(t) for t in t_values]

    # Calculate the path integral along the curve
    integral = path_integral(F, curve_points)
    return integral
end

function path_integral(F::Function; centre::AbstractVector{<:Real}, radius::Real, upvector::AbstractVector{<:Real}, num_points::Int=100)
    @assert length(centre) == 3 "Centre must be a 3D vector."
    @assert radius > 0 "Radius must be a positive number."
    @assert length(upvector) == 3 "Upvector must be a 3D vector."
    @assert num_points > 0 "Number of points must be a positive integer."

    # Normalize the upvector
    up = normalize(upvector)
    
    # Create two orthogonal vectors in the plane perpendicular to upvector
    # Choose an arbitrary vector that's not parallel to up
    temp = abs(up[1]) < 0.9 ? Vec3{Float64}(1, 0, 0) : Vec3{Float64}(0, 1, 0)
    u = normalize(cross(up, temp))
    v = cross(up, u)  # This is already normalized since up and u are orthonormal

    # Generate points on the circle in the plane defined by upvector
    theta = range(0, stop=2 * pi, length=num_points + 1)[1:end-1]  # Exclude the duplicate end point
    circle_points = [centre + radius * (cos(t) * u + sin(t) * v) for t in theta]
    
    # Close the curve by adding the first point at the end
    push!(circle_points, circle_points[1])

    # Calculate the line integral along the circle
    integral = line_integral(F, circle_points)

    return integral
end