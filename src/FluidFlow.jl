## Vector Field Visualization
println("Loading packages...")
using GLMakie, GeometryBasics, LinearAlgebra
println("Packages loaded successfully.")

cd(@__DIR__) # Changing the current working directory from project root to the parent directory of this file so that pwd() and @__DIR__ match.
# Import custom code and utility functions
include("../src/Rendering.jl")


# Initialize the custom structs and utilities
renderer = Renderer()


function exponential_spacing(min_val::Real, max_val::Real, resolution::Int; base::Real=ℯ)
    if resolution == 1
        return [(min_val + max_val) / 2]
    end
    if base < 0
        throw(ArgumentError("Base must be greater than 0 for exponential spacing."))
    end
    if base == 1
        return range(min_val, max_val, length=resolution) # Linear spacing if base is 1
    end
    
    # Create linear range from min to max
    linear_points = range(min_val, max_val, length=resolution)
    
    # Define casewise exponential function with custom base
    function f(x)
        if x > 0
            # For positive values: exponential decay towards zero
            # Maps [0, max] to [0, max] with more points near zero
            return max_val * (base^(x/max_val) - 1) / (base - 1)
        elseif x < 0
            # For negative values: exponential decay towards zero
            # Maps [min, 0] to [min, 0] with more points near zero
            return min_val * (base^(-x/(-min_val)) - 1) / (base - 1)
        else
            # x == 0
            return 0.0
        end
    end
    
    # Apply the exponential transformation
    exponential_points = [f(x) for x in linear_points]
    
    return exponential_points
end

function visualize_vector_field!(
    scene::Scene,     
    field::Function;
    bounds::Union{AbstractVector{<:AbstractVector{<:Real}}, AbstractVector{<:Real}, Real}=[[-5, 5], [-5, 5], [-5, 5]], # example: [[-10, 10], [-10, 10], [-10, 10]]
    resolution::Union{Int, AbstractVector{<:Int}}=5,
    color::Union{GLMakie.Colorant, Nothing}=nothing,
    scale::Real=1.0,
    custom_arrows::Bool=false,
    base::Real=1.0,
    center::Point3{Float64}=Point3{Float64}(0.0, 0.0, 0.0), # Center of the scene for camera positioning,
    normalization_condition::Bool=true # Normalize the vector field if true
    )

    if typeof(bounds) <: Real
        @assert bounds > 0 "Bounds must be a positive number when it's a single value for a symmetric visualization."
        bounds = [[-bounds, bounds], [-bounds, bounds], [-bounds, bounds]] # Convert single Real to 3D bounds
    elseif typeof(bounds) <: AbstractVector{<:Real}
        @assert length(bounds) == 3 "Bounds must be a vector of 3 AbstractVectors: x, y, and z bounds."
        @assert all(x -> x> 0, bounds) "Each bound must be a positive number or a vector of two positive numbers."
        bounds = [[-bounds[1], bounds[1]], [-bounds[2], bounds[2]], [-bounds[3], bounds[3]]] # Convert 2D to 3D bounds
    end

    # Assertions to ensure the bounds are correctly defined
    for bound in bounds
        @assert length(bound) == 2 "Each bound must be an AbstractVector of length 2: start and end. (e.g., [-10, 10])"
    end
    @assert length(bounds) == 3 "Bounds must be a vector of 3 AbstractVectors: x, y, and z bounds."
    @assert typeof(resolution) <: Integer || (typeof(resolution) <: AbstractVector && all(x -> x > 0, resolution) && length(resolution) == 3) "Resolution must be a positive integer or a vector of positive integers, corresponding to the x, y, and z dimensions."

    # Create a grid of points in the specified bounds
    if typeof(resolution) <: Integer
        resolution = [resolution, resolution, resolution] # Convert to a vector of 3 integers
    end
    
    # Fix: Ensure consistent array types for all dimensions
    x = resolution[1] != 1 ? exponential_spacing(bounds[1][1], bounds[1][2], resolution[1]; base=base) : [(bounds[1][1] + bounds[1][2])/2]
    y = resolution[2] != 1 ? exponential_spacing(bounds[2][1], bounds[2][2], resolution[2]; base=base) : [(bounds[2][1] + bounds[2][2])/2]
    z = resolution[3] != 1 ? exponential_spacing(bounds[3][1], bounds[3][2], resolution[3]; base=base) : [(bounds[3][1] + bounds[3][2])/2]

    x = [center[1] + xi for xi in x] # Center the grid around the specified center point
    y = [center[2] + yi for yi in y] # Center the grid around the specified center point
    z = [center[3] + zi for zi in z] # Center the grid around the specified center point

    # Generate the vector field
    # Fix: Correct NaN checking and mapping function
    mapping(i) = isnan(i) ? 1 : clamp(round(Int, 255*2*atan(i/2)/pi) + 1, 1, 256)
    viridis = GLMakie.cgrad(:viridis) # Creates a color map (a list of colors of length 256 with colors ranging from blue to yellow)
    
    arrows = Vector{Arrow}()
    arrow_colors = Vector{GLMakie.Colorant}()
    for xi in x, yi in y, zi in z
        f = field(xi, yi, zi)
        arrow_length = norm(f)
        
        # Fix: Handle zero-length vectors properly
        if isapprox(arrow_length*scale, 0.0, atol=1e-10) || isnan(arrow_length)
            # Skip zero-length vectors or use a default direction
            continue
        end
        if custom_arrows
            arrow_to_body_ratio = (1 - exp(-sqrt(arrow_length)) + 2 / 3 + exp(-1 / sqrt(arrow_length)) / 3) / 2
            shaft_length = Float64(arrow_to_body_ratio) * arrow_length
            head_length = arrow_length - shaft_length
            shaft_radius = head_length / 5 / 7.5
            head_radius = shaft_radius * 1.75
            push!(arrows, Arrow(Point3{Float64}(xi, yi, zi), normalization_condition == true ? normalize(Vec3{Float64}(f)).*(scale*2*atan(arrow_length/2)/pi) : Vec3{Float64}(f).*scale, shaft_radius=shaft_radius, head_radius=head_radius, arrow_to_body_ratio=0.75))
        else
            push!(arrows, Arrow(Point3{Float64}(xi, yi, zi), normalization_condition == true ? normalize(Vec3{Float64}(f)).*(scale*2*atan(arrow_length/2)/pi) : Vec3{Float64}(f).*scale))
        end
        if isnothing(color)
            push!(arrow_colors, viridis[mapping(arrow_length)])
        else
            push!(arrow_colors, color)
        end
    end

    arrow_meshes = []
    for i in eachindex(arrows)
        m = mesh!(scene, arrows[i].mesh; color=arrow_colors[i])
        push!(arrow_meshes, m)
    end

    display(scene)
    return arrow_meshes
end

# Define the vector field visualization function
"""
    visualize_vector_field(field::Function; bounds=[[-5, 5], [-5, 5], [-5, 5]], resolution=5, color=nothing, scale=1.0)
"""
function visualize_vector_field(
    field::Function;
    bounds::Union{AbstractVector{<:AbstractVector{<:Real}}, AbstractVector{<:Real}, Real}=[[-5, 5], [-5, 5], [-5, 5]], # example: [[-10, 10], [-10, 10], [-10, 10]]
    resolution::Union{Int, AbstractVector{<:Int}}=5,
    color::Union{GLMakie.Colorant, Nothing}=nothing,
    scale::Real=1.0,
    custom_arrows::Bool=false,
    base::Real=1.0,
    center::Point3{Float64}=Point3{Float64}(0.0, 0.0, 0.0), # Center of the scene for camera positioning,
    normalization_condition::Bool=true # Normalize the vector field if true
    )

    # Create a scene and add the arrows
    scene = Scene(camera=cam3d!)
    arrow_meshes = visualize_vector_field!(
        scene,
        field;
        bounds=bounds,
        resolution=resolution,
        color=color,
        scale=scale,
        custom_arrows=custom_arrows,
        base=base,
        center=center,
        normalization_condition=normalization_condition
    )

    return scene, arrow_meshes
end

println("Vector field visualization function defined successfully.")


function velocity(r::AbstractVector{<:Real}, R1::AbstractVector{<:Real}, R2::AbstractVector{<:Real}, Gamma::Real, core_radius::Real=0.0)
    #https://www.tfd.chalmers.se/~lada/postscript_files/Licentiate_thesis_Hamid.pdf
    @assert length(r) == 3 "Position vector r must be a 3D vector."
    @assert length(R1) == 3 "Position vector R1 must be a 3D vector."
    @assert length(R2) == 3 "Position vector R2 must be a 3D vector."

     
    # Apply core correction
    # If r is within the core_radius of the line Joining R1 and R2, assume a velocity to be a linear function of the radial distance in that direction (zero at norm(r)=0 and equal to the usual velocity at norm(r)=core_radius)
    
    function point_projection_on_line(point::AbstractVector{<:Real}, line_start::AbstractVector{<:Real}, line_end::AbstractVector{<:Real})
        # Project point onto the line defined by line_start and line_end
        line_dir = normalize(line_end - line_start)
        return line_start + dot(point - line_start, line_dir) * line_dir
    end
    
    distance_to_line = norm(r - point_projection_on_line(r, R1, R2))
    if distance_to_line < core_radius
        
        # Calculate the direction from the line to the point
        direction = normalize(r - point_projection_on_line(r, R1, R2))
        
        # Let v be the velocity at core_radius
        v = velocity(direction * core_radius, R1, R2, Gamma, core_radius)
        
        # Scale the velocity by the ratio of distance to core radius
        return v * (distance_to_line / core_radius)
    end

    R1r = r - R1
    R2r = r - R2

    K12 = dot(-Gamma * (R2 - R1) / (4 * pi * norm(cross(R1r, R2r))^2), (normalize(R2r) - normalize(R1r)))
   
    v = K12 * cross(R1r, R2r)

    return v
end

mutable struct VortexLine
    R1::AbstractVector{<:Real}  # Position of the first vortex
    R2::AbstractVector{<:Real}  # Position of the last vortex
    Gamma::Real                 # Circulation strength
    core_radius::Real           # Core radius for the velocity field adjustment
    
    # Inner constructor with validation
    function VortexLine(R1::AbstractVector{<:Real}, R2::AbstractVector{<:Real}, Gamma::Real, core_radius::Real=0.001)
        @assert length(R1) == 3 "Position vector R1 must be a 3D vector."
        @assert length(R2) == 3 "Position vector R2 must be a 3D vector."
        @assert core_radius >= 0 "Core radius must be non-negative."
        new(R1, R2, Gamma, core_radius)
    end
end

mutable struct VortexLoop
    vertices::Vector{<:AbstractVector{<:Real}}  # List of vertices defining the loop
    Gamma::Real                                 # Circulation strength
    core_radius::Real                           # Core radius for the velocity field adjustment
    direction::Real                             # Direction of the loop
    vortexlines::Vector{VortexLine}             # List of vortex lines forming the loop
end
# Define Constructors
function VortexLoop(vertices::Vector{<:AbstractVector{<:Real}}, Gamma::Real, core_radius::Real=0.001, direction::Real=1.0)
    @assert all(length(v) == 3 for v in vertices) "All vertices must be 3D vectors."
    @assert core_radius >= 0 "Core radius must be non-negative."
    @assert direction != 0 "The direction must define a sign (positive to traverse vertices in the order given, negative to traverse in the opposite order)."

    # Create vortex lines from the vertices
    vortexlines = Vector{VortexLine}()
    if direction > 0
        for i in 1:length(vertices)-1
            push!(vortexlines, VortexLine(vertices[i], vertices[i+1], Gamma, core_radius))
        end
        # Connect the last vertex to the first to close the loop
        push!(vortexlines, VortexLine(vertices[end], vertices[1], Gamma, core_radius))
    else
        for i in length(vertices):-1:2
            push!(vortexlines, VortexLine(vertices[i], vertices[i-1], Gamma, core_radius))
        end
        # Connect the first vertex to the last to close the loop
        push!(vortexlines, VortexLine(vertices[1], vertices[end], Gamma, core_radius))
    end
    return VortexLoop(vertices, Gamma, core_radius, direction, vortexlines)
end
function VortexLoop(vortexlines::Vector{VortexLine})
    vl_copy = deepcopy(vortexlines) # Create a copy of the vortex lines to avoid modifying the original
    # Make sure the vortex lines form a closed loop (i.e. for the entire vector, v[i].R2 == v[i+1].R1)
    @assert all(i -> vl_copy[i].R2 == vl_copy[i+1].R1, 1:length(vl_copy)-1) && vl_copy[end].R2 == vl_copy[1].R1 "Vortex lines must form a closed loop."
    
    # Make sure all vortex lines have the same circulation strength and core radius
    @assert all(vl -> vl.Gamma == vl_copy[1].Gamma, vl_copy) "All vortex lines must have the same circulation strength."
    @assert all(vl -> vl.core_radius == vl_copy[1].core_radius, vl_copy) "All vortex lines must have the same core radius."

    # Extract the vertices from the vortex lines
    vertices = [vl.R1 for vl in vl_copy]

    return VortexLoop(vertices, vl_copy[1].Gamma, vl_copy[1].core_radius, 1.0, vl_copy)
end

function _vertices_generator_radius(centre::AbstractVector{<:Real}, num_vertices::Int; radius::Real, angleoffset::Real=0.0, upvector::AbstractVector{<:Real}=[0.0, 0.0, 1.0])
    @assert length(centre) == 3 "Centre must be a 3D vector."
    @assert length(upvector) == 3 "Up vector must be a 3D vector."
    @assert radius > 0 "Radius must be a positive number."

    # Normalize the up vector
    up = normalize(upvector)
    
    # Create two orthonormal vectors in the plane perpendicular to upvector
    # Choose an arbitrary vector not parallel to up
    if abs(dot(up, [1.0, 0.0, 0.0])) < 0.9
        temp = [1.0, 0.0, 0.0]
    else
        temp = [0.0, 1.0, 0.0]
    end
    
    # Create first basis vector in the plane
    u1 = normalize(cross(up, temp))
    # Create second basis vector in the plane
    u2 = cross(up, u1)
    
    # Generate vertices in circular pattern
    vertices = Vector{Point{3, Float64}}()
    
    for i in 0:(num_vertices-1)
        angle = angleoffset + 2π * i / num_vertices - π/2  # Start at the [1,0,0] point by default
        
        # Calculate position in the plane
        local_pos = radius * (cos(angle) * u1 + sin(angle) * u2)
        
        # Translate to the centre
        vertex = centre + local_pos
        
        push!(vertices, vertex)
    end
    
    return vertices
end
function _vertices_generator_side(centre::AbstractVector{<:Real}, num_vertices::Int; side::Real, angleoffset::Real=0.0, upvector::AbstractVector{<:Real}=[0.0, 0.0, 1.0])
    @assert length(centre) == 3 "Centre must be a 3D vector."
    @assert length(upvector) == 3 "Up vector must be a 3D vector."
    @assert side > 0 "Side length must be a positive number."

    # Calculate vertex radial distance from centre if side length is given 
    radius = side / (2 * sin(π / num_vertices))  # Radius of the circumscribed circle for the polygon
    return vertices_generator(centre, num_vertices; radius=radius, angleoffset=angleoffset, upvector=upvector)
end

function vertices_generator(centre::AbstractVector{<:Real}, num_vertices::Int; side::Union{<:Real, Nothing}=nothing, radius::Union{<:Real, Nothing}=nothing, angleoffset::Real=0.0, upvector::AbstractVector{<:Real}=[0.0, 0.0, 1.0])
    @assert side === nothing || radius === nothing "Only one of side or radius can be provided."

    if side != nothing
        return _vertices_generator_side(centre, num_vertices; side=side, angleoffset=angleoffset, upvector=upvector)
    elseif radius != nothing
        return _vertices_generator_radius(centre, num_vertices; radius=radius, angleoffset=angleoffset, upvector=upvector)
    else
        # Use side=1.0 as default if neither side nor radius is provided
        return _vertices_generator_radius(centre, num_vertices; radius=1.0, angleoffset=angleoffset, upvector=upvector)
    end
end

function set_Gamma!(loop::VortexLoop, new_Gamma::Real)
    loop.Gamma = new_Gamma
    for vortexline in loop.vortexlines
        vortexline.Gamma = new_Gamma
    end
end
function set_Gamma!(loops::Vector{VortexLoop}, new_Gamma::Union{<:Real, <:AbstractVector{<:Real}})
    if typeof(new_Gamma) <: Real
        for loop in loops
            set_Gamma!(loop, new_Gamma)
        end
    elseif typeof(new_Gamma) <: AbstractVector{<:Real}
        @assert length(new_Gamma) == length(loops) "The length of new_Gamma must match the number of loops."
        for (loop, gamma) in zip(loops, new_Gamma)
            set_Gamma!(loop, gamma)
        end
    else
        throw(ArgumentError("new_Gamma must be either a Real or an AbstractVector of Reals."))
    end
end

function velocity(r::AbstractVector{<:Real}, vortexline::VortexLine)
    # Calculate the velocity at point r due to a single vortex line
    return velocity(r, vortexline.R1, vortexline.R2, vortexline.Gamma, vortexline.core_radius)
end
function velocity(r::AbstractVector{<:Real}, vortexloop::VortexLoop)
    # Calculate the total velocity at point r due to a vortex loop
    v_total = zeros(3)
    for vortexline in vortexloop.vortexlines
        v_total += velocity(r, vortexline)
    end
    return v_total
end
function velocity(r::AbstractVector{<:Real}, vortexloops::Vector{VortexLoop})
    # Calculate the total velocity at point r due to multiple vortex loops
    return sum(velocity(r, loop) for loop in vortexloops)
end

function calculate_path_in_field(F::Function, p0::AbstractVector{<:Real}, dt::Real, steps::Int, path::Observable{<:AbstractVector{<:Real}}, render_speed::Real=2.0)
    # Implementing a Runge-Kutta 4th order method to calculate the path in the vector field    
    p = p0
    for _ in 1:steps
        k1 = F(p[1], p[2], p[3])
        k2 = F(p[1] + dt/2 * k1[1], p[2] + dt/2 * k1[2], p[3] + dt/2 * k1[3])
        k3 = F(p[1] + dt/2 * k2[1], p[2] + dt/2 * k2[2], p[3] + dt/2 * k2[3])
        k4 = F(p[1] + dt * k3[1], p[2] + dt * k3[2], p[3] + dt * k3[3])
        p += (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)
        push!(path[], Point3{Float64}(p))
        notify(path)
        sleep(dt/render_speed) # Sleep for dt seconds to simulate time passing
    end
    nothing
end


function prefactor(r::AbstractVector{<:Real}, R1::AbstractVector{<:Real}, R2::AbstractVector{<:Real}, core_radius::Real)
    #https://www.tfd.chalmers.se/~lada/postscript_files/Licentiate_thesis_Hamid.pdf
    return velocity(r, R1, R2, 1.0, core_radius)
end
function prefactor(r::AbstractVector{<:Real}, vortexline::VortexLine)
    # Calculate the prefactor at point r due to a single vortex line
    return prefactor(r, vortexline.R1, vortexline.R2, vortexline.core_radius)
end
function prefactor(r::AbstractVector{<:Real}, vortexloop::VortexLoop)
    # Calculate the total prefactor at point r due to a vortex loop
    v_total = zeros(3)
    for vortexline in vortexloop.vortexlines
        v_total += prefactor(r, vortexline)
    end
    return v_total
end

function calculate_normal_vector(loop::VortexLoop)
    # Calculate the normal vector for the loop based on its vertices
    @assert length(loop.vertices) >= 3 "Loop must have at least 3 vertices to define a plane."
    v1 = loop.vertices[2] - loop.vertices[1]
    v2 = loop.vertices[3] - loop.vertices[1]
    n = normalize(cross(v1, v2))  # Normal vector using cross product
    return n
end

function calculate_normal_vector(loops::Vector{VortexLoop})
    # Calculate the normal vectors for each loop in the vector of loops
    return [calculate_normal_vector(loop) for loop in loops]
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
    integral = path_integral(F, circle_points)  # Changed from line_integral to path_integral

    return integral
end