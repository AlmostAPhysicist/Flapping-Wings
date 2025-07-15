## Vector Field Visualization
println("Loading packages...")
using GLMakie, GeometryBasics, LinearAlgebra
println("Packages loaded successfully.")

cd(@__DIR__) # Changing the current working directory from project root to the parent directory of this file so that pwd() and @__DIR__ match.
# Import custom code and utility functions
include("../src/Rendering.jl")
include("../src/FluidFlow.jl")


# Initialize the custom structs and utilities
renderer = Renderer()


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
    x = resolution[1] != 1 ? range(bounds[1][1], bounds[1][2], length=resolution[1]) : [(bounds[1][1] + bounds[1][2])/2]
    y = resolution[2] != 1 ? range(bounds[2][1], bounds[2][2], length=resolution[2]) : [(bounds[2][1] + bounds[2][2])/2]
    z = resolution[3] != 1 ? range(bounds[3][1], bounds[3][2], length=resolution[3]) : [(bounds[3][1] + bounds[3][2])/2]

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
        if arrow_length == 0.0 || isnan(arrow_length)
            # Skip zero-length vectors or use a default direction
            continue
        end
        
        push!(arrows, Arrow(Point3{Float64}(xi, yi, zi), normalize(Vec3{Float64}(f)).*(scale*2*atan(arrow_length/2)/pi)))
        if isnothing(color)
            push!(arrow_colors, viridis[mapping(arrow_length)])
        else
            push!(arrow_colors, color)
        end
    end

    # Create a scene and add the arrows
    scene = Scene(camera=cam3d!)
    arrow_meshes = []
    for i in eachindex(arrows)
        m = mesh!(scene, arrows[i].mesh; color=arrow_colors[i])
        push!(arrow_meshes, m)
    end

    display(scene)
    return scene, arrow_meshes
end



# ----------------------------------------------------------------------------



include("../src/FluidFlow.jl")
F(x,y,z) = Vec3{Float64}(x, y, z) # Example vector field function
F(x,y,z) = Vec3{Float64}(x*y,z,1.0)
F(x,y,z) = Vec3{Float64}(velocity(
    [x, y, z], 
    [0.0, 0.0, -100.0], 
    [0.0, 0.0, 100.0], 
    100.0
))

begin
s, arrow_meshes = visualize_vector_field(F; resolution=[15,15,5], bounds=[5,5,3], scale=1) # Toggle the third element of the resultion between 1 and 5
nothing
end
# meshscatter!(s, Point3{Float64}(0, 0, 0), marker=:Sphere, markersize=0.1, color=:black, space=:data)
meshscatter!(s, Point3{Float64}(0, 0, 0), marker=Arrow(Point3([0.0, 0.0, -1.0]), Point3([0.0, 0.0, 1.0])).mesh, markersize=1, color=:black, space=:data)

cc = Makie.cameracontrols(s)
update_cam!(s, cc, Vec3f(0.0, 0.0, 15.0), Vec3f(0.0,0.0,0.0), Vec3f(0.0,1.0,0.0)) 
# Sets camera position, lookat and upvector

save("vector_field_visualization.png", s, size=(1920, 1080))


# @async record(s, "vector_field_visualization.mp4", framerate=30) do io
#     for i in 1:30*10 # Record for 10 seconds at 30 FPS
#         recordframe!(io)
#         sleep(1/30)
#     end
# end




# ----------------------------------------------------------------------------

p = Point3(rand(3)) # Example point in 3D space
meshscatter!(s, p, marker=:Sphere, markersize=0.1, color=:black, space=:data) # Add a point to the scene

divergence(F, p) # Example divergence calculation
curl(F, p) # Example curl calculation

line_integral(F, Point3(0.0, 0.0, 0.0), 1.0, Point3(0.0,0.0,1.0), 100) # Example line integral 

# Let's verify the Theorem of Constant Contour Circulation.
# Simulate a point moving along the circle for the velocity function F and save the path in `path` for runge-kutta 4
# Implement calculation of position for velocity function F using Runge-Kutta 4
p0 = Point3{Float64}(2.0, 0.0, 0.0) # Initial position
path = Observable([p0]) # Observable to store the path
function runge_kutta_4(F, p0, dt, steps, path)
    p = p0
    for _ in 1:steps
        k1 = F(p[1], p[2], p[3])
        k2 = F(p[1] + dt/2 * k1[1], p[2] + dt/2 * k1[2], p[3] + dt/2 * k1[3])
        k3 = F(p[1] + dt/2 * k2[1], p[2] + dt/2 * k2[2], p[3] + dt/2 * k2[3])
        k4 = F(p[1] + dt * k3[1], p[2] + dt * k3[2], p[3] + dt * k3[3])
        p += (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)
        push!(path[], Point3{Float64}(p))
        notify(path)
        sleep(dt) # Sleep for dt seconds to simulate time passing
    end
    nothing
end
s
lines!(s, path; color=:red, linewidth=5, space=:data, fxaa=true) # Add the path to the scene
runge_kutta_4(F, p0, 0.1, 100, path) # Run the simulation for 100 steps with a time step of 0.1

# ----------------------------------------------------------------------------
# Example usage of the custom structs and functions

# Create a VortexLine
vortex_line = VortexLine(Float64[0, 0, -1], Float64[0, 0, 1], 10.0)
F(x,y,z) = velocity([x, y, z], vortex_line)
# Visualize the vector field generated by the VortexLine
begin
s, arrow_meshes = visualize_vector_field(F; resolution=[15,15,7], bounds=[3,3,2], scale=1.0)
# Add VortexLines associated with the VortexLine
    meshscatter!(s, Point3{Float64}(0.0,0.0,0.0), marker=Arrow(Point3(vortex_line.R1), Point3(vortex_line.R2)).mesh, markersize=1, color=:red, space=:data)
nothing
end
# ----------------------------------------------------------------------------

# add another VortexLine for visualization
vortex_line1 = VortexLine(Float64[1, 0, -1], Float64[1, 0, 1], 10.0)
vortex_line2 = VortexLine(Float64[-1, 0, -1], Float64[-1, 0, 1], -10.0)
F(x,y,z) = velocity([x, y, z], vortex_line1) + velocity([x, y, z], vortex_line2)

# Visualize the vector field generated by the two VortexLines
begin
s, arrow_meshes = visualize_vector_field(F; resolution=[15,15,7], bounds=[3,3,2], scale=0.75)
# Add VortexLines associated with the VortexLines
    meshscatter!(s, Point3{Float64}(0.0,0.0,0.0), marker=Arrow(Point3(vortex_line1.R1), Point3(vortex_line1.R2)).mesh, markersize=1, color=:red, space=:data)
    meshscatter!(s, Point3{Float64}(0.0,0.0,0.0), marker=Arrow(Point3(vortex_line2.R1), Point3(vortex_line2.R2)).mesh, markersize=1, color=:blue, space=:data)
nothing
end
# ----------------------------------------------------------------------------

# Create a VortexLoop 
vortex_loop = VortexLoop(Float64[0, 0, 0], 1.0, 100.0, Float64[0, 0, 1], 4, 1)
F(x,y,z) = velocity([x, y, z], vortex_loop)

F(10,0,0)-F(11, 0, 0) # Check the velocity at two points far away from the vortex loop
# Visualize the vector field generated by the VortexLoop
begin
s, arrow_meshes = visualize_vector_field(F; resolution=[12,12,5], bounds=[10,10,2], scale=0.5)

# Add VortexLines associated with the VortexLoop
for vortex in vortex_loop.vortices
    meshscatter!(s,  Point3{Float64}(0.0,0.0,0.0), marker=Arrow(Point3(vortex.R1), Point3(vortex.R2)).mesh, markersize=1, color=:red, space=:data)
end

nothing
end
begin
s, arrow_meshes = visualize_vector_field(F; resolution=[12,1,7], bounds=[1.5,1.5,2], scale=0.5) # Plot a single Plane
# s, arrow_meshes = visualize_vector_field(F; resolution=[12,5,5], bounds=[1.5,1.5,2], scale=0.5) # Add more layers

# Add VortexLines associated with the VortexLoop
for vortex in vortex_loop.vortices
    meshscatter!(s,  Point3{Float64}(0.0,0.0,0.0), marker=Arrow(Point3(vortex.R1), Point3(vortex.R2)).mesh, markersize=1, color=:red, space=:data)
end

nothing
end

# ----------------------------------------------------------------------------
# Create a VortexRibbon

begin
vortex_ribbon = VortexRibbon(Float64[0, 0, 0], 1.0, 100.0, Float64[0, 0, 1], 4, 4, Dict(:nlines=>3, :ngon=>12))
F(x,y,z) = velocity([x, y, z], vortex_ribbon)
end

# Visualize the vector field generated by the VortexRibbon
begin
    s, arrow_meshes = visualize_vector_field(F; resolution=[25,25,1], bounds=[3,3,0.75], scale=0.25)
    # Add VortexLines associated with the VortexRibbon
    for loop in vortex_ribbon.loops
        for vortex in loop.vortices
            meshscatter!(s, Point3{Float64}(vortex.R1...), marker=Arrow(Point3(vortex.R1), Point3(vortex.R2)).mesh, markersize=1, color=:red, space=:data)
        end
    end
    nothing
end
# ----------------------------------------------------------------------------

# Add a camera control for the scene
cc = Makie.cameracontrols(s)
update_cam!(s, cc, Vec3f(0.0, 0.0, 15.0), Vec3f(0.0,0.0,0.0), Vec3f(0.0,1.0,0.0))
# Sets camera position, lookat and upvector


# For each vortex in the VortexRibbon, if velocity([0,0,0], vortex) is positive, add a red arrow, otherwise add a blue arrow
v_sum = Vec3{Float64}(0.0, 0.0, 0.0)
for loop in vortex_ribbon.loops
    for vortex in loop.vortices
        v = velocity([0.0, 0.0, 0.0], vortex)
        color = sum(v.data)>0 ? :red : :blue
        meshscatter!(s, Point3{Float64}(vortex.R1...), marker=Arrow(Point3(vortex.R1), Point3(vortex.R2)).mesh, markersize=1, color=color, space=:data)
        v_sum += v
    end
end
v_sum


sum([vortex_ribbon.loops[1].vortices[i].R1 for i in 1:length(vortex_ribbon.loops[1].vortices)]) / length(vortex_ribbon.loops[1].vortices) - vortex_ribbon.loops[1].vortices[1].R1

copy_loop_inner = VortexLoop(Float64[0, 0, 0], 1.0-0.16018862050852034, -100.0, Float64[0, 0, 1], 4,4)
for line in copy_loop_inner.vortices
    meshscatter!(s, Point3{Float64}(line.R1...), marker=Arrow(Point3(line.R1), Point3(line.R2)).mesh, markersize=1, color=:green, space=:data)
end
copy_loop_outer = VortexLoop(Float64[0, 0, 0], 1.0+0.16018862050852034, 100.0, Float64[0, 0, 1], 4, 4)
for line in copy_loop_outer.vortices
    meshscatter!(s, Point3{Float64}(line.R1...), marker=Arrow(Point3(line.R1), Point3(line.R2)).mesh, markersize=1, color=:green, space=:data)
end


vel_inner(x,y,z) = velocity([x,y,z], copy_loop_inner)
vel_outer(x,y,z) = velocity([x,y,z], copy_loop_outer)
vel_ribbon(x,y,z) = velocity([x,y,z], vortex_ribbon)
vel_inner_outer(x,y,z) = vel_inner(x,y,z) + vel_outer(x,y,z)

diff_vel(x,y,z) = vel_inner_outer(x,y,z) - vel_ribbon(x,y,z)

vel_ribbon(0,0,0) # Check the velocity at the origin for the VortexRibbon
vel_inner_outer(0,0,0) # Check the velocity at the origin for the inner and outer loops combined


# Visualize the difference in velocity between the inner and outer loops and the ribbon
begin
    s, arrow_meshes = visualize_vector_field(diff_vel; resolution=[15,15,1], bounds=[3,3,1], scale=0.25)
    # Add VortexLines associated with the inner and outer loops
    for line in copy_loop_inner.vortices
        meshscatter!(s, Point3{Float64}(line.R1...), marker=Arrow(Point3(line.R1), Point3(line.R2)).mesh, markersize=1, color=:blue, space=:data)
    end
    for line in copy_loop_outer.vortices
        meshscatter!(s, Point3{Float64}(line.R1...), marker=Arrow(Point3(line.R1), Point3(line.R2)).mesh, markersize=1, color=:red, space=:data)
    end
    nothing
end



# ---------------

dominating_loop = VortexLoop(Float64[2, 0, 0], 1.0, 100.0, Float64[0, 0, 1], 4, 3)
far_away_loop = VortexLoop(Float64[1000, 0, 0], 1.0, 100.0, Float64[0, 0, 1], 4, 3)


# Calculate the velocity at the center of the dominating loop
v_dominating(x,y,z) = velocity([x,y,z], dominating_loop)
v_far_away(x,y,z) = velocity([x,y,z], far_away_loop)
v_net(x,y,z) = v_dominating(x,y,z) + v_far_away(x,y,z)

v_dominating(0,0,0)
v_far_away(0,0,0)
norm(v_net(0,0,0)-v_dominating(0,0,0))/norm(v_dominating(0,0,0))

begin 
    s, arrow_meshes = visualize_vector_field(v_net; resolution=[15,15,5], bounds=[15,15,3], scale=1.0)
    nothing
end


for line in dominating_loop.vortices
    meshscatter!(s, Point3{Float64}(0), marker=Arrow(Point3(line.R1), Point3(line.R2)).mesh, markersize=1, color=:red, space=:data)
end
for line in far_away_loop.vortices
    meshscatter!(s, Point3{Float64}(0), marker=Arrow(Point3(line.R1), Point3(line.R2)).mesh, markersize=1, color=:blue, space=:data)
end


meshscatter!(s, Point3(0,0,0), marker=:Sphere, markersize=0.3, color=:black, space=:data) # Add a point at the origin
