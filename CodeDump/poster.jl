println("Starting DragonFly Simulation...\n\tBe patient, this might take a while to load the models and packages...\n")
# Import necessary packages
println("Loading packages...")
using GLMakie, GeometryBasics, LinearAlgebra, Quaternions, FileIO
import Quaternions: Quaternion as Quaternion
using Statistics: mean
println("Packages loaded successfully.")

cd(@__DIR__) # Changing the current working directory from project root to the parent directory of this file so that pwd() and @__DIR__ match.
# Import custom code and utility functions
include("../src/State_and_Conversions.jl")
include("../src/Rendering.jl")
include("../src/Transformations.jl")
include("../src/WindowManager.jl")
include("../src/Recorder.jl")
include("../src/fluidflow.jl")

cd(@__DIR__) # Ensure the current working directory is set correctly for subsequent operations



# Initialize the custom structs and utilities
conversions = Conversions()
renderer = Renderer()
transformations = Transformations()
windowmanager = WindowManager()

println("Custom structs and utilities initialized successfully.")


vortex_line = VortexLine(Point3{Float64}(0, 0, -1), Point3{Float64}(0, 0, 1), 50.0, 0.5)
F(x,y,z) = velocity(Point3{Float64}(x, y, z), vortex_line)

s, m = visualize_vector_field(F; bounds=[1.5, 1.5, 1.5], resolution=[7, 7, 7], base=3, scale=0.3)

# for i in eachindex(m)
#     s[i].color[] = GLMakie.RGBA{Float64}(s[i].color[].r, s[i].color[].g, s[i].color[].b, 0.75)  # Set alpha to 0.5 for semi-transparency
#     s[i].transparency[] = false
# end
meshscatter!(s, Point3(0.0), marker=Arrow(vortex_line.R1, vortex_line.R2).mesh, color=:black, markersize=1, label="Vortex Line")

# Plot radial lines for contour 
generate_circle_points = (r, θ) -> Point3{Float64}(r * cos(θ), r * sin(θ), 0)
for r in range(0.1, 1.5, length=3)
    circle_points = [generate_circle_points(r, θ) for θ in range(0, stop=2π, length=100)]
    for i in range(-0, 0, length=1)
        vertical_offset = Point3{Float64}(0, 0, i)
        circle_points_offset = [p + vertical_offset for p in circle_points]
        lines!(s, circle_points_offset, color=:red, linewidth=1, label="Radial Lines")
    end
end


save("vortex_line_visualization_poster.png", s, px_per_unit=10)  # Save the visualization as an image
delete!(s, s[end])


s = Scene(camera=cam3d!)
display(s)
v = vertices_generator([0.0, 0.0, 0.0], 4; radius=1.0, angleoffset=pi/4, upvector=[0.0, 0.0, 1.0])
loop = VortexLoop(v, 100.0, 0.05, 1.0)

for vortexline in loop.vortexlines
    meshscatter!(s, Point3(0.0,0.0,0.0), markersize=1, color=:black, marker=Arrow(vortexline.R1, vortexline.R2, shaft_radius=loop.core_radius).mesh)
end

# Visualize velocity field 
F(x,y,z) = velocity(Point3{Float64}(x, y, z), loop)
visualize_vector_field!(s, F, bounds=2, resolution=7, scale=0.2, base=3)
save("vortex_loop_visualization_poster.png", s, px_per_unit=10)  # Save the visualization as an image



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



F1(x,y,z) = velocity(Point3{Float64}(x, y, z), Point3{Float64}(0.0, 0.0, 0.0), Point3{Float64}(0.0, 0.0, 1.0), 100.0)
core_radius = 1.0 # Define the core radius for the velocity field adjustment
F_adjusted(x,y,z) = velocity(Point3{Float64}(x, y, z), Point3{Float64}(0.0, 0.0, 0.0), Point3{Float64}(0.0, 0.0, 1.0), 100.0, core_radius)
x_vals = exponential_spacing(-5, 5, 1000; base=1)
y_vals = [norm(F1(x, 0, 0)) for x in x_vals]
y_adjusted = [norm(F_adjusted(x, 0, 0)) for x in x_vals]
padding = 10 # Padding for the y-axis limits

# GLMakie.Makie.inline!(true)
set_theme!(theme_minimal())
set_theme!(theme_latexfonts())
fig = Figure(fontsize=24)
ax = Axis(fig[1, 1],
    title="Plot to compare the strength of original and adjusted velocity fields\nas a function of radial distance from the vortex line",
    xlabel="x",
    ylabel="Norm of F1(x, 0, 0)",
    xticks = ([-core_radius, core_radius], ["Core Radius", "Core Radius"]),
    )

lines!(
    ax,
    x_vals, 
    y_vals ,
    label="Original Velocity Field",
    color=:blue,
    linestyle=:dash
    )

lines!(
    ax,
    x_vals,
    y_adjusted,
    label="Adjusted Velocity Field",
    color=:red)

axislegend()

# Plot a line parallel to the y-axis for x=±core_radius
vlines!(
    ax,
    [-core_radius, core_radius],
    linestyle=:dot,
    color=:black,
)

# Add x axis tick with label at core_radius
# Plot a point with a label on the x-axis at ± core_radius

ylims!(ax, minimum(y_adjusted), maximum(y_adjusted)+padding)

display(fig)

save("velocity_field_comparison_poster.png", fig.scene, px_per_unit=10)  # Save the plot as an image