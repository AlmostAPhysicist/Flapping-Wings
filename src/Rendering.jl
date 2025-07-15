using GLMakie, GeometryBasics, LinearAlgebra, Quaternions
import Quaternions: Quaternion as Quaternion
include("State_and_Conversions.jl")

# Function to create an arrow mesh using native Cylinder and Cone
function create_arrow_mesh_native(shaft_length::AbstractFloat, shaft_radius::AbstractFloat, head_length::AbstractFloat, head_base_radius::AbstractFloat, quality::Int=32)
    # Convert to Float32 for consistency
    sl = Float32(shaft_length)
    sr = Float32(shaft_radius)
    hl = Float32(head_length)
    hbr = Float32(head_base_radius)

    # Create the shaft (Cylinder from z=0 to z=shaft_length)
    shaft = Cylinder(Point3f(0, 0, 0), Point3f(0, 0, sl), sr)

    # Create the arrowhead (Cone from z=shaft_length to z=shaft_length+head_length)
    # Cone(; quality) = merge([
    #     Makie._circle(Point3f(0, 0, 0), hbr, Vec3f(0, 0, -1), quality),
    #     Makie._mantle(Point3f(0, 0, 0), Point3f(0, 0, hl), hbr, 0f0, quality)
    # ])
    cone = merge([
        Makie._circle(Point3f(0, 0, 0), hbr, Vec3f(0, 0, -1), quality),
        Makie._mantle(Point3f(0, 0, 0), Point3f(0, 0, hl), hbr, 0f0, quality)
    ])

    # Convert to meshes
    shaft_mesh = GeometryBasics.mesh(shaft)
    cone_mesh = GeometryBasics.mesh(cone)

    # Translate the cone mesh to sit on top of the shaft
    cone_positions = cone_mesh.vertex_attributes[1]
    translated_positions = [Point3f(p[1], p[2], p[3] + sl) for p in cone_positions]
    cone_faces = cone_mesh.faces
    # translated_cone_mesh = GeometryBasics.mesh(translated_positions, cone_faces)

    # Merge the two meshes
    shaft_positions = shaft_mesh.vertex_attributes[1]
    shaft_faces = shaft_mesh.faces
    cone_base_idx = length(shaft_positions) + 1
    merged_positions = vcat(shaft_positions, translated_positions)
    merged_faces = vcat(shaft_faces, [GLTriangleFace(f[1] + cone_base_idx - 1, f[2] + cone_base_idx - 1, f[3] + cone_base_idx - 1) for f in cone_faces])
    merged_mesh = GeometryBasics.mesh(merged_positions, merged_faces)

    return merged_mesh
end

mutable struct Arrow
    tail::Point{3,Float64}
    head::Point{3,Float64}
    shaft_length::Float64
    head_length::Float64
    shaft_radius::Float64
    head_radius::Float64
    quality::Int
    mesh::GeometryBasics.Mesh
end

# Constructor for Arrow given two points
function Arrow(tail::Point{3,<:Real}, head::Point{3,<:Real}; arrow_to_body_ratio::Union{Real,Nothing}=nothing, shaft_radius::Union{Real,Nothing}=nothing, head_radius::Union{Real,Nothing}=nothing, quality::Int=32)
    tail_f = Point{3,Float64}(tail)
    head_f = Point{3,Float64}(head)

    arrow_length = norm(head_f - tail_f)

    if isnothing(arrow_to_body_ratio)
        arrow_to_body_ratio = (1 - exp(-sqrt(arrow_length)) + 2 / 3 + exp(-1 / sqrt(arrow_length)) / 3) / 2
    end
    shaft_length = Float64(arrow_to_body_ratio) * arrow_length
    head_length = arrow_length - shaft_length

    if isnothing(shaft_radius)
        shaft_radius = head_length / 5
    end

    if isnothing(head_radius)
        head_radius = Float64(shaft_radius) * (3 - arrow_to_body_ratio)
    end

    base_mesh = create_arrow_mesh_native(shaft_length, Float64(shaft_radius), head_length, Float64(head_radius), quality)

    direction = head_f - tail_f # Direction vector from tail to head
    length_dir = norm(direction)
    if length_dir < 1e-10
        println("Direction vector has zero length, picking default direction for a small arrow.")
        random_dir = rand(Vec{3,Float64}) .* 1e-5
        return Arrow(tail_f, Point{3,Float64}(random_dir); arrow_to_body_ratio=arrow_to_body_ratio, shaft_radius=shaft_radius, head_radius=head_radius, quality=quality)
    end

    normalized_dir = normalize(direction)

    z_axis = Vec{3,Float64}(0, 0, 1)
    axis = cross(z_axis, Vec{3,Float64}(normalized_dir))
    angle = acos(clamp(dot(z_axis, Vec{3,Float64}(normalized_dir)), -1.0, 1.0))

    if norm(axis) < 1e-10
        if normalized_dir[3] < 0
            rotation = Matrix{Float64}([
                1 0 0
                0 -1 0
                0 0 -1
            ])
        else
            rotation = Matrix{Float64}(I, 3, 3)
        end
    else
        rotation = axisangle2rotmatrix(axis, angle)
    end

    positions = base_mesh.vertex_attributes[1]
    rotated_positions = [Point{3,Float64}(rotation * Vec{3,Float64}(p[1], p[2], p[3])) for p in positions]
    faces = base_mesh.faces
    translated_positions = [tail_f + p for p in rotated_positions]
    translated_mesh = GeometryBasics.mesh(translated_positions, faces)

    return Arrow(tail_f, head_f, shaft_length, head_length, Float64(shaft_radius), Float64(head_radius), quality, translated_mesh)
end

# Constructor for Arrow given a point and a direction vector
function Arrow(tail::Point{3,<:Real}, direction::Vec{3,<:Real}; arrow_to_body_ratio::Union{Real,Nothing}=nothing, shaft_radius::Union{Real,Nothing}=nothing, head_radius::Union{Real,Nothing}=nothing, quality::Int=32)
    direction_vec = Vec{3,Float64}(direction)
    tail_f = Point{3,Float64}(tail)
    head = tail_f + normalize(direction_vec) * norm(direction_vec)
    return Arrow(tail_f, head; arrow_to_body_ratio=arrow_to_body_ratio, shaft_radius=shaft_radius, head_radius=head_radius, quality=quality)
end

# Standard Arrow with tail at origin
function Arrow(direction::Union{Point{3,<:Real},Vec{3,<:Real}}=Point{3,Float64}(0, 0, 1); arrow_to_body_ratio::Union{Real,Nothing}=nothing, shaft_radius::Union{Real,Nothing}=nothing, head_radius::Union{Real,Nothing}=nothing, quality::Int=32)
    tail = Point{3,Float64}(0, 0, 0)
    return Arrow(tail, direction; arrow_to_body_ratio=arrow_to_body_ratio, shaft_radius=shaft_radius, head_radius=head_radius, quality=quality)
end



#------



"""
Attach a vector of points (or Observables of points) to an Observable State.
Returns a vector of Observables that always represent the points in the current state.
"""
function attach2State(points::AbstractVector, state_obs::Observable{State})
    attached = Vector{Observable{Point{3, Float64}}}(undef, length(points))
    for (i, p) in enumerate(points)
        if isa(p, Observable)
            attached[i] = lift((pt, st) -> coordinate_transform(pt, st), p, state_obs)
        else
            attached[i] = lift(st -> coordinate_transform(p, st), state_obs)
        end
    end
    return attached
end


# For State (default)
function drawState!(scene::Scene, state::State=State(); location::Point{3,<:Real}=Point(0, 0, 0), scale::Real=1, alpha::Real=1.0)

    ex_arrow = Arrow(location, location + scale * state.ex)
    ey_arrow = Arrow(location, location + scale * state.ey)
    ez_arrow = Arrow(location, location + scale * state.ez)

    transparency = alpha != 1

    a = mesh!(scene, ex_arrow.mesh; color=(1, 0, 0, alpha), transparency=transparency)
    b = mesh!(scene, ey_arrow.mesh; color=(0, 1, 0, alpha), transparency=transparency)
    c = mesh!(scene, ez_arrow.mesh; color=(0, 0, 1, alpha), transparency=transparency)

    return (a, b, c)
end

# For Observable{State}
function drawState!(scene::Scene, state::Observable{State}; location::Point{3,<:Real}=Point(0, 0, 0), scale::Real=1, alpha::Real=1.0)
    # Attach arrows to the observable state
    ex_arrow = lift(s -> Arrow(location, location + scale * s.ex), state)
    ey_arrow = lift(s -> Arrow(location, location + scale * s.ey), state)
    ez_arrow = lift(s -> Arrow(location, location + scale * s.ez), state)

    transparency = alpha != 1

    a = mesh!(scene, lift(a -> a.mesh, ex_arrow); color=(1, 0, 0, alpha), transparency=transparency)
    b = mesh!(scene, lift(a -> a.mesh, ey_arrow); color=(0, 1, 0, alpha), transparency=transparency)
    c = mesh!(scene, lift(a -> a.mesh, ez_arrow); color=(0, 0, 1, alpha), transparency=transparency)

    return (a, b, c)
end

# --- Renderer Struct ---

"""
    Renderer()

A struct that provides convenient access to Arrow, attach2State, and drawState! (overloaded for State and Observable{State}).
No need to wrap the functions if you do not need to restrict or modify their signatures.
"""
struct Renderer
    Arrow::Type
    attach2State::Function
    drawState!::Function
end

Renderer() = Renderer(Arrow, attach2State, drawState!)


# --- Example Usage ---
# using GLMakie: Scene, meshscatter!, cam3d!
# using GLMakie

# include("../src/Recorder.jl")

# s = Figure(size=(500,500), camera=cam3d!, title="Recording Test")
# axes = Axis3(s[1,1],  aspect = (1, 1, 1))
# meshscatter!(axes, (0, 0, 0))

# framerate = 30  # Frames per second
# t = Observable(0.0)
# recording = Observable(true)
# io_ref = Ref{Any}(nothing)
# filepath = joinpath(@__DIR__, "test.mp4")

# # Print time on bottom left corner
# text!(axes, (0,0,0), text=lift(t) do time
#     "Time: $(round(time/framerate, digits=2)) s"
# end, color=:black, fontsize=20, space=:pixel)

# # Print t value on top right corner
# text!(axes, (500, 500, 0), text=lift(t) do time
#     "t: $(round(time, digits=2))"
# end, color=:black, fontsize=20, space=:pixel, align=(:right, :top))

# # Start recording
# record_on_change_until(s.scene, t, recording, io_ref, filepath; framerate=framerate)



# for i in 1:1000
#     t[] += 1
#     sleep(1/framerate)  # Simulate some work
#     if !recording[]
#         break
#     end
# end


