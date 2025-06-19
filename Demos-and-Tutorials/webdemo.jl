# Web Demo for Quaternion Vector Rotation and State Rendering
# Open REPL and type "]" to enter package manager followed by "add WGLMakie, GeometryBasics, LinearAlgebra, Quaternions, DataStructures, FileIO"
# Go to all the source files and edit GLMakie to WGLMakie


using WGLMakie, GeometryBasics, LinearAlgebra, Quaternions
import Quaternions: Quaternion as Quaternion

# Import your custom code and utility functions
include("..src/State_and_Conversions.jl")
include("..src/Rendering.jl")
include("..src/Transformations.jl")

conversions = Conversions()
renderer = Renderer()
transformations = Transformations()

# --- 1. Basic Quaternion Vector Rotation Demo ---
scene = Scene(camera=cam3d!)

renderer.drawState!(scene)  # Draw reference axes

v = [0.0, 0.0, 1.0]
scale_arrow = 1.0
mesh!(scene, Arrow(Point3f(v .* scale_arrow)).mesh, color=:black, label="v")

axis = [1.0, 0.0, 0.0]
angle = π / 2
rotation_quaternion = conversions.axisangle2quat(axis, angle)
v_rotated = imag_part(rotation_quaternion * Quaternion(0, v...) * inv(rotation_quaternion))
mesh!(scene, Arrow(Point3f(v_rotated .* scale_arrow)).mesh, color=:red)

# --- 2. State and Attached Vectors Demo ---
scene = Scene(camera=cam3d!)
renderer.drawState!(scene)

wingstate = Observable(State())
custom_vectors = [Observable(Point3f(1,0,0)), Observable(Point3f(0,1,0)), Observable(Point3f(0,0,1)), Observable(Point3f(1,1,1))]
custom_colors = [(1,0,0), (0,1,0), (0,0,1), (0,0,0)]
attached = renderer.attach2State(custom_vectors, wingstate)
arrow_scale = 0.75
for i in eachindex(attached)
    mesh!(scene, lift(p -> Arrow(Point3f(p .* arrow_scale)).mesh, attached[i]), color=(custom_colors[i]..., 0.5), transparency=true)
end

# Tracker 
begin
    using DataStructures: CircularBuffer
    n = 400 # Buffer size

    buffer = CircularBuffer{Point3f}(n)
    buffer_obs = Observable(buffer)

    trail = lines!(scene, lift(x -> collect(x), buffer_obs), color=:black, linewidth=2, fxaa=true)

    trail_tracker = on(attached[4]) do new_point
        push!(buffer_obs[], new_point .* 0.75)
        notify(buffer_obs)
    end
end

# Update the state with a quaternion
rotatation_about_local_x = (conversions.axisangle2quat([1,0,0], π/4), true)
rotation_about_local_y = (conversions.axisangle2quat([0,1,0], π/4), true)
rotation_about_local_z = (conversions.axisangle2quat([0,0,1], π/4), true)
rotation_about_local_111 = (conversions.axisangle2quat([1,1,1], π/4), true)

transformations.combine_rotation_interpolations(wingstate, rotatation_about_local_x, rotation_about_local_y, rotation_about_local_z, rotation_about_local_111; n=100, time=2.0, rate_function=t -> sin(t * π / 2))
transformations.interpolate_states(wingstate, State())

delete!(scene, trail)
off(trail_tracker)  # Stop tracking the trail after the rotation is done

# --- 3. Wing Rendering Demo --- 
using FileIO
using LinearAlgebra

scene = Scene(camera=cam3d!)
renderer.drawState!(scene)

wingstate = Observable(State())
custom_vectors = [Observable(Point3f(1,0,0)), Observable(Point3f(0,1,0)), Observable(Point3f(0,0,1)), Observable(Point3f(1,1,1))]
custom_colors = [(1,0,0), (0,1,0), (0,0,1), (0,0,0)]
attached = renderer.attach2State(custom_vectors, wingstate)
arrow_scale = 0.75
for i in eachindex(attached)
    mesh!(scene, lift(p -> Arrow(Point3f(p .* arrow_scale)).mesh, attached[i]), color=(custom_colors[i]..., 0.5), transparency=true)
end

# Load the wing model
# YOU MIGHT NEED TO EDIT THIS PATH TO POINT TO YOUR ASSET DIRECTORY
wing = load("../Models/edited_wing.stl")  # Use the correct path relative to your project directory

# Create an observable mesh that rotates with vector_state
wing_scale = 0.0075
obswingmesh = lift(wingstate) do st
    R = conversions.quat2rotmatrix(st.q)
    positions = wing.vertex_attributes[1]
    rotated_positions = [wing_scale * Point3f(R * Vec3f(p)) for p in positions]
    faces = wing.faces
    GeometryBasics.mesh(rotated_positions, faces)
end

mesh!(scene, obswingmesh, color=:gold, transparency=true, alpha=0.5)

s1 = State()
s1 = State(conversions.axisangle2quat([1, 0, 0], π / 3) * s1.q) # Rotate around x-axis by 60 degrees

s2 = State()
s2 = State(conversions.axisangle2quat([1, 0, 0], -π / 3) * s2.q) # Rotate around x-axis by -60 degrees
s2 = State(conversions.coordinate_transform(conversions.axisangle2quat([0, 1, 0], π / 6), s2.q) * s2.q) # Rotate around LOCAL y-axis by 30 degrees

wingstate[] = s1 # Set the initial state of the wing mesh
wingstate[] = s2 # Set the second state of the wing mesh
wingstate[] = s1 # Reset to the first state

iterations = 10
n_per_iteration = 50 # Number of frames per iteration
time_per_iteration = 0.1 # Time for each iteration in seconds
for t in 1:iterations
    # Interpolate the wing mesh state between s1 and s2
    toState = iseven(t) ? s1 : s2 # Alternate between s1 and s2
    transformations.interpolate_states(wingstate, toState; n=n_per_iteration, time=time_per_iteration, rate_function=t -> sin(t * pi / 2))
end