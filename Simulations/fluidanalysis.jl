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

function get_user_input(prompt::String)
    print(prompt)
    return readline()
end



println("\nDragonfly Fluid Flow Analysis - Interactive Simulation")

start = "start"
println("Type 'start' to begin the visualization. and Press Enter twice to confirm.")
readline()
println("Starting visualization...")

# Defining Custom Mesh Transformation functions
function translate_obj(obj::AbstractMesh, translation::AbstractVector)
    obj_copy = deepcopy(obj)  # Create a copy of the object to avoid modifying the original
    # Translate positions
    for i in eachindex(obj_copy.vertex_attributes[1])
        obj_copy.vertex_attributes[1][i] += translation
    end

    # Translate normals if they exist
    if length(obj_copy.vertex_attributes) >= 3
        for i in eachindex(obj_copy.vertex_attributes[3])
            obj_copy.vertex_attributes[3][i] += translation
        end
    end

    return obj_copy
end

function rotate_obj(obj::AbstractMesh, R::AbstractMatrix; about::Union{AbstractVector,Nothing}=nothing)
    obj_copy = deepcopy(obj)  # Create a copy of the object to avoid modifying the original
    # First translate the object to the origin
    if isnothing(about)
        # If no specific point is given, just rotate the object
        # Rotate positions
        for i in eachindex(obj.vertex_attributes[1])
            obj_copy.vertex_attributes[1][i] = Point3f(R * Vec3f(obj_copy.vertex_attributes[1][i]))
        end

        # Rotate normals if they exist
        if length(obj_copy.vertex_attributes) >= 3
            for i in eachindex(obj_copy.vertex_attributes[3])
                obj_copy.vertex_attributes[3][i] = normalize(Point3f(R * Vec3f(obj_copy.vertex_attributes[3][i])))
            end
        end

        return obj_copy
    else
        # If a specific point is given, translate the object to that point, rotate it, and then translate it back
        translation = -about
        obj_copy = translate_obj(obj_copy, translation)  # Translate to origin
        obj_copy = rotate_obj(obj_copy, R)  # Rotate around the origin
        obj_copy = translate_obj(obj_copy, -translation)  # Translate back to the original position

        return obj_copy
    end
end

reflect(q::Quaternion, normal::AbstractVector) = Quaternion(0, normal...) * q * Quaternion(0, normal...)


# Load the shaders and models for the DragonFly
begin #Load _models
    # shader = load("../Models/DragonFly/HighQuality/body.png") # Set to true for using shaders, false for using the default renderer

    _body_model = (load("../Models/DragonFly/Body.stl"))
    _frontleftwing_model = (load("../Models/DragonFly/FrontLeft.stl"))
    _frontrightwing_model = (load("../Models/DragonFly/FrontRight.stl"))
    _hindleftwing_model = (load("../Models/DragonFly/HindLeft.stl"))
    _hindrightwing_model = (load("../Models/DragonFly/HindRight.stl"))

    # Scale all the models to 1/10 of their original size
    scale_factor = 0.5
    _body_model = GeometryBasics.expand_faceviews(GeometryBasics.Mesh(_body_model.vertex_attributes[1] * scale_factor, _body_model.faces))
    _frontleftwing_model = GeometryBasics.expand_faceviews(GeometryBasics.Mesh(_frontleftwing_model.vertex_attributes[1] * scale_factor, _frontleftwing_model.faces))
    _frontrightwing_model = GeometryBasics.expand_faceviews(GeometryBasics.Mesh(_frontrightwing_model.vertex_attributes[1] * scale_factor, _frontrightwing_model.faces))
    # _frontrightwing_model = GeometryBasics.expand_faceviews(GeometryBasics.Mesh([Point3f(imag_part(reflect(Quaternion(0, v...), [0,1,0]))) for v in _frontleftwing_model.vertex_attributes[1]], _frontleftwing_model.faces)) # Reflect the front left wing to create the front right wing
    _hindleftwing_model = GeometryBasics.expand_faceviews(GeometryBasics.Mesh(_hindleftwing_model.vertex_attributes[1] * scale_factor, _hindleftwing_model.faces))
    _hindrightwing_model = GeometryBasics.expand_faceviews(GeometryBasics.Mesh(_hindrightwing_model.vertex_attributes[1] * scale_factor, _hindrightwing_model.faces))
    # _hindrightwing_model = GeometryBasics.expand_faceviews(GeometryBasics.Mesh([Point3f(imag_part(reflect(Quaternion(0, v...), [0,1,0]))) for v in _hindleftwing_model.vertex_attributes[1]], _hindleftwing_model.faces)) # Reflect the hind left wing to create the hind right wing
end

# Define the wing positions relative to the body position (manually determined in BLENDER)
begin # Define Positions
    body_pos = Observable(Point3(0.0, 0.0, 0.0)) # Position of the body
    frontleftwing_pos = lift(body_pos -> body_pos .+ Point3(-0.4192, -0.3, 0.342).*scale_factor, body_pos) # Position of the front left wing
    frontrightwing_pos = lift(body_pos -> body_pos .+ Point3(-0.4192, 0.3, 0.342).*scale_factor, body_pos) # Position of the front right wing
    hindleftwing_pos = lift(body_pos -> body_pos .+ Point3(0.6, -0.25, 0.34).*scale_factor, body_pos) # Position of the hind left wing
    hindrightwing_pos = lift(body_pos -> body_pos .+ Point3(0.6, 0.25, 0.34).*scale_factor, body_pos) # Position of the hind right wing
end

# Transform the models to align them correctly
begin # Rotate _models
    # _body_model = rotate_obj(_body_model, conversions.axisangle2rotmatrix([0, 0, 1], -2π / 100))
    # _frontleftwing_model = rotate_obj(_frontleftwing_model, conversions.axisangle2rotmatrix([0, 0, 1], 2π / 100))
    # _frontrightwing_model = rotate_obj(_frontrightwing_model, conversions.axisangle2rotmatrix([0, 0, 1], 2π / 100))
    # _hindleftwing_model = rotate_obj(_hindleftwing_model, conversions.axisangle2rotmatrix([0, 0, 1], 2π / 100))
    # _hindrightwing_model = rotate_obj(_hindrightwing_model, conversions.axisangle2rotmatrix([0, 0, 1], 2π / 100))
end

begin # Translate _models
    offset = Point(0.0, 0.0, 0.0) # Offset to apply to the models
    _body_model = translate_obj(_body_model, offset+Point(0.0, -0.085*scale_factor, 0.0)) # Translate the body model to the position of the body
    _frontleftwing_model = translate_obj(_frontleftwing_model, offset) # Translate the front left wing model to the position of the front left wing
    _frontrightwing_model = translate_obj(_frontrightwing_model, offset) # Translate the front right wing model to the position of the front right wing
    _hindleftwing_model = translate_obj(_hindleftwing_model, offset) # Translate the hind left wing model to the position of the hind left wing
    _hindrightwing_model = translate_obj(_hindrightwing_model, offset) # Translate the hind right wing model to the position of the hind right wing
end

function attach2State(obj::AbstractMesh, state_obs::Observable{State}, pos::Union{Observable{<:AbstractVector}, <:AbstractVector, Nothing}=nothing)
    if isnothing(pos)
        return lift(st -> rotate_obj(obj, conversions.quat2rotmatrix(st.q)), state_obs)
    elseif isa(pos, Observable)
        return lift((st, pos)-> rotate_obj(obj, conversions.quat2rotmatrix(st.q), about=pos), state_obs, pos)
    else
        return rotate_obj(obj, conversions.quat2rotmatrix(state_obs[]), about=pos)
    end
end

begin # Define State Observables
    body = Observable(State()) # Create an observable state for the body
    frontleftwing = Observable(State()) # Create an observable state for the front left wing
    frontrightwing = Observable(State()) # Create an observable state for the front right wing
    hindleftwing = Observable(State()) # Create an observable state for the hind left wing
    hindrightwing = Observable(State()) # Create an observable state for the hind right wing
end
# Reassign the mesh observables for the body and wings to transform their positions
# transform the vertices of the models using the rotation matrix defined by the element state

begin # Attach the models to the state observables with fixed _models
    body_model = attach2State(_body_model, body, body_pos)
    frontleftwing_model = attach2State(_frontleftwing_model, frontleftwing, frontleftwing_pos)
    frontrightwing_model = attach2State(_frontrightwing_model, frontrightwing, frontrightwing_pos)
    hindleftwing_model = attach2State(_hindleftwing_model, hindleftwing, hindleftwing_pos)
    hindrightwing_model = attach2State(_hindrightwing_model, hindrightwing, hindrightwing_pos)
end

println("Models and positions defined successfully.")

println("\nCreating 3D scene with dragonfly model visualization...")
println("Rendering body and wing meshes with initial positioning...")

# Create a scene for the DragonFly
begin
set_theme!(theme_minimal()) # Set the theme for the scene
s_dragonfly = Scene(camera=cam3d!, size=(500, 500))
# Plot F(centroids[i].data...) - velocities[i] at centroids of the wings
shader = colorant"#546d64"
# drawState!(s_dragonfly)
body_mesh = meshscatter!(s_dragonfly, Point(0.0, 0.0, 0.0), marker=body_model, markersize=1, color=shader) # Add the body position to the scene
frontleftwing_mesh = meshscatter!(s_dragonfly, Point(0.0, 0.0, 0.0), marker=frontleftwing_model, markersize=1, color=shader)
frontrightwing_mesh = meshscatter!(s_dragonfly, Point(0.0, 0.0, 0.0), marker=frontrightwing_model, markersize=1, color=shader)
hindleftwing_mesh = meshscatter!(s_dragonfly, Point(0.0, 0.0, 0.0), marker=hindleftwing_model, markersize=1, color=shader)
hindrightwing_mesh = meshscatter!(s_dragonfly, Point(0.0, 0.0, 0.0), marker=hindrightwing_model, markersize=1, color=shader)
end
# drawState!(s_dragonfly; scale=10)
println("Scene created successfully.")

display(s_dragonfly) # Display the scene

println("Rendered initial dragonfly model successfully.")
if get_user_input("Press Enter to continue to wing kinematics analysis, or 'quit' to exit: ") == "quit"
    println("Exiting...")
    exit()
end

# ---------------------------------------
println("\nSetting up dragonfly wing kinematics with roll and pitch motion parameters...")

# transformations.interpolate_states(frontrightwing, State(conversions.axisangle2quat([0, 1, 1], 0π / 4)))


# -----------------------------------
println("\nDefining wing motion functions: roll (flapping) and pitch (twisting) with smooth transitions...")

begin # Define the affine transformation functions

# Axis definitions
roll_axis = [1.0, 0.0, 0.0]  # X-axis for roll
pitch_axis = [0.0, 1.0, 0.0]  # Y-axis for pitch
yaw_axis = [0.0, 0.0, 1.0]    # Z-axis for yaw

# Roll parameters
A_theta_roll = 0.5      # Amplitude of roll oscillation
phi_omega_roll = 0.0    # Phase shift of roll oscillation (radians)
k_omega_roll = 2.0      # Frequency of roll oscillation (Hz)

# Pitch parameters
A_theta_pitch = 0.3     # Amplitude of pitch oscillation
phi_omega_pitch = -pi/5   # Phase shift of pitch oscillation (radians)
k_omega_pitch = k_omega_roll  # Frequency of pitch oscillation (Hz)
pitch_fraction = 0.4    # Non-constant fraction of pitch oscillation

# Yaw parameters (unused but kept for completeness)
A_theta_yaw = 0.0       # Amplitude of yaw oscillation
phi_omega_yaw = 0.0     # Phase shift of yaw oscillation
k_omega_yaw = 0.0       # Frequency of yaw oscillation

phase_between_front_and_hind = pi  # Phase difference between front and hind wings (0.5 means 180 degrees out of phase)

# Utility function to map a value from one range to another
function map_range(val, from, to)
    normalized = (val - from[1]) / (from[2] - from[1])
    return to[1] + normalized * (to[2] - to[1])
end

# Roll velocity function (angular velocity in radians/s)
function roll_velocity(t)
    return A_theta_roll * k_omega_roll * 2pi * cos(k_omega_roll * t * 2π + phi_omega_roll)
end

function roll(t) # analytical integral of the velocity function
    return A_theta_roll * sin(k_omega_roll * t * 2π + phi_omega_roll)
end

# Smoothstep function and its derivative for smooth transitions
# Position polynomial: y(x) = 6x^5 - 15x^4 + 10x^3 Velocity polynomial: y'(x) = 30x^4 - 60x^3 + 30x^2 Acceleration polynomial: y''(x) = 120x^3 - 180x^2 + 60x
# This is the third derivative Bezier curve
smoothstep(x, edge0, edge1) = edge0 + (edge1 - edge0) * (6x^5 - 15x^4 + 10x^3)
smoothstep_vel(x, edge0, edge1) = (edge1 - edge0) * (30x^4 - 60x^3 + 30x^2)

# Pitch velocity for a single period
function pitch_vel_single_period(t)
    T = 1.0
    b = T / 2 * (1 - pitch_fraction)
    a = pitch_fraction * T / 2
    if t < b / 2
        return 0.0
    elseif t < b / 2 + a
        # Map t to [0, 1] for smoothstep, scale by k_omega_pitch for frequency
        return k_omega_pitch * smoothstep_vel(map_range(t, (b/2, b/2 + a), (0, 1)), A_theta_pitch, -A_theta_pitch) / a
    elseif t < b / 2 + a + b
        return 0.0
    elseif t < b / 2 + a + b + a
        # Map t to [0, 1] for smoothstep, scale by k_omega_pitch for frequency
        return k_omega_pitch * smoothstep_vel(map_range(t, (b/2 + a + b, b/2 + a + b + a), (0, 1)), -A_theta_pitch, A_theta_pitch) / a
    else
        return 0.0
    end
end

# Pitch velocity function with phase shift
function pitch_velocity(t)
    T = 1 / k_omega_pitch
    # Apply phase shift and map to [0, 1) range
    t_phase = mod(t + phi_omega_pitch / (2π * k_omega_pitch), T) / T
    return pitch_vel_single_period(t_phase)
end

# Runge-Kutta integration for angular positions
function calc_dtheta(func, t, dt)
    k1 = func(t)
    k2 = func(t + dt/2)
    k3 = func(t + dt/2)
    k4 = func(t + dt)
    return (dt / 6) * (k1 + 2k2 + 2k3 + k4)
end

function pitch_single_period(t)
    T = 1.0
    b = T / 2 * (1 - pitch_fraction)
    a = pitch_fraction * T / 2
    if t < b / 2
        return A_theta_pitch
    elseif t < b / 2 + a
        # Map t to [0, 1] for smoothstep, scale by k_omega_pitch for frequency
        return smoothstep(map_range(t, (b/2, b/2 + a), (0, 1)), A_theta_pitch, -A_theta_pitch)
    elseif t < b / 2 + a + b
        return -A_theta_pitch
    elseif t < b / 2 + a + b + a
        # Map t to [0, 1] for smoothstep, scale by k_omega_pitch for frequency
        return smoothstep(map_range(t, (b/2 + a + b, b/2 + a + b + a), (0, 1)), -A_theta_pitch, A_theta_pitch)
    else
        return A_theta_pitch
    end
end

function pitch(t)
    T = 1 / k_omega_pitch
    # Apply phase shift and map to [0, 1) range
    t_phase = mod(t + phi_omega_pitch / (2π * k_omega_pitch), T) / T
    return pitch_single_period(t_phase)
end

end

println("Plotting wing angle trajectories over time to visualize motion patterns...")
println("Generating analytical plots of roll and pitch angles with phase relationships...")

begin # Plotting the angular velocities and positions
# Time parameters
t_min = -1.0
t_max = 1.0
dt = 0.001
t_range = t_min:dt:t_max
t_steps = length(t_range)

# Integrate velocities to get angular positions
thetas_roll = zeros(t_steps)
thetas_pitch = zeros(t_steps)
thetas_roll[1] = roll(t_min)  # Initial roll angle
thetas_pitch[1] = pitch(t_min)  # Initial pitch angle

for i in 1:(t_steps-1)
    t = t_range[i]
    thetas_roll[i+1] = thetas_roll[i] + calc_dtheta(roll_velocity, t, dt)
    thetas_pitch[i+1] = thetas_pitch[i] + calc_dtheta(pitch_velocity, t, dt)
end

# Plot velocities and angles
set_theme!(theme_minimal()) # Set the theme for the plot
set_theme!(theme_latexfonts()) # Set the theme for the plot
fig = Figure(resolution=(1200, 800), fontsize=24)

# Velocity plot
# ax1 = Axis(fig[1, 1], title="Angular Velocities", xlabel="Time (s)", ylabel="Angular Velocity (rad/s)")
# lines!(ax1, t_range, roll_velocity.(t_range), label="Roll Velocity", color=:red)
# lines!(ax1, t_range, pitch_velocity.(t_range), label="Pitch Velocity", color=:blue)
# axislegend(ax1)

# Theta plot
ax2 = Axis(fig[1, 1], title=L"\text{Angular Positions}", xlabel="Time (s)", ylabel="Angle (rad)")
lines!(ax2, t_range, thetas_roll, label="Roll Theta", color=:red, linewidth=5)
lines!(ax2, t_range, thetas_pitch, label="Pitch Theta", color=:blue, linewidth=5)

# plot advanced and delayed pitch
lines!(ax2, t_range, pitch.(t_range .+ (pi/10+pi)), label="Delayed Pitch Theta", color=:green, linestyle=:dot, linewidth=3)
lines!(ax2, t_range, pitch.(t_range .- (pi/10+pi)), label="Advanced Pitch Theta", color=:orange, linestyle=:dot, linewidth=3)
# axislegend(ax2)

leg = Legend(fig[1, 2], ax2)
# add vlines! at the phase shift points
phase_shift_points = []
for i in t_range
    if mod(i + (phi_omega_roll+pi/2) / (2π * k_omega_roll), 1 / k_omega_roll) < dt
        push!(phase_shift_points, i)
    end
end
for point in phase_shift_points
    vlines!(ax2, point, color=:black, linestyle=:dash, label="Phase Shift Point")
end

display(fig)

end

println("Wing kinematics plots generated successfully.")
if get_user_input("Press Enter to continue to real-time wing animation setup, or 'quit' to exit: ") == "quit"
    println("Exiting...")
    exit()
end

# --------------------------------
println("\nCreating time-dependent observables for real-time wing state updates...")

t = Observable(0.0)  # Time observable

begin
theta_roll_front = lift(t -> roll(t), t)  # Roll angle for front wings
theta_pitch_front = lift(t -> pitch(t), t)  # Pitch angle for front wings
theta_roll_hind = lift(t -> roll(t + phase_between_front_and_hind), t)  # Roll angle for hind wings
theta_pitch_hind = lift(t -> pitch(t + phase_between_front_and_hind), t)  # Pitch angle for hind wings

omega_roll_front = lift(t -> roll_velocity(t), t)  # Roll velocity for front wings
omega_pitch_front = lift(t -> pitch_velocity(t), t)  # Pitch velocity for front wings
omega_roll_hind = lift(t -> roll_velocity(t + phase_between_front_and_hind), t)  # Roll velocity for hind wings
omega_pitch_hind = lift(t -> pitch_velocity(t + phase_between_front_and_hind), t)  # Pitch velocity for hind wings

theta_roll_front_vec = Observable(Float64[])
theta_pitch_front_vec = Observable(Float64[])
theta_roll_hind_vec = Observable(Float64[])
theta_pitch_hind_vec = Observable(Float64[])
end

function reset_values()
    theta_roll_front_vec[] = Float64[]
    theta_pitch_front_vec[] = Float64[]
    theta_roll_hind_vec[] = Float64[]
    theta_pitch_hind_vec[] = Float64[]
    t[] = t0
end

on(t) do _
    push!(theta_roll_front_vec[], theta_roll_front[])
    push!(theta_pitch_front_vec[], theta_pitch_front[])
    push!(theta_roll_hind_vec[], theta_roll_hind[])
    push!(theta_pitch_hind_vec[], theta_pitch_hind[])

    notify(theta_roll_front_vec)
    notify(theta_pitch_front_vec)
    notify(theta_roll_hind_vec)
    notify(theta_pitch_hind_vec)
end

# ---------------------------------- 
println("\nImplementing wing state update function to apply roll and pitch rotations...")

# Define update function for dragonfly wing states

function update_dragonfly_wing_states!()
    frontleftwing[] = State(conversions.axisangle2quat(roll_axis, theta_roll_front[])) # Apply roll in global frame
    frontleftwing[] = State(conversions.coordinate_transform(conversions.axisangle2quat(pitch_axis, -theta_pitch_front[]), frontleftwing[].q) * frontleftwing[].q) # Apply pitch in local frame

    frontrightwing[] = State(conversions.axisangle2quat(roll_axis, -theta_roll_front[])) # Apply roll in global frame
    frontrightwing[] = State(conversions.coordinate_transform(conversions.axisangle2quat(pitch_axis, -theta_pitch_front[]), frontrightwing[].q) * frontrightwing[].q) # Apply pitch in local frame

    hindleftwing[] = State(conversions.axisangle2quat(roll_axis, theta_roll_hind[])) # Apply roll in global frame
    hindleftwing[] = State(conversions.coordinate_transform(conversions.axisangle2quat(pitch_axis, -theta_pitch_hind[]), hindleftwing[].q) * hindleftwing[].q) # Apply pitch in local frame

    hindrightwing[] = State(conversions.axisangle2quat(roll_axis, -theta_roll_hind[])) # Apply roll in global frame
    hindrightwing[] = State(conversions.coordinate_transform(conversions.axisangle2quat(pitch_axis, -theta_pitch_hind[]), hindrightwing[].q) * hindrightwing[].q) # Apply pitch in local frame
end

on(t) do _ # Update the wing states whenever time changes
    update_dragonfly_wing_states!()
end

# ----------------------------------
println("\nSetting up fluid flow calculations using vortex loop method for each wing surface...")

# Define fluid flow functions for the dragonfly wings

# Calculate velocity of the wing each centroid of the wing loops
function calculate_wing_velocity(point, axis, omega, about_pos)
    # Calculate the velocity of the wing at a given point based on the angular velocity
    # For rigid body rotation: v = ω × r, where r is the position vector from rotation center
    r = point - about_pos  # Vector from rotation center to the point
    return cross(omega * axis, r)  # ω is scalar, axis is direction, so ω⃗ = ω * axis
end

function calculate_wing_velocity_at_centroids(centroids, axis, omega, about_pos)
    velocities = []
    for centroid in centroids
        push!(velocities, calculate_wing_velocity(centroid, axis, omega, about_pos))  # Calculate velocity at the centroid
    end
    return velocities
end

function get_wing_loop(wing_vertices, wing_face_indices)
    wing_loops = VortexLoop[]
    for i in eachindex(wing_face_indices)
        face = wing_face_indices[i]
        vertices = [wing_vertices[face[j]] for j in 1:length(face)]
        push!(wing_loops, VortexLoop(vertices, 0.0, 0.001, 1.0)) # Create a vortex loop for each face of the wing
    end
    return wing_loops
end

function get_velocity_axis_angle()
    # Calculate the velocity axis angle based on the current time
    qfrontleftwing = conversions.axisangle2quat(roll_axis, theta_roll_front[])
    qfrontleftwing = conversions.coordinate_transform(conversions.axisangle2quat(pitch_axis, -theta_pitch_front[]), qfrontleftwing) * qfrontleftwing

    qfrontrightwing = conversions.axisangle2quat(roll_axis, -theta_roll_front[])
    qfrontrightwing = conversions.coordinate_transform(conversions.axisangle2quat(pitch_axis, -theta_pitch_front[]), qfrontrightwing) * qfrontrightwing

    qhindleftwing = conversions.axisangle2quat(roll_axis, theta_roll_hind[])
    qhindleftwing = conversions.coordinate_transform(conversions.axisangle2quat(pitch_axis, -theta_pitch_hind[]), qhindleftwing) * qhindleftwing

    qhindrightwing = conversions.axisangle2quat(roll_axis, -theta_roll_hind[])
    qhindrightwing = conversions.coordinate_transform(conversions.axisangle2quat(pitch_axis, -theta_pitch_hind[]), qhindrightwing) * qhindrightwing

    return [
        quat2axisangle(qfrontleftwing),
        quat2axisangle(qfrontrightwing),
        quat2axisangle(qhindleftwing),
        quat2axisangle(qhindrightwing)
    ]
end


# For each wing, extract vertices and faces, then compute vortex loops

begin

frontleftwing_vertices = lift(t -> frontleftwing_model[].vertex_attributes[1], t)
frontleftwing_faces = lift(t -> frontleftwing_model[].faces, t)
frontleftwing_loops = lift(t -> get_wing_loop(frontleftwing_vertices[], t), frontleftwing_faces)

frontrightwing_vertices = lift(t -> frontrightwing_model[].vertex_attributes[1], t)
frontrightwing_faces = lift(t -> frontrightwing_model[].faces, t)
frontrightwing_loops = lift(t -> get_wing_loop(frontrightwing_vertices[], t), frontrightwing_faces)

hindleftwing_vertices = lift(t -> hindleftwing_model[].vertex_attributes[1], t)
hindleftwing_faces = lift(t -> hindleftwing_model[].faces, t)
hindleftwing_loops = lift(t -> get_wing_loop(hindleftwing_vertices[], t), hindleftwing_faces)

hindrightwing_vertices = lift(t -> hindrightwing_model[].vertex_attributes[1], t)
hindrightwing_faces = lift(t -> hindrightwing_model[].faces, t)
hindrightwing_loops = lift(t -> get_wing_loop(hindrightwing_vertices[], t), hindrightwing_faces)

frontleftwing_centroids = lift(t -> [mean(loop.vertices) for loop in frontleftwing_loops[]], t)
frontrightwing_centroids = lift(t -> [mean(loop.vertices) for loop in frontrightwing_loops[]], t)
hindleftwing_centroids = lift(t -> [mean(loop.vertices) for loop in hindleftwing_loops[]], t)
hindrightwing_centroids = lift(t -> [mean(loop.vertices) for loop in hindrightwing_loops[]], t)

velocity_axis_angles = lift(t -> get_velocity_axis_angle(), t)


frontleftwing_velocities = lift(t -> calculate_wing_velocity_at_centroids(frontleftwing_centroids[], velocity_axis_angles[][1][1], velocity_axis_angles[][1][2], frontleftwing_pos[]), t)
frontrightwing_velocities = lift(t -> calculate_wing_velocity_at_centroids(frontrightwing_centroids[], velocity_axis_angles[][2][1], velocity_axis_angles[][2][2], frontrightwing_pos[]), t)
hindleftwing_velocities = lift(t -> calculate_wing_velocity_at_centroids(hindleftwing_centroids[], velocity_axis_angles[][3][1], velocity_axis_angles[][3][2], hindleftwing_pos[]), t)
hindrightwing_velocities = lift(t -> calculate_wing_velocity_at_centroids(hindrightwing_centroids[], velocity_axis_angles[][4][1], velocity_axis_angles[][4][2], hindrightwing_pos[]), t)


nothing
end

# begin
# #Visualize wing_loops
# function get_arrows(wing_loops)
#     arrows = []
#     for loop in wing_loops
#         for vortexline in loop.vortexlines
#             push!(arrows, Arrow(vortexline.R1, vortexline.R2))
#         end
#     end
#     return arrows
# end

# # Create arrows for each wing loop
# frontleftwing_arrows = lift(t -> get_arrows(frontleftwing_loops[]), t)
# frontrightwing_arrows = lift(t -> get_arrows(frontrightwing_loops[]), t)
# hindleftwing_arrows = lift(t -> get_arrows(hindleftwing_loops[]), t)
# hindrightwing_arrows = lift(t -> get_arrows(hindrightwing_loops[]), t)


# frontleftwing_arrows_mesh = []
# frontrightwing_arrows_mesh = []
# hindleftwing_arrows_mesh = []
# hindrightwing_arrows_mesh = []

# for i in eachindex(frontleftwing_arrows[])
#     color = rand(GLMakie.RGB)  # Random color for each arrow
#     push!(frontleftwing_arrows_mesh, meshscatter!(s_dragonfly, Point3(0.0, 0.0, 0.0), marker=lift(t->frontleftwing_arrows[][i].mesh, t), markersize=1, color=color))
# end
# for i in eachindex(frontrightwing_arrows[])
#     color = rand(GLMakie.RGB)  # Random color for each arrow
#     push!(frontrightwing_arrows_mesh, meshscatter!(s_dragonfly, Point3(0.0, 0.0, 0.0), marker=lift(t->frontrightwing_arrows[][i].mesh, t), markersize=1, color=color))
# end
# for i in eachindex(hindleftwing_arrows[])
#     color = rand(GLMakie.RGB)  # Random color for each arrow
#     push!(hindleftwing_arrows_mesh, meshscatter!(s_dragonfly, Point3(0.0, 0.0, 0.0), marker=lift(t->hindleftwing_arrows[][i].mesh, t), markersize=1, color=color))
# end
# for i in eachindex(hindrightwing_arrows[])
#     color = rand(GLMakie.RGB)  # Random color for each arrow
#     push!(hindrightwing_arrows_mesh, meshscatter!(s_dragonfly, Point3(0.0, 0.0, 0.0), marker=lift(t->hindrightwing_arrows[][i].mesh, t), markersize=1, color=color))
# end

# end

# begin
# # delete the arrows from the scene
# for arrow in frontleftwing_arrows_mesh
#     delete!(s_dragonfly, arrow)
# end
# for arrow in frontrightwing_arrows_mesh
#     delete!(s_dragonfly, arrow)
# end
# for arrow in hindleftwing_arrows_mesh
#     delete!(s_dragonfly, arrow)
# end
# for arrow in hindrightwing_arrows_mesh
#     delete!(s_dragonfly, arrow)
# end

# end

# begin
#     # Visualize the velocities at the centroids of the wings
#     wing_velocity_centroid_arrows = []
#     for i in eachindex(frontleftwing_centroids[])
#         color = :orange  # Fixed color for each velocity vector
#         push!(wing_velocity_centroid_arrows, meshscatter!(s_dragonfly, Point(0.0, 0.0, 0.0), markersize=1, color=color, marker=lift(t -> Arrow(frontleftwing_centroids[][i], Vec{3, Float64}(frontleftwing_velocities[][i])).mesh, t)))
#     end
#     for i in eachindex(frontrightwing_centroids[])
#         color = :orange  # Fixed color for each velocity vector
#         push!(wing_velocity_centroid_arrows, meshscatter!(s_dragonfly, Point(0.0, 0.0, 0.0), markersize=1, color=color, marker=lift(t -> Arrow(frontrightwing_centroids[][i], Vec{3, Float64}(frontrightwing_velocities[][i])).mesh, t)))
#     end
#     for i in eachindex(hindleftwing_centroids[])
#         color = :orange  # Fixed color for each velocity vector
#         push!(wing_velocity_centroid_arrows, meshscatter!(s_dragonfly, Point(0.0, 0.0, 0.0), markersize=1, color=color, marker=lift(t -> Arrow(hindleftwing_centroids[][i], Vec{3, Float64}(hindleftwing_velocities[][i])).mesh, t)))
#     end
#     for i in eachindex(hindrightwing_centroids[])
#         color = :orange  # Fixed color for each velocity vector
#         push!(wing_velocity_centroid_arrows, meshscatter!(s_dragonfly, Point(0.0, 0.0, 0.0), markersize=1, color=color, marker=lift(t -> Arrow(hindrightwing_centroids[][i], Vec{3, Float64}(hindrightwing_velocities[][i])).mesh, t)))
#     end
# end
# begin
#     for arrow in wing_velocity_centroid_arrows
#         delete!(s_dragonfly, arrow)  # Delete the arrows from the scene
#     end 
# end

# Visualize Velocity Field of Air 
function calculate_Gamma(wing_loops, centroids, v_w)
    # Calculate the circulation strengths for the vortex loops based on the desired velocities at their centroids
    num_loops = length(wing_loops) # Number of loops
    @assert num_loops > 0 "There must be at least one vortex loop to calculate circulation strengths."
    phi = zeros(Point3{Float64}, num_loops) # Desired additive velocity vector due to known factors at the centroid of each loop
    function calculate_normal_vector(loop::VortexLoop, centroid)
        # Calculate the normal vector for the loop based on its vertices
        @assert length(loop.vertices) >= 3 "Loop must have at least 3 vertices to define a plane."
        v1 = centroid - loop.vertices[1]  # Vector from first vertex to centroid
        v2 = loop.vertices[2] - loop.vertices[1]  # Vector from first to second vertex

        n = normalize(cross(v1, v2))  # Normal vector using cross product
        return n
    end
    n_hat = [calculate_normal_vector(loop, centroids[i]) for (i, loop) in enumerate(wing_loops)] # Normal vector for each loop

    A = zeros(num_loops,num_loops)
    for i in 1:num_loops
        for j in 1:num_loops
            A[i,j] = dot(prefactor(centroids[i], wing_loops[j]), n_hat[i]) # Calculate the prefactor at the centroid of each loop
        end
    end

    b = [dot(v_w[i]-phi[i], n_hat[i]) for i in 1:num_loops] # Calculate the right-hand side vector

    Gamma = inv(factorize(A)) * b # Solve for the circulation strengths

    return Gamma
end

wing_loops = lift(t -> vcat(frontleftwing_loops[], frontrightwing_loops[], hindleftwing_loops[], hindrightwing_loops[]), t)  # Concatenate the vortex loops of all wings
centroids = lift(t -> vcat(frontleftwing_centroids[], frontrightwing_centroids[], hindleftwing_centroids[], hindrightwing_centroids[]), t)
v_w = lift(t -> vcat(frontleftwing_velocities[], frontrightwing_velocities[], hindleftwing_velocities[], hindrightwing_velocities[]), t)  # Concatenate the velocities of all wings

# ----------------------------------

t0 = 0.0

reset_values()  # Reset the values at the start

t[] = t0

dt = 0.01
n=1+rand()
n=0.11
n=1

println("\nStarting real-time wing animation with vortex circulation calculations...")
println("Animation duration: $(n) seconds at $(1/dt) FPS")
println("Watch for wing color changes indicating circulation strength:")
println("- Red surfaces: Positive circulation (upward flow)")
println("- Blue surfaces: Negative circulation (downward flow)")
println("- Pink arrows: Velocity difference vectors")

display(s_dragonfly)  # Display the dragonfly scene

gamma_meshes= []
vs = []

sigmoid(x) = 1 / (1 + exp(-x))  # Sigmoid function for color mapping

# # Start recording BEFORE adding particles
# framerate = 30  # Frames per second
# recording = Observable(true)
# io_ref = Ref{Any}(nothing)
# filepath = "fluidflowwingmotion_4_plots.mp4"
# change = Observable(true)

# record_on_change_until(fig.scene, change, recording, io_ref, filepath; framerate=framerate)

# positions = [Point3(0.0)]
# velocities = [Point3(0.0)]
# accelerations = [Point3(0.0)]

function triangle_area_3d(p1, p2, p3)
    # Calculate the area of a triangle in 3D space using the cross product
    v1 = p2 .- p1
    v2 = p3 .- p1
    return norm(cross(v1, v2)) / 2.0  # Area = 0.5 * ||v1 x v2||
end
momenta = []

for i in range(dt, n, step=dt)
    for m in gamma_meshes
        delete!(s_dragonfly, m)  # Delete the previous gamma meshes from the scene
    end
    for v in vs
        delete!(s_dragonfly, v)  # Delete the previous velocity vectors from the scene
    end

    t[] += dt  # Increment time by dt seconds

    # xlims!(ax, t0, t[]) # Set x-limits based on the time range
    # ylims!(((-1.1, 1.1) .* max(A_theta_pitch, A_theta_roll))...)  # Set y-limits based on max amplitude
    yield()
    loops = wing_loops[]  # Get the current vortex loops
    Gamma = calculate_Gamma(loops, centroids[], v_w[])  # Calculate the circulation strengths for the vortex loops
    set_Gamma!(loops, Gamma)  # Set the circulation strengths for the vortex loops
    eval(:(global F(x,y,z) = velocity(Point3(x, y, z), $(loops))))  # Define the velocity field function for the vortex loops

    
    for i in eachindex(Gamma)
        reds = GLMakie.cgrad(:reds)
        blues = GLMakie.cgrad(:blues)
        color = Gamma[i] > 0 ? reds[map_range(Gamma[i], (minimum(Gamma), maximum(Gamma)), (0, 1))] : blues[map_range(Gamma[i], (minimum(Gamma), maximum(Gamma)), (1, 0))]  # Map circulation strength to color using map_range function
        push!(gamma_meshes,mesh!(s_dragonfly, wing_loops[][i].vertices, color=color))  # Visualize the vortex loops with color based on circulation strength
    end
    for i in eachindex(centroids[])
        v = F(centroids[][i].data...) - v_w[][i]
        push!(vs, meshscatter!(s_dragonfly, Point(0.0, 0.0, 0.0), marker=Arrow(centroids[][i], Vec3(v)).mesh, markersize=1, color=:pink)) # Visualize the difference between the velocity field and the wing velocity at each centroid
    end

    # Update the momenta vector with the current vortex momenta rho * Gamma * V_air * loop_area * normal_vector
    push!(momenta, sum([1 * loop.Gamma * norm(F(0,0,0)) * triangle_area_3d(loop.vertices...) * calculate_normal_vector(loop) for loop in loops]))  # Sum the momenta of all loops


    # Update the position, velocity and acceleration vectors
    # Calculate acceleration to be F(0,0,0)/m, take m=1
    # Calculate the velocity as the integral of the acceleration'
    # Calculate the position as the integral of the velocity
    # acceleration_val = F(0.0, 0.0, 0.0)  # Calculate the acceleration at the origin
    # velocity_val = velocities[end] + acceleration_val * dt  # Update the velocity based on the acceleration
    # position_val = positions[end] + velocity_val * dt  # Update the position based on the velocity
    # push!(accelerations, acceleration_val)  # Append the acceleration to the list
    # push!(velocities, velocity_val)  # Append the velocity to the list
    # push!(positions, position_val)  # Append the position to the list

    yield()
    # notify(change)
    sleep(dt)  # Sleep for dt seconds to simulate real-time updates
end


d_momenta_dt = [(momenta[i+1]-momenta[i])/dt for i in eachindex(momenta[1:end-1])]  # Calculate the derivative of the momenta vector with respect to time
# d_momenta_dt = Forces for each dt step
mass = 1.0  # Mass of the dragonfly, assuming unit mass for simplicity
acceleration = [d_momenta_dt[i] / mass for i in eachindex(d_momenta_dt)]  # Calculate the acceleration from the momenta derivative

lines(norm.(d_momenta_dt))
hlines!([0.0])
# Initialize velocity and position arrays
vel_over_time = Vector{Point3{Float64}}(undef, length(acceleration))
position_over_time = Vector{Point3{Float64}}(undef, length(acceleration))

# Set initial conditions
vel_over_time[1] = Point3(0.0, 0.0, 0.0)  # Initial velocity
position_over_time[1] = Point3(0.0, 0.0, 0.0)  # Initial position

# Integrate using proper numerical integration
for i in 2:length(acceleration)
    # Velocity integration: v(t+dt) = v(t) + a(t) * dt
    vel_over_time[i] = vel_over_time[i-1] + acceleration[i-1] * dt

    # Position integration using trapezoidal rule for better accuracy
    # x(t+dt) = x(t) + (v(t) + v(t+dt)) * dt / 2
    position_over_time[i] = position_over_time[i-1] + (vel_over_time[i-1] + vel_over_time[i]) * dt / 2
end

# lines(position_over_time)
lines(position_over_time.*0.1, color=[GLMakie.RGB(1.0 - i/length(position_over_time), 0.0, i/length(position_over_time)) for i in eachindex(position_over_time)], linewidth=5)  # Visualize the position over time with color gradient
position_over_time
s_dragonfly
lines!(s_dragonfly, position_over_time .* 10, linewidth=5, color=[GLMakie.RGB(1.0 - i/length(position_over_time), 0.0, i/length(position_over_time)) for i in eachindex(position_over_time)])

println("Flight trajectory added to 3D scene with color gradient (red=start, blue=end)")
if get_user_input("Press Enter to visualize fluid velocity fields, or 'quit' to exit: ") == "quit"
    println("Exiting...")
    exit()
end
delete!(s_dragonfly, s_dragonfly[end])  # Delete the previous gamma meshes from the scene
for v in vs
    delete!(s_dragonfly, v)  # Delete the previous velocity vectors from the scene
end
for i in eachindex(centroids[])
    v = F(centroids[][i].data...) - v_w[][i]
    push!(vs, meshscatter!(s_dragonfly, Point(0.0, 0.0, 0.0), marker=Arrow(centroids[][i], Vec3(v)).mesh, markersize=1, color=:pink)) # Visualize the difference between the velocity field and the wing velocity at each centroid
end
sleep(2.0)
# ------------------
println("\nComputing and visualizing 3D fluid velocity field around dragonfly...")
println("Displaying vector field generated by wing vortices using Biot-Savart law...")

vecs = visualize_vector_field!(s_dragonfly, F, bounds=[1.5, 1.5, 0.5], resolution=[15, 15, 20], center=rand(centroids[]))

println("3D velocity field visualization added to scene.")
if get_user_input("Press Enter to analyze relative velocity field, or 'quit' to exit: ") == "quit"
    println("Exiting...")
    exit()
end

for v in vecs
    delete!(s_dragonfly, v)  # Delete the previous vector field from the scene
end

println("\nAnalyzing relative velocity field in wing reference frame...")
println("Computing flow velocities relative to moving wing surface...")

F_rel(x,y,z) = F(x,y,z) .- calculate_wing_velocity(Point3(x, y, z), velocity_axis_angles[][1][1], velocity_axis_angles[][1][2], frontleftwing_pos[])  # Relative velocity field function
frontleftwing_centroid = mean(frontleftwing_centroids[])

m = visualize_vector_field!(s_dragonfly, F_rel, bounds=[1.5, 1.5, 1.3], resolution=[13, 11, 11], center=frontleftwing_centroid, custom_arrows=true, scale=0.5*0.75, base=3)  # Visualize the relative velocity field at the front left wing centroid

println("Relative velocity field visualization completed around front left wing.")
if get_user_input("Press Enter to view final time-series analysis, or 'quit' to exit: ") == "quit"
    println("Exiting...")
    exit()
end

for v in m
    delete!(s_dragonfly, v)  # Delete the previous relative vector field from the scene
end

# @async for i in 1:30*30
# notify(change)
# sleep(1/30)
# end

# recording[] = false

#-------------------

println("\nGenerating final time-series plots of wing angular positions...")
println("Displaying complete motion history throughout simulation duration...")

if get_user_input("Press Enter to continue to time-series analysis, or 'quit' to exit: ") == "quit"
    println("Exiting...")
    exit()
end

fig = Figure()
ax = Axis(fig[1, 1], title="Angular Positions Over Time", xlabel="Time (s)", ylabel="Angle (rad)")
lines!(ax, lift(theta_roll_front_vec -> range(t0, step=dt, length=length(theta_roll_front_vec)), theta_roll_front_vec), theta_roll_front_vec, label="Front Roll Theta", color=:red)
lines!(ax, lift(theta_pitch_front_vec -> range(t0, step=dt, length=length(theta_pitch_front_vec)), theta_pitch_front_vec), theta_pitch_front_vec, label="Front Pitch Theta", color=:blue)
lines!(ax, lift(theta_roll_hind_vec -> range(t0, step=dt, length=length(theta_roll_hind_vec)), theta_roll_hind_vec), theta_roll_hind_vec, label="Hind Roll Theta", color=:green)
lines!(ax, lift(theta_pitch_hind_vec -> range(t0, step=dt, length=length(theta_pitch_hind_vec)), theta_pitch_hind_vec), theta_pitch_hind_vec, label="Hind Pitch Theta", color=:orange)

# Add axis legend in fig[1,2]
Legend(fig[1, 2], ax, position=:bottomright, title="Angular Positions", fontsize=10)

display(fig)  # Display the figure with the initial state

println("Time-series analysis completed successfully!")
println("\nDragonfly fluid flow simulation finished!")
println("All visualizations generated:")
println("✓ 3D dragonfly model with animated wings")
println("✓ Wing kinematics plots (roll/pitch angles)")
println("✓ Real-time vortex circulation visualization")
println("✓ Flight trajectory from aerodynamic forces") 
println("✓ 3D fluid velocity field")
println("✓ Relative velocity field analysis")
println("✓ Complete time-series motion data")

println("\nPress Enter to exit simulation...")
readline()
println("Exiting dragonfly fluid flow analysis.")