# Import necessary packages
using GLMakie, GeometryBasics, LinearAlgebra, Quaternions, FileIO
import Quaternions: Quaternion as Quaternion

# Import custom code and utility functions
include("src/State_and_Conversions.jl")
include("src/Rendering.jl")
include("src/Transformations.jl")
include("src/WindowManager.jl")

# Initialize the custom structs and utilities
conversions = Conversions()
renderer = Renderer()
transformations = Transformations()
windowmanager = WindowManager()

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

# Load the shaders and models for the DragonFly
begin #Load _models
    shader = load("Models/DragonFly/HighQuality/body.png") # Set to true for using shaders, false for using the default renderer

    _body_model = GeometryBasics.expand_faceviews(load("Models/DragonFly/HighQuality/body.obj").mesh)
    _frontleftwing_model = GeometryBasics.expand_faceviews(load("Models/DragonFly/HighQuality/frontleftwing.obj").mesh)
    _frontrightwing_model = GeometryBasics.expand_faceviews(load("Models/DragonFly/HighQuality/frontrightwing.obj").mesh)
    _hindleftwing_model = GeometryBasics.expand_faceviews(load("Models/DragonFly/HighQuality/hindleftwing.obj").mesh)
    _hindrightwing_model = GeometryBasics.expand_faceviews(load("Models/DragonFly/HighQuality/hindrightwing.obj").mesh)
end

# Define the wing positions relative to the body position (manually determined in BLENDER)
begin # Define Positions
    body_pos = Observable(Point3(0.0, 0.0, 0.0)) # Position of the body
    frontleftwing_pos = lift(body_pos -> body_pos .+ Point3(-0.4192, -0.1287, 1.342), body_pos) # Position of the front left wing
    frontrightwing_pos = lift(body_pos -> body_pos .+ Point3(-0.4192, 0.1287, 1.342), body_pos) # Position of the front right wing
    hindleftwing_pos = lift(body_pos -> body_pos .+ Point3(-0.2366, -0.1728, 1.184), body_pos) # Position of the hind left wing
    hindrightwing_pos = lift(body_pos -> body_pos .+ Point3(-0.2366, 0.1728, 1.184), body_pos) # Position of the hind right wing
end

# Transform the models to align them correctly
begin # Rotate _models
    _body_model = rotate_obj(_body_model, conversions.axisangle2rotmatrix([1, 0, 0], π / 2))
    _frontleftwing_model = rotate_obj(_frontleftwing_model, conversions.axisangle2rotmatrix([1, 0, 0], π / 2))
    _frontrightwing_model = rotate_obj(_frontrightwing_model, conversions.axisangle2rotmatrix([1, 0, 0], π / 2))
    _hindleftwing_model = rotate_obj(_hindleftwing_model, conversions.axisangle2rotmatrix([1, 0, 0], π / 2))
    _hindrightwing_model = rotate_obj(_hindrightwing_model, conversions.axisangle2rotmatrix([1, 0, 0], π / 2))
end

begin # Translate _models
    _body_model = translate_obj(_body_model, -body_pos[]) # Translate the body model to the position of the body
    _frontleftwing_model = translate_obj(_frontleftwing_model, -frontleftwing_pos[]) # Translate the front left wing model to the position of the front left wing
    _frontrightwing_model = translate_obj(_frontrightwing_model, -frontrightwing_pos[]) # Translate the front right wing model to the position of the front right wing
    _hindleftwing_model = translate_obj(_hindleftwing_model, -hindleftwing_pos[]) # Translate the hind left wing model to the position of the hind left wing
    _hindrightwing_model = translate_obj(_hindrightwing_model, -hindrightwing_pos[]) # Translate the hind right wing model to the position of the hind right wing
end

function attach2State(obj::AbstractMesh, state_obs::Observable{State})
    return lift(st -> rotate_obj(obj, conversions.quat2rotmatrix(st.q)), state_obs)
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
body_model = attach2State(_body_model, body)
frontleftwing_model = attach2State(_frontleftwing_model, frontleftwing)
frontrightwing_model = attach2State(_frontrightwing_model, frontrightwing)
hindleftwing_model = attach2State(_hindleftwing_model, hindleftwing)
hindrightwing_model = attach2State(_hindrightwing_model, hindrightwing)
end


# Create a scene for the DragonFly
set_theme!(theme_dark()) # Set the theme for the scene
s_dragonfly = Scene(camera=cam3d!, size=(1920, 1080))
# drawState!(s_dragonfly)
body_mesh = meshscatter!(s_dragonfly, body_pos, marker=body_model, markersize=1, color=shader) # Add the body position to the scene
frontleftwing_mesh = meshscatter!(s_dragonfly, frontleftwing_pos, marker=frontleftwing_model, markersize=1, color=shader)
frontrightwing_mesh = meshscatter!(s_dragonfly, frontrightwing_pos, marker=frontrightwing_model, markersize=1, color=shader)
hindleftwing_mesh = meshscatter!(s_dragonfly, hindleftwing_pos, marker=hindleftwing_model, markersize=1, color=shader)
hindrightwing_mesh = meshscatter!(s_dragonfly, hindrightwing_pos, marker=hindrightwing_model, markersize=1, color=shader)

# Optional Repositioning of the DragonFly
body_pos[] = Point3(0.25, 0.0, -1.0)





# Simulate Flapping Wings

# Animate the wing mesh by rotating it around the ±60 degrees around the global x-axis as well as 0-30 degreesglobal y-axis

# It turns out that the sandwich between q and inv(q) rotates quaternions.
# Similarly, the sandwich between q and q reflects quaternions.
#https://www.euclideanspace.com/maths/geometry/affine/reflection/quaternion/index.htm#:~:text=We%20can%20represent%20reflection%20using,which%20pass%20through%20the%20origin.

plane_normal = normalize([2,3,4]) # Normal vector of the plane for reflection
reflection_quaternion = Quaternion(0, plane_normal...) # Quaternion representing the reflection across the plane

pt = Point3(1.0, 2.0, 3.0) # Point through which the reflection is performed
reflected_pt = imag_part(reflection_quaternion * Quaternion(0, pt...) * reflection_quaternion) # Reflect the point across the plane

reflect(q::Quaternion, normal::AbstractVector) = Quaternion(0, normal...) * q * Quaternion(0, normal...)

sfrontright1 = State()
sfrontright1 = State(conversions.axisangle2quat([1, 0, 0], π / 10) * sfrontright1.q) # Rotate around x-axis by 60 degrees
sfrontleft1 = State(reflect(sfrontright1.q, [0, 1, 0])) # Reflect the quaternion across the XZ plane (i.e. the normal vector is [0, 1, 0])

sfrontright2 = State()
sfrontright2 = State(conversions.axisangle2quat([1, 0, 0], -π / 6) * sfrontright2.q) # Rotate around x-axis by -60 degrees
sfrontright2 = State(conversions.coordinate_transform(conversions.axisangle2quat([0, 1, 0], π / 6), sfrontright2.q) * sfrontright2.q) # Rotate around y-axis by 30 degrees
sfrontleft2 = State(reflect(sfrontright2.q, [0, 1, 0])) # Reflect the quaternion across the XZ plane (i.e. the normal vector is [0, 1, 0])

frontleftwing[] = State(sfrontleft1.q)
frontrightwing[] = State(sfrontright1.q)

shindright1 = State()
shindright1 = State(conversions.axisangle2quat([1, 0, 0], -π / 6) * shindright1.q) # Rotate around x-axis by 60 degrees
shindright1 = State(conversions.coordinate_transform(conversions.axisangle2quat([0, 1, 0], π / 6), shindright1.q) * shindright1.q) # Rotate around y-axis by -30 degrees
shindleft1 = State(reflect(shindright1.q, [0, 1, 0])) # Reflect the quaternion across the XZ plane (i.e. the normal vector is [0, 1, 0])

shindright2 = State()
shindright2 = State(conversions.axisangle2quat([1, 0, 0], π / 10) * shindright2.q) # Rotate around x-axis by -60 degrees
shindleft2 = State(reflect(shindright2.q, [0, 1, 0])) # Reflect the quaternion across the XZ plane (i.e. the normal vector is [0, 1, 0])

hindleftwing[] = State(shindleft1.q)
hindrightwing[] = State(shindright1.q)

# Make sure the ordering of states is consistent with the wing observables:
#
# wingstates order: [frontleftwing, frontrightwing, hindleftwing, hindrightwing]
# states order: [(sfrontleft1, sfrontleft2), (sfrontright1, sfrontright2),
#                (shindleft1, shindleft2), (shindright1, shindright2)]
# current_states in the same order:
current_states = [sfrontleft1, sfrontright1, shindleft1, shindright1]
wingstates = [frontleftwing, frontrightwing, hindleftwing, hindrightwing]
states = [(sfrontleft1, sfrontleft2), (sfrontright1, sfrontright2),
          (shindleft1, shindleft2), (shindright1, shindright2)]

iterations = 10
n_per_iteration = 50           # Number of frames per iteration
time_per_iteration = 0.1         # Time for each iteration in seconds

for flap in 1:iterations
    # Alternate between state1 and state2 for each wing.
    # Use state1 on odd flaps and state2 on even flaps.
    toState = iseven(flap) ? 2 : 1
    next_states = [s[toState] for s in states]

    # Store the starting states for this flap
    start_states = deepcopy(current_states)

    rate_function = t -> (1 - cos(t * π)) / 2  # Smooth interpolation using a cosine function

    for i in 1:n_per_iteration
        t_normalized = rate_function(i / n_per_iteration)  # Normalize t to [0, 1]
        for j in 1:length(current_states)
            # Interpolate between the start and target state for this wing
            wingstates[j][] = State(slerp(start_states[j].q, next_states[j].q, t_normalized))
        end
        yield()  # Yield to allow the GUI to update
        sleep(time_per_iteration / n_per_iteration)  # Control the speed of the animation
    end

    # Update current_states to the new states after this flap
    current_states = deepcopy(next_states)
end


recording = Observable(true)
global io_ref = Ref{Any}(nothing)  # To store the io handle globally

on(events(s_dragonfly).keyboardbutton) do event
    if event.action == Keyboard.press && event.key == Keyboard.q
        recording[] = false
    end
end

@async begin
    sleep(1)  # Wait for the scene to be fully rendered before starting the recording
    record(s_dragonfly, "DragonFly.mp4"; framerate=n_per_iteration/time_per_iteration) do io
        io_ref[] = io  # Store the io handle for use in the listener

        # Listener: record a frame only when wingstate changes and recording is on
        wingstate_listener = on(hindrightwing) do new_state
            if recording[] && io_ref[] !== nothing
                recordframe!(io_ref[])
            end
        end

        # Wait until recording is stopped or window is closed
        while isopen(s) && recording[]
            yield()  # Yield to event loop, no need to sleep
        end

        # Clean up
        off(wingstate_listener)
        io_ref[] = nothing
    end
end

n_per_iteration/time_per_iteration


# Method 2: Rotation velocity
wingstates = [frontleftwing, frontrightwing, hindleftwing, hindrightwing]
for i in wingstates
    i[] = State()  # Reset the wing states to the initial state
end

reflect(q::Quaternion, normal::AbstractVector) = Quaternion(0, normal...) * q * Quaternion(0, normal...)


# Parameters for global rotation around the x-axis
phi_x_global = deg2rad(0)
omega_x_global = 40
theta_min = deg2rad(0)  # Minimum angle for the global x-axis rotation
theta_max = deg2rad(45)  # Maximum angle for the global x-axis rotation
# Map -1,1 to theta_min, theta_max
squeeze(f, x) = f[1] + (f[2] - f[1]) * (x + 1) / 2
theta_x_global(t) = squeeze((theta_min, theta_max), cos(omega_x_global * t + phi_x_global))
# Plot the global rotation around the x-axis
plot(0:0.1:5, theta_x_global.(0:0.1:5))


# Parameters for local rotation around the y-axis
phi_y_local = deg2rad(10)
omega_y_local = 40
theta_min_y = deg2rad(-40)  # Minimum angle for the local y-axis rotation
theta_max_y = deg2rad(40)  # Maximum angle for the local y-axis rotation
# Map -1,1 to theta_min_y, theta_max_y
squeeze_y(f, x) = f[1] + (f[2] - f[1]) * (x + 1) / 2
theta_y_local(t) = squeeze_y((theta_min_y, theta_max_y), sin(omega_y_local * t + phi_y_local))
# Plot the local rotation around the y-axis
plot(0:0.1:5, theta_y_local.(0:0.1:5))

t = Observable(0.0)
max_time = 10.0
dt = 0.1/omega_x_global  # Time step for the simulation
max_iterations = Int(max_time/dt)




function update_wings(t)
    frontrightwing[] = State(conversions.axisangle2quat([1, 0, 0], -theta_x_global(t[])))
    hindrightwing[] = State(conversions.axisangle2quat([1, 0, 0], -theta_x_global(t[]+pi/omega_x_global)))
    
    # Apply local rotation around y-axis
    frontrightwing[] = State(conversions.coordinate_transform(conversions.axisangle2quat([0, 1, 0], theta_y_local(t[])), frontrightwing[].q)*frontrightwing[].q)
    hindrightwing[] = State(conversions.coordinate_transform(conversions.axisangle2quat([0, 1, 0], theta_y_local(t[]+pi/omega_y_local)), hindrightwing[].q)*hindrightwing[].q)

    # Reflect the left wings across the XZ plane
    frontleftwing[] = State(reflect(frontrightwing[].q, [0, 1, 0]))  # Reflect the quaternion across the XZ plane
    hindleftwing[] = State(reflect(hindrightwing[].q, [0, 1, 0]))  # Reflect the quaternion across the XZ plane
end

update_wings(t)  # Initial update to set the initial state of the wings
display(s_dragonfly)  # Display the scene


for i in 1:max_iterations
    t[] = t[] + dt  # Update time observable

    # Update the states based on the angular velocities
    # frontrightwing[] = State()
    # hindrightwing[] = State()
    update_wings(t)
    sleep(dt)  # Yield to allow the GUI to update
end


recording = Observable(true)
global io_ref = Ref{Any}(nothing)  # To store the io handle globally

on(events(s_dragonfly).keyboardbutton) do event
    if event.action == Keyboard.press && event.key == Keyboard.p
        recording[] = false
    end
end

@async begin
    sleep(1)  # Wait for the scene to be fully rendered before starting the recording
    record(s_dragonfly, "DragonFly40HzFlapping.mp4"; framerate=Int(1/dt)) do io
        io_ref[] = io  # Store the io handle for use in the listener

        # Listener: record a frame only when wingstate changes and recording is on
        wingstate_listener = on(t) do new_frame
            if recording[] && io_ref[] !== nothing
                recordframe!(io_ref[])
            end
        end

        # Wait until recording is stopped or window is closed
        while isopen(s_dragonfly) && recording[]
            yield()  # Yield to event loop, no need to sleep
        end

        # Clean up
        off(wingstate_listener)
        io_ref[] = nothing
    end
end






v = rand(3) # Random vector for demonstration


q = Quaternion(normalize(rand(4))...)
RotM = conversions.quat2rotmatrix(q)

using BenchmarkTools: @benchmark
@benchmark conversions.rotate_vector(v, q)
@benchmark Point{3, Float64}(RotM * v)
