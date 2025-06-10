# Import necessary packages
using GLMakie, GeometryBasics, LinearAlgebra, Quaternions
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

# Initialize the scene
scene = Scene(camera=cam3d!)

# Create a display screen for the scene
windowmanager.closeall() # Close all windows before displaying new ones
windowmanager.display(scene; names=["Main Scene"])


renderer.drawState!(scene) # would create a default State coordinate arrows within the scene


# Create an observable state for animation
wingstate = Observable(State()) # Create an observable state


# Create a render custom vectors attached to the observable state
custom_vectors_to_show = [Observable(Point3f(1, 0, 0)), Observable(Point3f(0, 1, 0)), Observable(Point3f(0, 0, 1)), Observable(Point3f(1, 1, 1))]
custom_vector_colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0, 0)] # Colors for the custom vectors

attached_vectors = renderer.attach2State(custom_vectors_to_show, wingstate) # Creates a vector of Observable Points that attaches the custom vectors to the observable state

# Draw the state with attached vectors
for i in eachindex(attached_vectors)
    meshscatter!(scene, (0, 0, 0), marker=lift(p -> Arrow(p).mesh, attached_vectors[i]), markersize=0.75, color=(custom_vector_colors[i]..., 0.5), transparency=true)
end


# Transformations of the observable state to some other state
toState = State(conversions.axisangle2quat([0, 0, 1], π / 4)) # Define the target state for interpolation
transformations.interpolate_states(wingstate, toState; n=100, time=2.0, rate_function=t -> sin(t * pi / 2))

toState = State(conversions.axisangle2quat(rand(3), rand() * 2π)) # Define the target state for interpolation
transformations.interpolate_states(wingstate, toState; n=100, time=2.0, rate_function=t -> sin(t * pi / 2))

#----------------------------------------------------------------------

# Transformation of the observable state for a rotation in a specific basis 

# Lets demonstate a 90 degree rotation around the y-axis in the standard basis
rotation_quaternion = conversions.axisangle2quat([0, 1, 0], π / 2) # 90 degree rotation around z-axis

# For Quaternions, if q1 and q2 are two quaternions, then q2*q1  is the quaternion representing the rotation of q1 followed by q2.
toState = State(rotation_quaternion * wingstate[].q) # Apply the rotation to the current state

transformations.interpolate_states(wingstate, toState; n=100, time=2.0, rate_function=t -> sin(t * pi / 2))

# But what if we want to rotate in a different basis?
# We can use the coordinate_transform functino that I defined

# Let's say we want to rotate 180 degrees around the x-axis in the basis defined by the current wingstate itself.
rotation_quaternion_local = conversions.axisangle2quat([1, 0, 0], π) # 180 degree rotation around x-axis
rotation_quaternion_standard_basis = conversions.coordinate_transform(rotation_quaternion_local, wingstate[]) # Convert the rotation quaternion to the standard basis
toState = State(rotation_quaternion_standard_basis * wingstate[].q) # Apply the rotation to the current state in the standard basis
transformations.interpolate_states(wingstate, toState; n=100, time=2.0, rate_function=t -> sin(t * pi / 2))

# And we can chose any axis we want 
rotation_quaternion_local = conversions.axisangle2quat([1, 1, 1], π) # 180 degree rotation around x-axis
rotation_quaternion_standard_basis = conversions.coordinate_transform(rotation_quaternion_local, wingstate[]) # Convert the rotation quaternion to the standard basis
toState = State(rotation_quaternion_standard_basis * wingstate[].q) # Apply the rotation to the current state in the standard basis
transformations.interpolate_states(wingstate, toState; n=100, time=2.0, rate_function=t -> sin(t * pi / 2))

#------------------------------------------------------------------------

# Adding a little trail at the tip of the [1,1,1] vector in the wingstate 
# Initialize a circular buffer to store the trail points
using DataStructures: CircularBuffer
n = 200 # Buffer size
buffer = CircularBuffer{Point{3,Float64}}(n)
for _ in 1:n
    push!(buffer, copy(attached_vectors[4][]) .* 0.75) # Assuming the 4th vector is the one we want to trail
end
buffer = Observable(buffer)

# Use buffer in plot
trail = lines!(scene, lift(x -> collect(x), buffer), color=[GLMakie.RGBA(0, 0, 0, 1 - i / n) for i in 0:n-1], linewidth=2)

# add a lister function to update the buffer whenever the vector changes.
trail_tracker = on(attached_vectors[4]) do new_point
    push!(buffer[], new_point .* 0.75) # Update the buffer with the new point, scaled down
    notify(buffer) # Notify the observable to update the plot
end

rotation_quaternion_local = conversions.axisangle2quat(rand(3), π) # 180 degree rotation around x-axis
rotation_quaternion_standard_basis = conversions.coordinate_transform(rotation_quaternion_local, wingstate[]) # Convert the rotation quaternion to the standard basis
toState = State(rotation_quaternion_standard_basis * wingstate[].q) # Apply the rotation to the current state in the standard basis
transformations.interpolate_states(wingstate, toState; n=100, time=2.0, rate_function=t -> sin(t * pi / 2))


off(trail_tracker) # Stop tracking the trail when done
delete!(scene, trail)  # Clean up the scene by removing the trail
windowmanager.closeall() # Close all windows when done

#------------------------------------------------------------------------
#------------------------------------------------------------------------

# We can have more visualization ways of combining different rotations.

scene_1 = Scene(camera=cam3d!)
scene_2 = Scene(camera=cam3d!)
windowmanager.closeall() # Close all windows before displaying new ones
windowmanager.display(scene_1, scene_2; names=["Local", "Global"])

# Let's showcase a basic construction of Euler Angles 
# I would be simulating intrinsic XYZ euler angles (XY'Z'')

# Draw the standard basis on both scenes 
renderer.drawState!(scene_1)
renderer.drawState!(scene_2)


wingstate_1 = Observable(State()) # Create an observable state
# Create a render custom vectors attached to the observable state
custom_vectors_to_show_1 = [Observable(Point3f(1, 0, 0)), Observable(Point3f(0, 1, 0)), Observable(Point3f(0, 0, 1)), Observable(Point3f(1, 1, 1))]
custom_vector_colors_1 = [(1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0, 0)] # Colors for the custom vectors

attached_vectors_1 = renderer.attach2State(custom_vectors_to_show_1, wingstate_1) # Creates a vector of Observable Points that attaches the custom vectors to the observable state

# Draw the state with attached vectors
for i in eachindex(attached_vectors_1)
    meshscatter!(scene_1, (0, 0, 0), marker=lift(p -> Arrow(p).mesh, attached_vectors_1[i]), markersize=0.75, color=(custom_vector_colors_1[i]..., 0.5), transparency=true)
end

wingstate_2 = Observable(State()) # Create an observable state
# Create a render custom vectors attached to the observable state
custom_vectors_to_show_2 = [Observable(Point3f(1, 0, 0)), Observable(Point3f(0, 1, 0)), Observable(Point3f(0, 0, 1)), Observable(Point3f(1, 1, 1))]
custom_vector_colors_2 = [(1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0, 0)] # Colors for the custom vectors

attached_vectors_2 = renderer.attach2State(custom_vectors_to_show_2, wingstate_2) # Creates a vector of Observable Points that attaches the custom vectors to the observable state

# Draw the state with attached vectors
for i in eachindex(attached_vectors_2)
    meshscatter!(scene_2, (0, 0, 0), marker=lift(p -> Arrow(p).mesh, attached_vectors_2[i]), markersize=0.75, color=(custom_vector_colors_2[i]..., 0.5), transparency=true)
end

begin
    # Example: rotate around an axis by π/2 in the local frame and global frame
    axis = [1,0,0]
    angle = π / 2
    rotation = conversions.axisangle2quat(axis, angle)
    @async transformations.combine_rotation_interpolations(wingstate_1, rotation; n=100, time=2.0, rate_function=t -> sin(t * pi / 2), opState=true)
    @async transformations.combine_rotation_interpolations(wingstate_2, rotation; n=100, time=2.0, rate_function=t -> sin(t * pi / 2), opState=false)
end

begin
    # Example: rotate around a random axis by π/2 in the local frame
    axis = [0,1,0]
    angle = π / 2
    rotation = conversions.axisangle2quat(axis, angle)
    @async transformations.combine_rotation_interpolations(wingstate_1, rotation; n=100, time=2.0, rate_function=t -> sin(t * pi / 2), opState=true)
    @async transformations.combine_rotation_interpolations(wingstate_2, rotation; n=100, time=2.0, rate_function=t -> sin(t * pi / 2), opState=false)
end

begin
    @async transformations.interpolate_states(wingstate_1, State()) # Reset to the standard state
    @async transformations.interpolate_states(wingstate_2, State()) # Reset to the standard state
end


# ----------------------------------------------------------------------
# ----------------------------------------------------------------------

# TRAIL
# attach a trail to the tip of the [1,1,1] vector in both the wingstates
begin
    using DataStructures: CircularBuffer
    n = 300 # Buffer size

    buffer_1 = CircularBuffer{Point{3,Float64}}(n)
    for _ in 1:n
        push!(buffer_1, copy(attached_vectors_1[4][]) .* 0.75) # Assuming the 4th vector is the one we want to trail
    end
    buffer_1 = Observable(buffer_1)

    # Use buffer in plot
    trail_1 = lines!(scene_1, lift(x -> collect(x), buffer_1), color=[GLMakie.RGBA(0, 0, 0, 1 - i / n) for i in 0:n-1], linewidth=2, fxaa=true)

    # add a lister function to update the buffer whenever the vector changes.
    trail_tracker_1 = on(attached_vectors_1[4]) do new_point
        push!(buffer_1[], new_point .* 0.75) # Update the buffer with the new point, scaled down
        notify(buffer_1) # Notify the observable to update the plot
    end

    buffer_2 = CircularBuffer{Point{3,Float64}}(n)
    for _ in 1:n
        push!(buffer_2, copy(attached_vectors_2[4][]) .* 0.75) # Assuming the 4th vector is the one we want to trail
    end
    buffer_2 = Observable(buffer_2)

    # Use buffer in plot
    trail_2 = lines!(scene_2, lift(x -> collect(x), buffer_2), color=[GLMakie.RGBA(0, 0, 0, 1 - i / n) for i in 0:n-1], linewidth=2, fxaa=true)

    # add a lister function to update the buffer whenever the vector changes.
    trail_tracker_2 = on(attached_vectors_2[4]) do new_point
        push!(buffer_2[], new_point .* 0.75) # Update the buffer with the new point, scaled down
        notify(buffer_2) # Notify the observable to update the plot
    end
end

# Now let's simulate the intrinsic XYZ euler angles (XY'Z'') in the first scene and at the same time simulate the combined effect of the sequence of rotations in the second scene.
windowmanager.closeall() # Close all windows before displaying new ones
windowmanager.display(scene_1, scene_2; names=["XY'Z'' Movements (Local)", "Combined Movements (Global)"])

function simulate_euler_angles(wingstate::Observable{State}, alpha::Real, beta::Real, gamma::Real; n::Int=100, time::Real=1.0, rate_function=t -> t, islocal::Bool=true)
    rotations = [
        (conversions.axisangle2quat([1, 0, 0], alpha), islocal),
        (conversions.axisangle2quat([0, 1, 0], beta), islocal),
        (conversions.axisangle2quat([0, 0, 1], gamma), islocal)
    ]
    transformations.combine_rotation_interpolations(wingstate, rotations...; n=n, time=time, rate_function=rate_function)
end


# Demonstration: asynchronously animate both wingstates 
# wingstate_1 will simulate intrinsic XYZ euler angles, while wingstate_2 will simulate the combined effect of the sequence of rotations.
begin
    alpha = rand() * 2 * pi
    beta = rand() * 2 * pi
    gamma = rand() * 2 * pi
    islocal = Bool(rand() > 0.5) # Randomly choose between local and standard basis rotations
    # Set to true for local rotations, false for standard basis rotations
    
    @async simulate_euler_angles(wingstate_1, alpha, beta, gamma; n=100, time=1.0, rate_function=t -> sin(t * pi / 2), islocal=islocal)
    @async transformations.interpolate_states(wingstate_2, transformations.combined_rotation(wingstate_2[], 
        (conversions.axisangle2quat([1, 0, 0], alpha), islocal),
        (conversions.axisangle2quat([0, 1, 0], beta), islocal),
        (conversions.axisangle2quat([0, 0, 1], gamma), islocal));
        n=300, time=3.0, rate_function=t -> sin(t * pi / 2)
    )
end

begin
    @async transformations.interpolate_states(wingstate_1, State()) # Reset to the standard state
    @async transformations.interpolate_states(wingstate_2, State()) # Reset to the standard state
end





# Demonstrate XY'Z'' vs ZYX

windowmanager.closeall() # Close all windows before displaying new ones
windowmanager.display(scene_1, scene_2; names=["XY'Z'' Movements (Local)", "ZYX Movements (Global)"])
begin
    @async transformations.interpolate_states(wingstate_1, State()) # Reset to the standard state
    @async transformations.interpolate_states(wingstate_2, State()) # Reset to the standard state
end

begin
    alpha = π / 3 # 45 degrees around x-axis
    beta = -π # 45 degrees around y-axis
    gamma = π / 4 # 45 degrees around z-axis

    # alpha, beta, gamma = rand(3) * 2 * pi # Random angles for demonstration


    @async transformations.combine_rotation_interpolations(wingstate_1, 
        (conversions.axisangle2quat([1, 0, 0], alpha), true), # Rotate around x-axis in local frame
        (conversions.axisangle2quat([0, 1, 0], beta), true), # Rotate around y-axis in local frame
        (conversions.axisangle2quat([0, 0, 1], gamma), true); # Rotate around z-axis in local frame
        n=100, time=2.0, rate_function=t -> sin(t * pi / 2)
    )
    current_state_2 = wingstate_2[] # Get the current state of wingstate_2

    @async transformations.combine_rotation_interpolations(wingstate_2, 
        (conversions.axisangle2quat([0, 0, 1], gamma), current_state_2), # Rotate around z-axis in standard frame
        (conversions.axisangle2quat([0, 1, 0], beta), current_state_2), # Rotate around y-axis in standard frame
        (conversions.axisangle2quat([1, 0, 0], alpha), current_state_2); # Rotate around x-axis in standard frame
        n=100, time=2.0, rate_function=t -> sin(t * pi / 2)
    )
end

isapprox(buffer_1[][end],buffer_2[][end], atol=1e-10)

begin
    @async transformations.interpolate_states(wingstate_1, State()) # Reset to the standard state
    @async transformations.interpolate_states(wingstate_2, State()) # Reset to the standard state
end



# --------------------------------------------------------------------------
# ---------------------------------------------------------------------------

# MATRIX FACTORIZATION/DECOMPOSITION
# Let's demonstrate the matrix decomposition of a smooth rotation through axis-angle representation into an intrinsic XYZ euler angles representation.

windowmanager.closeall() # Close all windows before displaying new one
windowmanager.display(scene_1, scene_2; names=["Smooth Movements by Axis-Angle", "Matrix Decomposition (local XYZ)"])

function euler_angles_xyz(R::AbstractMatrix{<:Real})
    a11, a12, a13 = R[1,1], R[1,2], R[1,3]
    a21, a22, a23 = R[2,1], R[2,2], R[2,3]
    a31, a32, a33 = R[3,1], R[3,2], R[3,3]

    if abs(a13) != 1
        θ = asin(a13)
        φ = atan(-a23, a33)
        ψ = atan(-a12, a11)
    elseif a13 == 1
        θ = π/2
        φ = atan(a21, a22)
        ψ = 0  # Arbitrary; we pick 0
    elseif a13 == -1
        θ = -π/2
        φ = atan(-a21, -a22)
        ψ = 0  # Arbitrary; we pick 0
    end

    return φ, θ, ψ  # Local XYZ angles
end

# Simulate a smooth rotation about an axis for a given angle in one scene and in the other scene, decompose the rotation matrix into intrinsic XYZ euler angles.

begin 
    # Simulate a smooth rotation about an axis for a given angle in one scene
    axis = normalize(rand(3)) # Rotate around x-axis
    angle = rand()*2pi # 45 degrees
    rotation_quaternion = conversions.axisangle2quat(axis, angle) # Convert to quaternion

    @async transformations.combine_rotation_interpolations(wingstate_1, rotation_quaternion; n=300, time=3.0, rate_function=t -> sin(t * pi / 2), opState=true)

    # In the other scene, decompose the rotation matrix into intrinsic XYZ euler angles
    alpha, beta, gamma = euler_angles_xyz(conversions.quat2rotmatrix(rotation_quaternion)) # Decompose the rotation matrix into intrinsic XYZ euler angles
    @async transformations.combine_rotation_interpolations(wingstate_2, 
        (conversions.axisangle2quat([1, 0, 0], alpha), true), # Rotate around x-axis in local frame
        (conversions.axisangle2quat([0, 1, 0], beta), true), # Rotate around y-axis in local frame
        (conversions.axisangle2quat([0, 0, 1], gamma), true); # Rotate around z-axis in local frame
        n=100, time=1.0, rate_function=t -> sin(t * pi / 2)
    )
end
begin
    @async transformations.interpolate_states(wingstate_1, State()) # Reset to the standard state
    @async transformations.interpolate_states(wingstate_2, State()) # Reset to the standard state
end


# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

# Demonstrate the SLERP (Spherical Linear Interpolation) between two quaternionic states in the scene by smoothly moving in one scene and decomposing the rotation at each iteration in the other scene.

windowmanager.closeall() # Close all windows before displaying new ones
windowmanager.display(scene_1, scene_2; names=["SLERP (Smooth Movement)", "Decomposition of SLERP"])

begin
    # define the next quaternionic state to move to
    q1 = Quaternion(rand(4)...) # Initial quaternionic state

    # transition wingstate_1 to the next quaternionic state through SLERP 
    # transition wingstate_2 to the next quaternionic state through applying operations computed from the decomposition of the transformation quaternion from current state to the next state in the interpolation
    
    
    @async transformations.interpolate_states(wingstate_1, State(q1); n=3000, time=5.0, rate_function=t -> sin(t * pi / 2))

    @async begin
        initial_state = wingstate_2[].q # Get the current state of wingstate_2
        n = 500
        for i in range(0, 1, length=n)
            # Compute the Euler Angles to rotate in local XYZ basis to reach next interpolation state from the current state
            rotation_quaternion_global_space = sqrt(slerp(initial_state, q1, i) * inv(wingstate_2[].q)) # This is the quaternion that represents the rotation from the current state to the next state in the global space
            rotation_quaternion_local_space = conversions.coordinate_transform(rotation_quaternion_global_space, State(), wingstate_2[]) # This is the quaternion that represents the rotation from the current state to the next state in the local space
            alpha, beta, gamma = euler_angles_xyz(conversions.quat2rotmatrix(rotation_quaternion_local_space)) # Decompose the rotation matrix into intrinsic XYZ euler angles
            # Apply the rotation to the current state in the local space
            transformations.combine_rotation_interpolations(wingstate_2, 
                (conversions.axisangle2quat([1, 0, 0], alpha), true), # Rotate around x-axis in local frame
                (conversions.axisangle2quat([0, 1, 0], beta), true), # Rotate around y-axis in local frame
                (conversions.axisangle2quat([0, 0, 1], gamma), true); # Rotate around z-axis in local frame
                n=1000÷n, time=5.0/3n, rate_function=t -> sin(t * pi / 2)
            )
        end
    end
end

isapprox(buffer_1[][end],buffer_2[][end], atol=1e-5)
println(buffer_1[][end])
println(buffer_2[][end]) 
# Check if the final points are approximately equal

begin
    @async transformations.interpolate_states(wingstate_1, State()) # Reset to the standard state
    @async transformations.interpolate_states(wingstate_2, State()) # Reset to the standard state
end



# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

# Finally, demonstrate the rendering of a 3D object that is attached to the observable state and rotates with it.
using GLMakie
using FileIO
using LinearAlgebra

wing = load(assetpath("D:/Programming/Flapping Wing Flying Research/Models/edited_wing.STL"))
s = Scene(camera=cam3d!, size=(1920, 1080))
windowmanager.closeall() # Close all windows before displaying new ones
windowmanager.display(s; names=["Wing Mesh"])
drawState!(s; alpha=0.5)

wingstate = Observable(State()) # Create an observable state for the wing mesh


# Create an observable mesh that rotates with vector_state
obswingmesh = lift(wingstate) do st
    R = quat2rotmatrix(st.q)
    positions = wing.vertex_attributes[1]
    rotated_positions = [Point3f(R * Vec3f(p)) for p in positions]
    faces = wing.faces
    GeometryBasics.mesh(rotated_positions, faces)
end

wing_mesh_scatter = meshscatter!(s, (0,0,0), marker=obswingmesh, markersize=0.0075, color=:gold, transparency=true, alpha=0.5)

# Create a render custom vectors attached to the observable state
custom_vectors_to_show = [Point3(1.0, 0.0, 0.0), Point3(0.0, 1.0, 0.0), Point3(0.0, 0.0, 1.0)]
custom_vector_colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)] # Colors for the custom vectors

attached_vectors = renderer.attach2State(custom_vectors_to_show, wingstate) # Creates a vector of Observable Points that attaches the custom vectors to the observable state

# Draw the state with attached vectors
for i in eachindex(attached_vectors)
    meshscatter!(s, (0, 0, 0), marker=lift(p -> Arrow(p).mesh, attached_vectors[i]), markersize=0.75, color=(custom_vector_colors[i]..., 0.5), transparency=true)
end

# Animate the wing mesh by rotating it around the ±60 degrees around the global x-axis as well as 0-30 degreesglobal y-axis

s1 = State()
s1 = State(conversions.axisangle2quat([1, 0, 0], π / 3) * s1.q) # Rotate around x-axis by 60 degrees

s2 = State()
s2 = State(conversions.axisangle2quat([1, 0, 0], -π / 3) * s2.q) # Rotate around x-axis by -60 degrees
s2 = State(conversions.axisangle2quat([0, 1, 0], π / 6) * s2.q) # Rotate around y-axis by 30 degrees

wingstate[] = s1 # Set the initial state of the wing mesh


iterations = 10
n_per_iteration = 50 # Number of frames per iteration
time_per_iteration = 0.01 # Time for each iteration in seconds
for t in 1:iterations
    # Interpolate the wing mesh state between s1 and s2
    toState = iseven(t) ? s1 : s2 # Alternate between s1 and s2
    transformations.interpolate_states(wingstate, toState; n=n_per_iteration, time=time_per_iteration, rate_function=t -> sin(t * pi / 2))
    notify(wingstate) # Notify the observable to update the plot
end





#----------------------------------------------------------------------------
recording = Observable(true)
global io_ref = Ref{Any}(nothing)  # To store the io handle globally

on(events(s).keyboardbutton) do event
    if event.action == Keyboard.press && event.key == Keyboard.q
        recording[] = false
    end
end

## Recorder
# Open the video file and set up the listener ONCE
@async begin
    record(s, "Revised_Wing_Motion_Demo.mp4"; framerate=n_per_iteration/time_per_iteration) do io
        io_ref[] = io  # Store the io handle for use in the listener

        # Listener: record a frame only when wingstate changes and recording is on
        wingstate_listener = on(wingstate) do new_state
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