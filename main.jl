# Import necessary packages
using GLMakie, GeometryBasics, LinearAlgebra, Quaternions
import Quaternions: Quaternion as Quaternion

# Import custom code and utility functions
include("src/State_and_Conversions.jl")
include("src/Rendering,jl")
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

# Lets demonstate a 90 degree rotation around the z-axis in the standard basis
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
windowmanager.display(scene_1, scene_2; names=["Distinct Movements", "Combined Movements"])

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

function interpolate_rotation(wingstate::Observable{State}, axis::AbstractVector{<:Real}, angle::Real; n::Int=100, time::Real=1.0, rate_function=t -> t, islocal::Bool=true)
    # Interpolate rotation of the observable state around a given axis by a given angle
    rotation_quaternion = conversions.axisangle2quat(axis, angle) # Create the rotation quaternion
    if islocal
        rotation_quaternion = conversions.coordinate_transform(rotation_quaternion, wingstate[]) # Convert the local rotation quaternion to the standard basis
    end
    toState = State(rotation_quaternion * wingstate[].q) # Apply the rotation to the current state
    transformations.interpolate_states(wingstate, toState; n=n, time=time, rate_function=rate_function)

    return toState
end

begin
    @async interpolate_rotation(wingstate_1, rand(3), π / 2; time=2.0, rate_function=t -> sin(t * pi / 2)) # function test
    @async interpolate_rotation(wingstate_2, rand(3), π / 2; time=2.0, rate_function=t -> sin(t * pi / 2)) # function test
end

begin
    @async transformations.interpolate_states(wingstate_1, State()) # Reset to the standard state
    @async transformations.interpolate_states(wingstate_2, State()) # Reset to the standard state
end


# Now let's simulate the intrinsic XYZ euler angles (XY'Z'') in the first scene and at the same time simulate the combined effect of the sequence of rotations in the second scene.

function simulate_euler_angles(wingstate_1::Observable{State}, wingstate_2::Observable{State}, alpha::Real, beta::Real, gamma::Real; n::Int=100, time::Real=1.0, rate_function=t -> t)
    # Simulate intrinsic XYZ euler angles (XY'Z'') in the first scene
    # and the combined effect of the sequence of rotations in the second scene
    @async begin
        # First scene: intrinsic XYZ euler angles
        interpolate_rotation(wingstate_1, [1, 0, 0], alpha; n=n, time=time, rate_function=rate_function, islocal=true) # Rotate around local X-axis by alpha
        interpolate_rotation(wingstate_1, [0, 1, 0], beta; n=n, time=time, rate_function=rate_function, islocal=true) # Rotate around local Y-axis by beta
        interpolate_rotation(wingstate_1, [0, 0, 1], gamma; n=n, time=time, rate_function=rate_function, islocal=true) # Rotate around local Z-axis by gamma
    end
    @async begin
        # Second scene: combined effect of the sequence of rotations
        state_quaternion = wingstate_2[].q
        state_quaternion = conversions.coordinate_transform(conversions.axisangle2quat([1, 0, 0], alpha), state_quaternion) * state_quaternion # Convert the local rotation quaternion to the standard basis and apply the rotation to the current state
        state_quaternion = conversions.coordinate_transform(conversions.axisangle2quat([0, 1, 0], beta), state_quaternion) * state_quaternion # Convert the local rotation quaternion to the standard basis and apply the rotation to the current state
        state_quaternion = conversions.coordinate_transform(conversions.axisangle2quat([0, 0, 1], gamma), state_quaternion) * state_quaternion # Convert the local rotation quaternion to the standard basis and apply the rotation to the current state

        toState = State(state_quaternion) # Create the new state with the combined rotation
        transformations.interpolate_states(wingstate_2, toState; n=3n, time=3 * time, rate_function=rate_function) # Interpolate to the new state
    end
end

simulate_euler_angles(wingstate_1, wingstate_2, π / 4, π / 4, π / 4; n=100, time=1.0, rate_function=t -> sin(t * pi / 2))
begin
    @async transformations.interpolate_states(wingstate_1, State()) # Reset to the standard state
    @async transformations.interpolate_states(wingstate_2, State()) # Reset to the standard state
end

# attach a trail to the tip of the [1,1,1] vector in both the wingstates
begin
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

simulate_euler_angles(wingstate_1, wingstate_2, π / 2, π / 2, π / 2; n=100, time=1.0, rate_function=t -> sin(t * pi / 2))
begin
    @async transformations.interpolate_states(wingstate_1, State()) # Reset to the standard state
    @async transformations.interpolate_states(wingstate_2, State()) # Reset to the standard state
end

simulate_euler_angles(wingstate_1, wingstate_2, rand()*2pi, rand()*2pi, rand()*2pi; n=100, time=1.0/3, rate_function=t -> sin(t * pi / 2))

isapprox(buffer_1[][end],buffer_2[][end], atol=1e-10)

begin
    @async transformations.interpolate_states(wingstate_1, State()) # Reset to the standard state
    @async transformations.interpolate_states(wingstate_2, State()) # Reset to the standard state
end