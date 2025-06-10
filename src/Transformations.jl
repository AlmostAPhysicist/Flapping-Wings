using GLMakie, GeometryBasics, LinearAlgebra, Quaternions
import Quaternions: Quaternion as Quaternion
include("State_and_Conversions.jl")


function interpolate_states(from::Observable{State}, to::State; n::Int=100, time::Real=1.0, rate_function=t -> t)
    # Interpolate between two states from_copy to to with n steps and time duration time
    from_copy = from[] # Copy the initial state
    for i in 1:n
        t = i / n
        from[] = State(slerp(from_copy.q, to.q, rate_function(t)))
        sleep(time / n) # Sleep for a short duration to control the speed of the animation
        yield()
    end
end

# add movenemt type to be "smooth" or any of the Euler Angle Convensions (e.g. "ZXZ", "XY'Z''", etc.)

# a long term goal is to have a custom interpolation path within the Volume residing in the 4D space of the quaternionic states instead of a simple SLERP (the shortest of all possible paths)


function interpolate_vector(
    v1::Observable{AbstractVector{<:Real}},
    v2::AbstractVector{<:Real},
    axis::Union{AbstractVector{<:Real},Nothing}=nothing;
    n::Int=100, time::Real=1.0, rate_function=t -> t
)
# Animate rotation of v1 to v2 about axis (or cross(v1, v2) if axis is nothing)
    # In principle, rotate_vector(axisangle2quat(axis, angle), v1) should equal v2
    # We must find this angle for the 2 vectors v1 and v2 for the given axis 
    # If the angle exists, we interpolate between the angle and transform v1 into the vectors given by the intermediate angles.
    # If the angle does not exist, throw an error.
    v_start = copy(v1[])
    v_end = normalize(v2)
    v0 = normalize(v_start)
    if isnothing(axis)
        axis = cross(v0, v_end)
        if norm(axis) < 1e-8
            # Vectors are parallel or anti-parallel
            # Pick any perpendicular axis if anti-parallel
            if dot(v0, v_end) < 0
                axis = abs(v0[1]) < 0.9 ? cross(v0, [1, 0, 0]) : cross(v0, [0, 1, 0])
            else
                # Already aligned, nothing to do
                return
            end
        end
    end
    axis = normalize(axis)
    angle = acos(clamp(dot(v0, v_end), -1.0, 1.0))
    if isnan(angle)
        error("The angle between the vectors is not defined.")
    end
    for i in 1:n
        t = i / n
        interpolated_angle = angle * rate_function(t)
        v_interpolated = rotate_vector(axisangle2quat(axis, interpolated_angle), v_start)
        v1[] = v_interpolated
        sleep(time / n)
    end
end

function combined_rotation(
    wingstate::State,
    rotations...;
    opState::Union{Bool, State}=true,
)
    toStatequaternion = wingstate.q # Start with the current state
    for rot in rotations
        if rot isa Quaternion
            op = opState
            quat = rot
        elseif rot isa Tuple && length(rot) == 2 && rot[1] isa Quaternion
            quat, op = rot
        else
            error("Each rotation must be a Quaternion or a Tuple{Quaternion, Bool/State}")
        end

        if op isa Bool
            op_state = op ? toStatequaternion : State()
        elseif op isa State
            op_state = op
        else
            error("Operation state must be Bool or State")
        end

        rot_quat = conversions.coordinate_transform(quat, op_state)
        toStatequaternion = rot_quat * toStatequaternion
    end
    return State(toStatequaternion)
end


function combine_rotation_interpolations(
    wingstate::Observable{State},
    rotations...;
    opState::Union{Bool, State}=true,
    n::Int=100,
    time::Real=1.0,
    rate_function::Function = t -> t,
    pause_time::Real = 0.0
)
    for rot in rotations
        if rot isa Quaternion
            op = opState
            quat = rot
        elseif rot isa Tuple && length(rot) == 2 && rot[1] isa Quaternion
            quat, op = rot
        else
            error("Each rotation must be a Quaternion or a Tuple{Quaternion, Bool/State}")
        end

        if op isa Bool
            op_state = op ? wingstate[] : State()
        elseif op isa State
            op_state = op
        else
            error("Operation state must be Bool or State")
        end

        rot_quat = conversions.coordinate_transform(quat, op_state)
        toState = State(rot_quat * wingstate[].q)
        transformations.interpolate_states(wingstate, toState; n=n, time=time, rate_function=rate_function)
        sleep(pause_time)
    end
end

"""
    Transformations()

A struct that provides convenient access to interpolate_states and interpolate_vector.
"""
struct Transformations
    rotate_vector::Function
    interpolate_states::Function
    interpolate_vector::Function
    combined_rotation::Function
    combine_rotation_interpolations::Function
end

Transformations() = Transformations(rotate_vector, interpolate_states, interpolate_vector, combined_rotation, combine_rotation_interpolations)

# Usage:
# transformations = Transformations()
# transformations.interpolate_states(from, to; n=100)
# transformations.interpolate_vector(v1, v2)


