using LinearAlgebra
using Quaternions
using GeometryBasics: Point

# --- State Type and Constructors ---

mutable struct State
    # Define Standard State to be [[0,0,1], [0,1,0], [1,0,0]] and rotation from standard state is null therefore standard q = [1,0,0,0]
    ex::Point{3,Float64}
    ey::Point{3,Float64}
    ez::Point{3,Float64}
    q::Quaternion
end

# State Constructor, given a Quaternion
function State(q::Quaternion)
    q = normalize(q)
    r = quat2rotmatrix(q)
    ex = Point{3,Float64}(r[:, 1])
    ey = Point{3,Float64}(r[:, 2])
    ez = Point{3,Float64}(r[:, 3])
    return State(ex, ey, ez, q)
end

# State Constructor, given state vectors
function State(ex::AbstractVector{<:Real}, ey::AbstractVector{<:Real}, ez::AbstractVector{<:Real})
    ex_f = Point{3,Float64}(ex)
    ey_f = Point{3,Float64}(ey)
    ez_f = Point{3,Float64}(ez)

    # normalize vectors
    ex_f = normalize(ex_f)
    ey_f = normalize(ey_f)
    ez_f = normalize(ez_f)

    # check orthonormality by calculating pairwise dot products
    if abs(dot(ex_f, ey_f)) > 1e-6 || abs(dot(ex_f, ez_f)) > 1e-6 || abs(dot(ey_f, ez_f)) > 1e-6
        error("Vectors are not orthonormal")
    end

    q = rotmatrix2quat([Vector(ex_f), Vector(ey_f), Vector(ez_f)])

    return State(ex_f, ey_f, ez_f, q)
end

# State Constructor, given a rotation matrix
function State(r::AbstractMatrix{<:Real})
    return State(r[:, 1], r[:, 2], r[:, 3])
end

# Default State
State() = State(Quaternion(1.0, 0.0, 0.0, 0.0)) # Default State




# --- Conversions --- 
# --- Conversion Functions ---

@inline function quat_from_axisangle(axis::AbstractVector, theta::Real)
    @assert length(axis) == 3 "Must be a 3-vector"
    s, c = sincos(theta / 2)
    axis = normalize(axis)
    return Quaternion(c, s * axis[1], s * axis[2], s * axis[3])
end
axisangle2quat = quat_from_axisangle

@inline function axisangle_from_quat(q::Quaternion)
    norm_q = norm(q)
    @assert isapprox(norm_q, 1; atol=1e-6) "Quaternion must be unit (norm ≈ 1), got norm = $norm_q"
    w = real(q)
    v = [q.v1, q.v2, q.v3]
    theta = 2 * acos(clamp(w, -1.0, 1.0))
    sin_half_theta = sqrt(1 - w^2)
    if sin_half_theta < 1e-6
        axis = [0.0, 0.0, 1.0]
    else
        axis = normalize(v ./ sin_half_theta)
    end
    return axis, theta
end
quat2axisangle = axisangle_from_quat

@inline function rotate_vector(q::Quaternion, u::AbstractVector)
    @assert length(u) == 3 "Must be a 3-vector"
    return Point{3, Float64}(imag_part(q * Quaternion(0, u[1], u[2], u[3]) * inv(q)))
end
rotate_vector(u::AbstractVector, q::Quaternion) = rotate_vector(q, u)

@inline function quat_from_rotmatrix(dcm::AbstractMatrix{T}) where {T<:Real}
    tr = dcm[1, 1] + dcm[2, 2] + dcm[3, 3]
    if tr > 0
        S = sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (dcm[3, 2] - dcm[2, 3]) / S
        y = (dcm[1, 3] - dcm[3, 1]) / S
        z = (dcm[2, 1] - dcm[1, 2]) / S
    elseif (dcm[1, 1] > dcm[2, 2]) && (dcm[1, 1] > dcm[3, 3])
        S = sqrt(1.0 + dcm[1, 1] - dcm[2, 2] - dcm[3, 3]) * 2
        w = (dcm[3, 2] - dcm[2, 3]) / S
        x = 0.25 * S
        y = (dcm[1, 2] + dcm[2, 1]) / S
        z = (dcm[1, 3] + dcm[3, 1]) / S
    elseif dcm[2, 2] > dcm[3, 3]
        S = sqrt(1.0 + dcm[2, 2] - dcm[1, 1] - dcm[3, 3]) * 2
        w = (dcm[1, 3] - dcm[3, 1]) / S
        x = (dcm[1, 2] + dcm[2, 1]) / S
        y = 0.25 * S
        z = (dcm[2, 3] + dcm[3, 2]) / S
    else
        S = sqrt(1.0 + dcm[3, 3] - dcm[1, 1] - dcm[2, 2]) * 2
        w = (dcm[2, 1] - dcm[1, 2]) / S
        x = (dcm[1, 3] + dcm[3, 1]) / S
        y = (dcm[2, 3] + dcm[3, 2]) / S
        z = 0.25 * S
    end
    return Quaternion(w, x, y, z)
end
rotmatrix2quat = quat_from_rotmatrix

@inline function rotmatrix_from_quat(q::Quaternion)
    s, x, y, z = q.s, q.v1, q.v2, q.v3
    sx, sy, sz = 2s * x, 2s * y, 2s * z
    xx, xy, xz = 2x^2, 2x*y, 2x*z
    yy, yz, zz = 2y^2, 2y*z, 2z^2
    return [
        1 - (yy + zz)  xy - sz        xz + sy
        xy + sz        1 - (xx + zz)  yz - sx
        xz - sy        yz + sx        1 - (xx + yy)
    ]
end
quat2rotmatrix = rotmatrix_from_quat

@inline function axisangle2rotmatrix(axis::AbstractVector, theta::Real)
    # Direct Rodrigues' rotation formula (no quaternions)
    @assert length(axis) == 3 "Must be a 3-vector"
    axis = normalize(axis)
    x, y, z = axis
    s, c = sincos(theta)
    C = 1 - c
    return [
        c + x^2*C     x*y*C - z*s   x*z*C + y*s
        y*x*C + z*s   c + y^2*C     y*z*C - x*s
        z*x*C - y*s   z*y*C + x*s   c + z^2*C
    ]
end
rotmatrix_from_axisangle = axisangle2rotmatrix

# --- Change of Basis and Coordinate Transform Functions ---

@inline function change_of_basis_quaternion(from::Quaternion, to::Quaternion)
    # Gives the quaternion that when applied (used to sandwich between it and it's inverse) to a quaternion in the 'from' basis will convert it to the 'to' basis
    # Example: if v1 in q1 basis maps to v0 in the standard basis, then the change of basis quaternion q_star is such that v2 = q_star * v1 * inv(q_star) will give v2 such that v2 in q2 basis maps to the same physical vector v0 in the standard basis
    return inv(to) * from
end

@inline function coordinate_transform(q::Quaternion, from::Quaternion, to::Quaternion=Quaternion(1.0, 0.0, 0.0, 0.0))
    # Convert quaternion q from 'from' basis to 'to' basis
    change_of_basis = change_of_basis_quaternion(from, to)
    return change_of_basis * q * inv(change_of_basis)
end

# If you have a State type with a .q field (quaternion)
@inline function coordinate_transform(q::Quaternion, from::State, to::State=State())
    return coordinate_transform(q, from.q, to.q)
end

@inline function coordinate_transform(v::AbstractVector{<:Real}, from::Quaternion, to::Quaternion=Quaternion(1.0, 0.0, 0.0, 0.0))
    # Convert vector v from 'from' basis to 'to' basis
    q = Quaternion(0, v...)
    transformed_q = coordinate_transform(q, from, to)
    return Point{3,Float64}(imag_part(transformed_q))
end

@inline function coordinate_transform(v::AbstractVector{<:Real}, from::State, to::State=State())
    return coordinate_transform(v, from.q, to.q)
end

# --- Conversions Struct ---

struct Conversions
    quat_from_axisangle::Function
    axisangle2quat::Function
    axisangle_from_quat::Function
    quat2axisangle::Function
    rotate_vector::Function
    quat_from_rotmatrix::Function
    rotmatrix2quat::Function
    rotmatrix_from_quat::Function
    quat2rotmatrix::Function
    axisangle2rotmatrix::Function
    rotmatrix_from_axisangle::Function
    change_of_basis_quaternion::Function
    coordinate_transform::Function

    function Conversions()
        new(
            quat_from_axisangle,
            axisangle2quat,
            axisangle_from_quat,
            quat2axisangle,
            rotate_vector,
            quat_from_rotmatrix,
            rotmatrix2quat,
            rotmatrix_from_quat,
            quat2rotmatrix,
            axisangle2rotmatrix,
            rotmatrix_from_axisangle,
            change_of_basis_quaternion,
            coordinate_transform
        )
    end
end

# Usage:
# conversions = Conversions()
# conversions.quat_from_axisangle([0,0,1], π/2)
# conversions.quat2rotmatrix(q)