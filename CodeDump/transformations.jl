using LinearAlgebra
using GLMakie: Point3  # Assuming Point{3,Float64} is from GeometryBasics

# Helper function to convert Point{3,Float64} to Vector{Float64}
to_vector(p::Point) = [p.data...]
to_point(v::AbstractVector{<:Real}) = Point3{Float64}(v[1], v[2], v[3])

# Computes weights to express a 3D point as a linear combination of three 3D basis points
function find_linear_combination_weights(p::Union{AbstractVector{<:Real}, Point3{Float64}}, p1::Union{AbstractVector{<:Real}, Point3{Float64}}, p2::Union{AbstractVector{<:Real}, Point3{Float64}}, p3::Union{AbstractVector{<:Real}, Point3{Float64}})
    p_vec = p isa Point3 ? to_vector(p) : p
    p1_vec = p1 isa Point3 ? to_vector(p1) : p1
    p2_vec = p2 isa Point3 ? to_vector(p2) : p2
    p3_vec = p3 isa Point3 ? to_vector(p3) : p3
    
    @assert length(p_vec) == 3 && length(p1_vec) == 3 && length(p2_vec) == 3 && length(p3_vec) == 3 "All points must be 3D vectors"
    
    A = hcat(p1_vec, p2_vec, p3_vec)
    try
        weights = A \ p_vec
        return weights
    catch e
        error("The points are linearly dependent or the system is singular: $e")
    end
end

# Maps points from one basis (three points) to another, assuming fixed origin
function get_linear_transformation(points1::AbstractVector{<:Union{AbstractVector{<:Real}, Point3{Float64}}}, points2::AbstractVector{<:Union{AbstractVector{<:Real}, Point3{Float64}}})
    @assert length(points1) == 3 && length(points2) == 3 "Each point set must contain exactly 3 points"
    points1_vec = [p isa Point3 ? to_vector(p) : p for p in points1]
    points2_vec = [p isa Point3 ? to_vector(p) : p for p in points2]
    @assert all(length(p) == 3 for p in points1_vec) && all(length(p) == 3 for p in points2_vec) "All points must be 3D vectors"
    
    function f(p::Union{AbstractVector{<:Real}, Point3{Float64}})
        p_vec = p isa Point3 ? to_vector(p) : p
        weights = find_linear_combination_weights(p_vec, points1_vec...)
        result = weights[1] * points2_vec[1] + weights[2] * points2_vec[2] + weights[3] * points2_vec[3]
        return p isa Point3 ? to_point(result) : result
    end
    return f
end

# Computes a full affine transformation (linear + translation) using four points per set
function get_affine_transformation(points1::AbstractVector{<:AbstractVector{<:Real}}, points2::AbstractVector{<:AbstractVector{<:Real}})
    @assert length(points1) == 4 && length(points2) == 4 "Each point set must contain exactly 4 points"
    to_vector(p::Point) = [p.data...]
    to_point(v::AbstractVector{<:Real}) = Point3{Float64}(v[1], v[2], v[3])
    points1_vec = [p isa Point3 ? to_vector(p) : p for p in points1]
    points2_vec = [p isa Point3 ? to_vector(p) : p for p in points2]
    @assert all(length(p) == 3 for p in points1_vec) && all(length(p) == 3 for p in points2_vec) "All points must be 3D vectors"
    
    # Form homogeneous coordinate matrices (4x4 system: 3D points + translation)
    A = hcat([vcat(p, 1.0) for p in points1_vec]...)
    B = hcat([vcat(p, 1.0) for p in points2_vec]...)
    
    try
        # Solve for transformation matrix T where A * T = B
        T = B / A  # Corrected: B = T * A, so T = B * inv(A)
        function f(p::AbstractVector{<:Real})
            p_vec = p isa Point3 ? to_vector(p) : p
            @assert length(p_vec) == 3 "Input point must be 3D"
            p_homogeneous = vcat(p_vec, 1.0)
            result = T * p_homogeneous
            return p isa Point3 ? to_point(result[1:3]) : result[1:3]
        end
        return f
    catch e
        error("The points are linearly dependent or the system is singular: $e")
    end
end

# Alternative

# Computes a full affine transformation (linear + translation) using four points per set
function get_affine_transformation(points1::AbstractVector{<:AbstractVector{<:Real}}, points2::AbstractVector{<:AbstractVector{<:Real}})
    @assert length(points1) == 3 && length(points2) == 3 "Each point set must contain exactly 3 points"
    to_vector(p::Point) = [p.data...]
    to_point(v::AbstractVector{<:Real}) = Point3{Float64}(v[1], v[2], v[3])
    points1_vec = [p isa Point3 ? to_vector(p) : p for p in points1]
    points2_vec = [p isa Point3 ? to_vector(p) : p for p in points2]
    @assert all(length(p) == 3 for p in points1_vec) && all(length(p) == 3 for p in points2_vec) "All points must be 3D vectors"
    
    # Form homogeneous coordinate matrices (4x4 system: 3D points + translation)
    A = hcat([vcat(p, 1.0) for p in points1_vec]...)
    B = hcat([vcat(p, 1.0) for p in points2_vec]...)
    
    try
        # Solve for transformation matrix T where A * T = B
        T = B / A  # Corrected: B = T * A, so T = B * inv(A)
        function f(p::AbstractVector{<:Real})
            p_vec = p isa Point3 ? to_vector(p) : p
            @assert length(p_vec) == 3 "Input point must be 3D"
            p_homogeneous = vcat(p_vec, 1.0)
            result = T * p_homogeneous
            return p isa Point3 ? to_point(result[1:3]) : result[1:3]
        end
        return f
    catch e
        error("The points are linearly dependent or the system is singular: $e")
    end
end


# Define points
points1 = rand(Point3{Float64}, 3)  # Randomly generated points
points2 = rand(Point3{Float64}, 3)  # Randomly generated points

# Get affine transformation
f = get_affine_transformation(points1, points2)
f_inv = get_affine_transformation(points2, points1)

transformed_points = [f(p) for p in points1]
for i in eachindex(transformed_points)
    #check reversed transformation
    println(f_inv(transformed_points[i]) == points1[i])  # Should be true
end

# Test transformations
println(f(Point3(0,0,0)))  # Should print Point3(1,1,1)
println(f(Point3(1,0,0)))  # Should print Point3(2,1,1)
println(f(Point3(0,1,0)))  # Should print Point3(1,2,1)
println(f(Point3(0,0,1)))  # Should print Point3(1,1,2)
println(f(Point3(1,1,0)))  # Should print Point3(2,2,2) (linear extrapolation)


sum(points2 .* [-1,1,1])