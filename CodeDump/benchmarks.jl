using Quaternions, FileIO
using LinearAlgebra

# Import custom code and utility functions
include("..src/State_and_Conversions.jl")

# Initialize the custom structs and utilities
conversions = Conversions()

v = rand(3) # Random vector for demonstration


q = Quaternion(normalize(rand(4))...)
RotM = conversions.quat2rotmatrix(q)


println("Vector v: ", v)
println("Quaternion q: ", q)
println("Rotation Matrix RotM: ", RotM)
println("Rotating vector v using Quaternions and Matrix Multiplication:")
conversions.rotate_vector(v, q) ≈ Point{3, Float64}(RotM * v) ? println("Results match!") : println("Results do not match!")

println("\n\tcomputing Benchmarks...\n")
using BenchmarkTools: @benchmark
b = @benchmark conversions.rotate_vector(v, q)
c = @benchmark Point{3, Float64}(RotM * v)

io = IOBuffer()
show(io, "text/plain", b)
s1 = String(take!(io))
show(io, "text/plain", c)
s2 = String(take!(io))

println("Benchmark using Quaternions:")
println(s1)
println("\n")
println("Benchmark using Matrix Multiplication:")
println(s2)

