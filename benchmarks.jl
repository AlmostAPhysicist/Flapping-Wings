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

v = rand(3) # Random vector for demonstration


q = Quaternion(normalize(rand(4))...)
RotM = conversions.quat2rotmatrix(q)

using BenchmarkTools: @benchmark
@benchmark conversions.rotate_vector(v, q)
@benchmark Point{3, Float64}(RotM * v)
