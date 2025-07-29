using Quaternions, FileIO
using LinearAlgebra

# Import custom code and utility functions
include("../src/State_and_Conversions.jl")

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

# --------------------------------

# Take 1000 samples each for 1000 iterations for benchmarking rotations of a vector using a Quaternion and a Rotation Matrix
# Plot the results using GLMakie.jl as a line plot with 1000 datapoints for the mean time taken for each iteration, also show the standard deviation as a shaded area around the mean line
using GLMakie
using Statistics

iterations = 1100
samples = 1100

times_quat = zeros(samples, iterations)
times_matrix = zeros(samples, iterations)
for i in 1:samples
    random_vectors = [rand(3) for _ in 1:iterations]
    rotation_quaternions = [Quaternion(normalize(rand(4))...) for _ in 1:iterations]
    rotation_matrices = [conversions.quat2rotmatrix(q) for q in rotation_quaternions]

    for j in 1:iterations
        t_i_quat = time_ns()
        conversions.rotate_vector(random_vectors[j], rotation_quaternions[j])
        t_quat = time_ns() - t_i_quat
        times_quat[i, j] = t_quat

        t_i_matrix = time_ns()
        Point{3, Float64}(rotation_matrices[j] * random_vectors[j])
        t_matrix = time_ns() - t_i_matrix
        times_matrix[i, j] = t_matrix
    end

end

# leave out the first 100 iterations to avoid warm-up effects 
# leave out the first 100 samples to avoid warm-up effects
times_quat = times_quat[101:end, 101:end]
times_matrix = times_matrix[101:end, 101:end]

times_quat = times_quat ./ 1e9 # Convert nanoseconds to seconds
times_matrix = times_matrix ./ 1e9 # Convert nanoseconds to seconds
mean_times_quat = mean(times_quat, dims=1)
mean_times_matrix = mean(times_matrix, dims=1)
std_times_quat = std(times_quat, dims=1)
std_times_matrix = std(times_matrix, dims=1)

# Plotting the results
set_theme!(theme_minimal())
set_theme!(theme_latexfonts())
fig = Figure(size=(800, 600), fontsize=24)
ax = Axis(fig[1, 1], xlabel="Sample", ylabel="Time (seconds)", title=L"\text{Mean time taken for } 1000 \text{ rotation calculations}")
lines!(ax, 1:iterations-100, mean_times_matrix[1, :], color=:red, label="Matrix Rotation")
band!(ax, 1:iterations-100, mean_times_matrix[1, :], max.(1e-10, mean_times_matrix[1, :] .+ std_times_matrix[1, :]), color=:red, alpha=0.2, label="Matrix Rotation Standard Deviation")
lines!(ax, 1:iterations-100, mean_times_quat[1, :], color=:blue, label="Quaternion Rotation")
band!(ax, 1:iterations-100, mean_times_quat[1, :], max.(1e-10, mean_times_quat[1, :] .+ std_times_quat[1, :]), color=:blue, alpha=0.2, label="Quaternion Standard Deviation")
# legend = Legend(fig[1, 2], ax)
axislegend(ax)

display(fig)

overall_mean_quat = mean(mean_times_quat)
overall_mean_matrix = mean(mean_times_matrix)
println("Overall Mean Time for Quaternion Rotation: ", overall_mean_quat, " seconds")
println("Overall Mean Time for Matrix Rotation: ", overall_mean_matrix, " seconds")
println("Rotation Matrices are ", 100*((overall_mean_matrix / overall_mean_quat)-1), " % slower than Quaternions.")

save("rotation_benchmark_plot.png", fig.scene, px_per_unit=10)  # Save the benchmark plot as an image
# save("rotation_benchmark_poster.png", fig.scene, px_per_unit=10)  # Save the benchmark plot as an image

