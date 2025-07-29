using BenchmarkTools

func() = rand(3,3) .* rand(3,3) # Example function to benchmark

b = @benchmarkable func()

t = run(b)

t
fieldnames(typeof(t))


t.params
minimum(t.times)
maximum(t.times)
median(t.times)
mean(t.times)
std(t.times)



using GLMakie, BenchmarkTools


function exponential_spacing(min_val::Real, max_val::Real, resolution::Int; base::Real=ℯ)
    if resolution == 1
        return [(min_val + max_val) / 2]
    end
    if base < 0
        throw(ArgumentError("Base must be greater than 0 for exponential spacing."))
    end
    if base == 1
        return range(min_val, max_val, length=resolution) # Linear spacing if base is 1
    end
    
    # Create linear range from min to max
    linear_points = range(min_val, max_val, length=resolution)
    
    # Define casewise exponential function with custom base
    function f(x)
        if x > 0
            # For positive values: exponential decay towards zero
            # Maps [0, max] to [0, max] with more points near zero
            return max_val * (base^(x/max_val) - 1) / (base - 1)
        elseif x < 0
            # For negative values: exponential decay towards zero
            # Maps [min, 0] to [min, 0] with more points near zero
            return min_val * (base^(-x/(-min_val)) - 1) / (base - 1)
        else
            # x == 0
            return 0.0
        end
    end
    
    # Apply the exponential transformation
    exponential_points = [f(x) for x in linear_points]
    
    return exponential_points
end

function plot_benchmark(func::Function, samples, title="Benchmark Plot", xlabel="Sample Index", ylabel="Execution Time (s)")
    b = @benchmarkable func() samples=samples
    t = run(b)

    t_min, t_max, t_median, t_mean, t_std = minimum(t.times), maximum(t.times), median(t.times), mean(t.times), std(t.times)/ sqrt(length(t.times))


    fig = Figure(title=title)
    ax = Axis(fig[1, 1], xlabel=xlabel, ylabel=ylabel)
    # xlims!(ax, 0, samples)
    # ylims!(ax, 0, maximum(t.times) * 1.1) # Set y-limits to 10% above max time

    # # Make a bar plot of the execution times
    # # Overlay on the bar plot with a line for the mean along with error bars for std deviation
    # # Overlay on the bar plot with a line for the median

    # hist!(ax, t.times, bins=50, color=:blue, label="Execution Times")

    # vlines!(ax, [t_mean], color=:red, label="Mean", linewidth=2)
    # vlines!(ax, [t_median], color=:green, label="Median", linewidth=2)
    # vlines!(ax, [t_mean - t_std, t_mean + t_std], color=:orange, label="Std Dev", linewidth=2, linestyle=:dash)


    # Bin the execution times into a histogram with a custom function 
    # The Bin containing the Mean should be highlighted as green 
    # The Bin containing the Median should be highlighted as blue
    # The Bins generated should be spaced exponentially around the median 
    bins = exponential_spacing(t_min, t_max, 50, base=10.0)

    hist!(ax, t.times, bins=bins, color=:black, label="Execution Times")

    vlines!(ax, [t_mean], color=:green, label="Mean", linewidth=2)
    vlines!(ax, [t_median], color=:blue, label="Median", linewidth=2)
    vlines!(ax, [t_mean - t_std, t_mean + t_std], color=:lightgreen, label="Std Dev", linewidth=2, linestyle=:dash)
    return fig
end

plot_benchmark(func, 10000)


@benchmark func() samples=1000