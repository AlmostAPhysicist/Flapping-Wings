using GLMakie: Scene, meshscatter!, cam3d!
using GLMakie

include("../src/Recorder.jl")

s = Figure(size=(500,500), camera=cam3d!, title="Recording Test")
axes = Axis3(s[1,1],  aspect = (1, 1, 1))
meshscatter!(axes, (0, 0, 0))

framerate = 30  # Frames per second
t = Observable(0.0)
recording = Observable(true)
io_ref = Ref{Any}(nothing)
filepath = joinpath(@__DIR__, "test.mp4")

# Print time on bottom left corner
text!(axes, (0,0,0), text=lift(t) do time
    "Time: $(round(time/framerate, digits=2)) s"
end, color=:black, fontsize=20, space=:pixel)

# Print t value on top right corner
text!(axes, (500, 500, 0), text=lift(t) do time
    "t: $(round(time, digits=2))"
end, color=:black, fontsize=20, space=:pixel, align=(:right, :top))

# Start recording
record_on_change_until(s.scene, t, recording, io_ref, filepath; framerate=framerate)



for i in 1:1000
    t[] += 1
    sleep(1/framerate)  # Simulate some work
    if !recording[]
        break
    end
end

# println("pwd() = ", pwd())
# println("@__DIR__ = ", @__DIR__)





