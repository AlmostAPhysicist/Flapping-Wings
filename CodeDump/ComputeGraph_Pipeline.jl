using GLMakie
# Create a scene
s = Scene(camera=cam3d!)

# Define the Observable for the sphere's center position
a = Observable(Point3{Float64}(0.0, 0.0, 0.0))

# Create the meshscatter plot with an initial static marker
plot = meshscatter!(s, Point3(0.0, 0.0, 0.0), marker=Sphere(Point3(0.0, 0.0, 0.0), 1.0), markersize=1, space=:data)

# Listen to the Observable and trigger Makie.update!
on(a) do val
    Makie.update!(plot, marker=Sphere(val, 1.0))
end

# Update the Observable to trigger the update
a[] = Point3(1.0, 1.0, 1.0)
notify(a)

# Display the scene
display(s)