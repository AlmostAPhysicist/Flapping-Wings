using GLMakie, GeometryBasics, LinearAlgebra, Quaternions
import Quaternions: Quaternion as Quaternion

# Import custom code and utility functions
include("..src/State_and_Conversions.jl")
include("..src/Rendering.jl")
include("..src/Transformations.jl")
include("..src/WindowManager.jl")

# Initialize the custom structs and utilities
conversions = Conversions()
renderer = Renderer()
transformations = Transformations()
windowmanager = WindowManager()


# Create a scene and a reference basis 
scene1 = Scene(camera=cam3d!)
drawState!(scene1)
scene2 = Scene(camera=cam3d!)
drawState!(scene2)
windowmanager.closeall()

windowmanager.display(scene1, scene2; names=["Scene1", "Scene2"])

# Rotation of a vector 
v = [0.0, 0.0, 1.0]
meshscatter!(scene1, (0,0,0), marker=Arrow(Point{3, Float64}(v)).mesh, markersize=1, color=:black, label="v")

axis= [1.0, 1.0, 0.0] # Rotate around y-axis
angle = π / 3 # 90 degrees
rotation_quaternion = axisangle2quat(axis, angle)

v_rotated = imag_part(rotation_quaternion * Quaternion(0, v...) * inv(rotation_quaternion))
meshscatter!(scene1, (0,0,0), marker=Arrow(Point{3, Float64}(v_rotated)).mesh, markersize=1, color=:black)



# SLERP 

q1 = Quaternion(0, v...)
q2 = Quaternion(0, v_rotated...)

interpolated_states = Observable(q1)
interpolated_arrow = lift(q -> Arrow(Point{3, Float64}(imag_part(q))), interpolated_states)

axis_mesh = meshscatter!(scene2, (0,0,0), marker=Arrow(Point{3, Float64}(axis)).mesh, markersize=1, color=:yellow)
arrow_mesh = meshscatter!(scene2, (0,0,0), marker=lift(a -> a.mesh, interpolated_arrow), markersize=1, color=:black)

# Animate the SLERP
for t in range(0, 1, length=100)
    interpolated_states[] = norm(q1).*slerp(q1, q2, t)
    sleep(0.01) # Control the speed of the animation
end

# let q_star be such that q2 = q_star * q1 * inv(q_star)
# ⟹ q_star = sqrt(q2 * inv(q1))
# This means that q_star has the same axis as q2 * inv(q1) and the angle is half of the angle of q2 * inv(q1)

q_star = sqrt(q2 * inv(q1))

axis_angle = quat2axisangle(q_star)

axis ≈ axis_angle[1] # Check if the axis is the same
angle ≈ axis_angle[2] # Check if the angle is the same


# -----# Close all windows after the demo

windowmanager.closeall()

s = Scene(camera=cam3d!)
windowmanager.closeall()
windowmanager.display(s; names=["Quaternion Demo"])
drawState!(s)

v1 = normalize(rand(3))
v2 = normalize(rand(3))


q1 = Quaternion(0, v1...)
q2 = Quaternion(0, v2...)

interpolation = Observable(q1)
meshscatter!(s, (0,0,0), marker=lift(a -> Arrow(Point3(imag_part(a))).mesh, interpolation), markersize=1, color=:black)


# Axis and angle for the rotation
q_star = sqrt(q2 * inv(q1))
axis_angle = quat2axisangle(q_star)
meshscatter!(s, (0,0,0), marker=Arrow(Point3(axis_angle[1])).mesh, markersize=1, color=:yellow)

# Animate the interpolation
for t in range(0, 1, length=100)
    interpolation[] = slerp(q1, q2, t)
    sleep(0.01) # Control the speed of the animation
end

# SLERP would thus be equivalent to applying rotating about axis_angle[1] by an angle axis_angle[2]/n for n steps.