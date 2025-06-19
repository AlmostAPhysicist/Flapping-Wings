# Import necessary packages
using GLMakie, GeometryBasics, LinearAlgebra, Quaternions, FileIO
import Quaternions: Quaternion as Quaternion

cd(@__DIR__) # Changing the current working directory from project root to the parent directory of this file so that pwd() and @__DIR__ match.
# Import custom code and utility functions
include("../src/State_and_Conversions.jl")
include("../src/Rendering.jl")
include("../src/Transformations.jl")
include("../src/WindowManager.jl")
include("../src/Recorder.jl")



# Initialize the custom structs and utilities
conversions = Conversions()
renderer = Renderer()
transformations = Transformations()
windowmanager = WindowManager()

# Defining Custom Mesh Transformation functions
function translate_obj(obj::AbstractMesh, translation::AbstractVector)
    obj_copy = deepcopy(obj)  # Create a copy of the object to avoid modifying the original
    # Translate positions
    for i in eachindex(obj_copy.vertex_attributes[1])
        obj_copy.vertex_attributes[1][i] += translation
    end

    # Translate normals if they exist
    if length(obj_copy.vertex_attributes) >= 3
        for i in eachindex(obj_copy.vertex_attributes[3])
            obj_copy.vertex_attributes[3][i] += translation
        end
    end

    return obj_copy
end

function rotate_obj(obj::AbstractMesh, R::AbstractMatrix; about::Union{AbstractVector,Nothing}=nothing)
    obj_copy = deepcopy(obj)  # Create a copy of the object to avoid modifying the original
    # First translate the object to the origin
    if isnothing(about)
        # If no specific point is given, just rotate the object
        # Rotate positions
        for i in eachindex(obj.vertex_attributes[1])
            obj_copy.vertex_attributes[1][i] = Point3f(R * Vec3f(obj_copy.vertex_attributes[1][i]))
        end

        # Rotate normals if they exist
        if length(obj_copy.vertex_attributes) >= 3
            for i in eachindex(obj_copy.vertex_attributes[3])
                obj_copy.vertex_attributes[3][i] = normalize(Point3f(R * Vec3f(obj_copy.vertex_attributes[3][i])))
            end
        end

        return obj_copy
    else
        # If a specific point is given, translate the object to that point, rotate it, and then translate it back
        translation = -about
        obj_copy = translate_obj(obj_copy, translation)  # Translate to origin
        obj_copy = rotate_obj(obj_copy, R)  # Rotate around the origin
        obj_copy = translate_obj(obj_copy, -translation)  # Translate back to the original position

        return obj_copy
    end
end

# Load the shaders and models for the DragonFly
begin #Load _models
    shader = load("../Models/DragonFly/HighQuality/body.png") # Set to true for using shaders, false for using the default renderer

    _body_model = GeometryBasics.expand_faceviews(load("../Models/DragonFly/HighQuality/body.obj").mesh)
    _frontleftwing_model = GeometryBasics.expand_faceviews(load("../Models/DragonFly/HighQuality/frontleftwing.obj").mesh)
    _frontrightwing_model = GeometryBasics.expand_faceviews(load("../Models/DragonFly/HighQuality/frontrightwing.obj").mesh)
    _hindleftwing_model = GeometryBasics.expand_faceviews(load("../Models/DragonFly/HighQuality/hindleftwing.obj").mesh)
    _hindrightwing_model = GeometryBasics.expand_faceviews(load("../Models/DragonFly/HighQuality/hindrightwing.obj").mesh)
end

# Define the wing positions relative to the body position (manually determined in BLENDER)
begin # Define Positions
    body_pos = Observable(Point3(0.0, 0.0, 0.0)) # Position of the body
    frontleftwing_pos = lift(body_pos -> body_pos .+ Point3(-0.4192, -0.1287, 1.342), body_pos) # Position of the front left wing
    frontrightwing_pos = lift(body_pos -> body_pos .+ Point3(-0.4192, 0.1287, 1.342), body_pos) # Position of the front right wing
    hindleftwing_pos = lift(body_pos -> body_pos .+ Point3(-0.2366, -0.1728, 1.184), body_pos) # Position of the hind left wing
    hindrightwing_pos = lift(body_pos -> body_pos .+ Point3(-0.2366, 0.1728, 1.184), body_pos) # Position of the hind right wing
end

# Transform the models to align them correctly
begin # Rotate _models
    _body_model = rotate_obj(_body_model, conversions.axisangle2rotmatrix([1, 0, 0], π / 2))
    _frontleftwing_model = rotate_obj(_frontleftwing_model, conversions.axisangle2rotmatrix([1, 0, 0], π / 2))
    _frontrightwing_model = rotate_obj(_frontrightwing_model, conversions.axisangle2rotmatrix([1, 0, 0], π / 2))
    _hindleftwing_model = rotate_obj(_hindleftwing_model, conversions.axisangle2rotmatrix([1, 0, 0], π / 2))
    _hindrightwing_model = rotate_obj(_hindrightwing_model, conversions.axisangle2rotmatrix([1, 0, 0], π / 2))
end

begin # Translate _models
    _body_model = translate_obj(_body_model, -body_pos[]) # Translate the body model to the position of the body
    _frontleftwing_model = translate_obj(_frontleftwing_model, -frontleftwing_pos[]) # Translate the front left wing model to the position of the front left wing
    _frontrightwing_model = translate_obj(_frontrightwing_model, -frontrightwing_pos[]) # Translate the front right wing model to the position of the front right wing
    _hindleftwing_model = translate_obj(_hindleftwing_model, -hindleftwing_pos[]) # Translate the hind left wing model to the position of the hind left wing
    _hindrightwing_model = translate_obj(_hindrightwing_model, -hindrightwing_pos[]) # Translate the hind right wing model to the position of the hind right wing
end

function attach2State(obj::AbstractMesh, state_obs::Observable{State})
    return lift(st -> rotate_obj(obj, conversions.quat2rotmatrix(st.q)), state_obs)
end

begin # Define State Observables
    body = Observable(State()) # Create an observable state for the body
    frontleftwing = Observable(State()) # Create an observable state for the front left wing
    frontrightwing = Observable(State()) # Create an observable state for the front right wing
    hindleftwing = Observable(State()) # Create an observable state for the hind left wing
    hindrightwing = Observable(State()) # Create an observable state for the hind right wing
end
# Reassign the mesh observables for the body and wings to transform their positions
# transform the vertices of the models using the rotation matrix defined by the element state

begin # Attach the models to the state observables with fixed _models
    body_model = attach2State(_body_model, body)
    frontleftwing_model = attach2State(_frontleftwing_model, frontleftwing)
    frontrightwing_model = attach2State(_frontrightwing_model, frontrightwing)
    hindleftwing_model = attach2State(_hindleftwing_model, hindleftwing)
    hindrightwing_model = attach2State(_hindrightwing_model, hindrightwing)
end


# --- The Controls and Data Plots ---
#------------------------------------
function map_range(val, from, to)
    # Works for any order of from and to ranges (increasing or decreasing)
    normalized = (val - from[1]) / (from[2] - from[1])
    return to[1] + normalized * (to[2] - to[1])
end

# Defining the initial parameters for the rotation
# ---------------

# Define observables
dt = Observable(0.01)

# Global rotation around the x-axis
phi_x_global     = Observable(deg2rad(0))
omega_x_global   = Observable(12)
theta_min_x_global = Observable(deg2rad(0))
theta_max_x_global = Observable(deg2rad(45))

# Local rotation around the y-axis
phi_y_local      = Observable(deg2rad(10))
omega_y_local    = Observable(12)
theta_min_y_local = Observable(deg2rad(-40))
theta_max_y_local = Observable(deg2rad(40))

# Environment variables
phase_between_front_and_hind = Observable(Float64(pi))           # Phase difference (constant for now)
sync_omegas = Observable(true)                   # Toggle for synchronization
pause     = Observable(false)                    # New toggle for pause


# Map -1,1 to theta_min, theta_max
theta_x_global(t) = map_range(cos(omega_x_global[] * t + phi_x_global[]), (-1, 1), (theta_min_x_global[], theta_max_x_global[]))
angular_velocity_x_global(t) = -omega_x_global[] * (theta_max_x_global[] - theta_min_x_global[]) / 2 * sin(omega_x_global[] * t + phi_x_global[]) # Analytical derivative of theta_x_global

theta_y_local(t) = map_range(sin(omega_y_local[] * t + phi_y_local[]), (-1, 1), (theta_min_y_local[], theta_max_y_local[]))
angular_velocity_y_local(t) = omega_y_local[] * (theta_max_y_local[] - theta_min_y_local[]) / 2 * cos(omega_y_local[] * t + phi_y_local[]) # Analytical derivative of theta_y_local


n = rand()
theta_x_global(n)
theta_x_global(n + pi/omega_x_global[])

initial_values = [
    omega_x_global[], phi_x_global[], theta_min_x_global[], theta_max_x_global[],
    omega_y_local[], phi_y_local[], theta_min_y_local[], theta_max_y_local[],
    dt[], phase_between_front_and_hind[], sync_omegas[], pause[]
]



# Create a main figure
begin # Define Controls figure with plots and controls
    Makie.set_theme!(theme_ggplot2())
    Makie.set_theme!(theme_latexfonts())
    controls = Figure(size = (600,600), fontsize = 10)

    # Create two GridLayouts: one for plots and one for controls
    gl_main = GridLayout(controls[1, 1], tellwidth = false)

    # 1. GridLayout for plots
    gl_plots = GridLayout(gl_main[1, 1], tellwidth = false)
    ax_thetas = Axis(gl_plots[1, 1], title="Rotational Angle", xlabel="Time (s)", ylabel="Angle (rad)")
    ax_angular_velocities = Axis(gl_plots[1, 2], title="Angular Velocity", xlabel="Time (s)", ylabel="Angular Velocity (rad/s)")

    # 2. GridLayout for controls
    gl_controls = GridLayout(gl_main[2, 1], tellwidth = false)

    # 2.1 SliderGrid for Global X-axis rotation parameters
    sg_global = SliderGrid(
        gl_controls[1, 1],
        (label = L"Omega X Global ($\omega_{x_\text{G}}$)", range = 0:1:100, format = "{:.0f} Hz", startvalue = omega_x_global[], width = Relative(1)),
        (label = L"Phi X Global ($\phi_{x_\text{G}}$)", range = -π:0.1:π, format = "{:.2f} rad", startvalue = phi_x_global[], width = Relative(1)),
        (label = L"Theta Min Global ($\theta_{x_\text{G}}$)", range = -π:0.1:π, format = "{:.2f} rad", startvalue = theta_min_x_global[], width = Relative(1)),
        (label = L"Theta Max Global ($\theta_{x_\text{G}}$)", range = -π:0.1:π, format = "{:.2f} rad", startvalue = theta_max_x_global[], width = Relative(1)),
        height = Fixed(80),  # Compact height
        tellheight = false   # Prevent influencing row height
    )

    # 2.2 SliderGrid for Local Y-axis rotation parameters
    sg_local = SliderGrid(
        gl_controls[1, 2],
        (label = L"Omega Y Local ($\omega_{y_\text{L}}$)", range = 0:1:100, format = "{:.0f} Hz", startvalue = omega_y_local[], width = Relative(1)),
        (label = L"Phi Y Local ($\phi_{y_\text{L}}$)", range = -π:0.1:π, format = "{:.2f} rad", startvalue = phi_y_local[], width = Relative(1)),
        (label = L"Theta Min Y Local ($\theta_{y_\text{L}}$)", range = -π:0.1:π, format = "{:.2f} rad", startvalue = theta_min_y_local[], width = Relative(1)),
        (label = L"Theta Max Y Local ($\theta_{y_\text{L}}$)", range = -π:0.1:π, format = "{:.2f} rad", startvalue = theta_max_y_local[], width = Relative(1)),
        height = Fixed(80),  # Compact height
        tellheight = false   # Prevent influencing row height
    )

    # 3. GridLayout for Environment variables
    sg_env_var = GridLayout(gl_controls[2, 1:2], tellwidth = false, width = Relative(1))

    # 3.1 SliderGrid for Environment variables (numeric)
    # sg_env_num = SliderGrid(
    #     sg_env_var[1, 2],
    #     (label = "dt", range = 0.001:0.001:0.1, format = "{:.3f} s", startvalue = dt[]),
    #     (label = "Phase Front-Hind", range = 0:0.01:2π, format = "{:.2f} rad", startvalue = phase_between_front_and_hind[]),
    #     height = Fixed(40),  # Smaller height for 2 sliders
    #     tellheight = false,   # Prevent influencing row height
    #     # width = Fixed(400),    # Fixed width for the slider grid
    #     tellwidth = false      # Prevent influencing column width
    # )

    dt_slider = SliderGrid(sg_env_var[1, 1], (label = "dt", range = 0.001:0.001:0.1, format = "{:.3f} s", startvalue = dt[]), width = Relative(1), tellwidth = false)
    phase_front_hind_slider = SliderGrid(sg_env_var[1, 2], (label = "Phase Front-Hind", range = 0:0.01:2π, format = "{:.2f} rad", startvalue = phase_between_front_and_hind[]), width = Relative(1), tellwidth = false)

    # 3.2 Toggles for Environment variables (boolean) using nested GridLayouts
    # Sync Omegas toggle
    gl_sync = GridLayout(sg_env_var[2, 1], tellwidth = false, width = Relative(2))
    sync_label = Label(gl_sync[1, 1], L"Sync Omegas ($\omega_{x_\text{G}} = \omega_{y_\text{L}}$)")
    sync_toggle = Toggle(gl_sync[1, 2], active = sync_omegas[])

    # Pause toggle
    gl_pause = GridLayout(sg_env_var[2, 2], tellwidth = false, width = Relative(2))
    pause_label = Label(gl_pause[1, 1], "Pause")
    pause_toggle = Toggle(gl_pause[1, 2], active = pause[])

    # 3.3 Reset button for environment variables as well as the save plots button
    save_plots_button = Button(sg_env_var[3, 1], label="Save Plots", tellwidth = false)
    record_button = Button(sg_env_var[3, 2], label="Record", tellwidth = false)
    reset_button = Button(sg_env_var[4, 1:2], label="Reset Environment Variables", tellwidth = false)


    # Adjust row sizes to prioritize plot height and improve spacing
    rowsize!(gl_main, 1, Relative(0.5))  # Plots take 50% of height
    rowgap!(gl_main, Fixed(10))  # Spacing between plots and controls
    rowgap!(gl_controls, Fixed(1))  # Spacing between control rows
    # rowgap!(sg_env_var, 1, Fixed(1))
    # rowgap!(sg_env_var, Fixed(1))  # Tight spacing within environment variables
    rowgap!(gl_controls, 1, Fixed(20))
    colgap!(gl_sync, Fixed(5))  # Space between label and toggle
    colgap!(gl_pause, Fixed(5)) # Space between label and toggle

    display(controls)
end

begin # Define Updater Functions
    # Update Global X-axis rotation parameters
    omega_x_global_updater = on(sg_global.sliders[1].value) do slider_val
        omega_x_global[] = slider_val
    end
    phi_x_global_updater = on(sg_global.sliders[2].value) do slider_val
        phi_x_global[] = slider_val
    end
    theta_min_x_global_updater = on(sg_global.sliders[3].value) do slider_val
        theta_min_x_global[] = slider_val
    end
    theta_max_x_global_updater = on(sg_global.sliders[4].value) do slider_val
        theta_max_x_global[] = slider_val
    end

    # Update Local Y-axis rotation parameters
    omega_y_local_updater = on(sg_local.sliders[1].value) do slider_val
        omega_y_local[] = slider_val
    end
    phi_y_local_updater = on(sg_local.sliders[2].value) do slider_val
        phi_y_local[] = slider_val
    end
    theta_min_y_local_updater = on(sg_local.sliders[3].value) do slider_val
        theta_min_y_local[] = slider_val
    end
    theta_max_y_local_updater = on(sg_local.sliders[4].value) do slider_val
        theta_max_y_local[] = slider_val
    end

    # Update Environment variables
    dt_updater = on(dt_slider.sliders[1].value) do slider_val
        dt[] = slider_val
    end
    phase_front_hind_updater = on(phase_front_hind_slider.sliders[1].value) do slider_val
        phase_between_front_and_hind[] = slider_val
    end

    # Sync Omegas toggle updater
    update_omega_x_global_to_omega_y_local = on(omega_y_local) do val
        if sync_omegas[]
            if val != omega_x_global[]
                sg_global.sliders[1].value[] = val  # Update the slider value to reflect the change, omega_x_global will be updated automatically due to omega_x_global_updater
                set_close_to!(sg_global.sliders[1], val)  # Ensure the slider reflects the new value
            end
        end
    end
    update_omega_y_local_to_omega_x_global = on(omega_x_global) do val
        if sync_omegas[]
            if val != omega_y_local[]
                sg_local.sliders[1].value[] = val  # Update the slider value to reflect the change, omega_y_local will be updated automatically due to omega_y_local_updater
                set_close_to!(sg_local.sliders[1], val)  # Ensure the slider reflects the new value
            end
        end
    end
    sync_omegas_updater = on(sync_toggle.active) do active
        sync_omegas[] = active
        if active
            println("Synchronization of omegas enabled.")
        else
            println("Synchronization of omegas disabled.")
        end
    end
    # Pause toggle updater
    pause_updater = on(pause_toggle.active) do active
        pause[] = active
        if active
            println("Simulation paused.")
        else
            println("Simulation resumed.")
        end
    end

    # Reset button updater
    reset_button_updater = on(reset_button.clicks) do _event
        # Reset all observables to their initial values
        omega_x_global[] = initial_values[1]
        phi_x_global[] = initial_values[2]
        theta_min_x_global[] = initial_values[3]
        theta_max_x_global[] = initial_values[4]

        omega_y_local[] = initial_values[5]
        phi_y_local[] = initial_values[6]
        theta_min_y_local[] = initial_values[7]
        theta_max_y_local[] = initial_values[8]

        dt[] = initial_values[9]
        phase_between_front_and_hind[] = initial_values[10]

        sync_omegas[] = initial_values[11]
        # pause[] = initial_values[12]

        # set_close_to! function would ensure the sliders reflect the reset values
        set_close_to!(sg_global.sliders[1], omega_x_global[])
        set_close_to!(sg_global.sliders[2], phi_x_global[])
        set_close_to!(sg_global.sliders[3], theta_min_x_global[])
        set_close_to!(sg_global.sliders[4], theta_max_x_global[])
        set_close_to!(sg_local.sliders[1], omega_y_local[])
        set_close_to!(sg_local.sliders[2], phi_y_local[])
        set_close_to!(sg_local.sliders[3], theta_min_y_local[])
        set_close_to!(sg_local.sliders[4], theta_max_y_local[])
        set_close_to!(sg_env_num.sliders[1], dt[])
        set_close_to!(sg_env_num.sliders[2], phase_between_front_and_hind[])
        # set_close_to!(sync_toggle, sync_omegas[])

        println("Environment variables reset to default values.")
    end

    using Dates
    # Save plots button updater
    save_plots_button_updater = on(save_plots_button.clicks) do _event
        # Save the current plots
        # Append the current time to the filenames to avoid overwriting
        timestamp = Dates.format(now(), "yyyy-mm-dd_HH-MM-SS")
        # Create directories with Plots_timestamp
        path = joinpath(@__DIR__, "..", "Plots/plots_$timestamp")
        mkpath(path)
        # Save the plots in the created directory
        save("$path/rotational_angle_plot.png", colorbuffer(ax_thetas))
        save("$path/angular_velocity_plot.png", colorbuffer(ax_angular_velocities))
        println("Plots saved in $path/")
        save("$path/controls_figure.png", controls)
    end

    # Record animation button updater
    # Use the record_on_change_until function within Recorder.jl to record the animation

    # Read the record_button.clicks. Set record_button.label to "Recording ($record_button.clicks[]==1 ? "Off" : "On"$)" to indicate whether recording is active or not.
    # Whenever the recording has been set active and then back to inactive, reset recording_button.clicks to 0.

    # Initialize the recording label as Off
    global recording = Observable(false) # Observable to track recording state
    record_button.label = "Record (Off)"
    record_button_updater = on(record_button.clicks) do _event
        # Toggle the recording state
        if record_button.clicks[]%2 == 1
            record_button.label = "Record (on)"

            global ioref1 = Ref{Any}(nothing) # Reference to the IO object for recording
            global ioref2 = Ref{Any}(nothing) # Reference to the IO object for recording

            # Save the recordings under Media/Videos with the current timestamp
            timestamp = Dates.format(now(), "yyyy-mm-dd_HH-MM-SS")
            folder = joinpath(@__DIR__, "../Media/Videos", "Dragonfly_$timestamp")
            mkpath(folder)  # Create the folder if it doesn't exist
            filename1 = joinpath(folder, "controls.mp4")
            filename2 = joinpath(folder, "animation.mp4")

            record_on_change_until(controls.scene, t, recording, ioref1, filename1; framerate=1/dt[])
            record_on_change_until(s_dragonfly, t, recording, ioref2, filename2; framerate=1/dt[])
            println("Recording started. Press the button again (or press 'r') to stop recording.")
        elseif record_button.clicks[] != 0

            record_button.label = "Record (Off)"
            recording[] = false  # Stop the recording
            # Stop the recording
            # global ioref1 = nothing  # Stop the recording for the controls figure
            # global ioref2 = nothing  # Stop the recording for the scene
            record_button.clicks[] = 0  # Reset the clicks to 0 after stopping the recording

            println("Recording stopped. Videos saved in Media/Videos/...")
        end
    end
end

# Plot the global and local theta and angular velocity on the same plot with lift functions to automatically update the plot
times = lift(dt->0:dt:5*2π/initial_values[1], dt) # Time vector for the simulation

thetas_x_front = lift((t, omega, phi, tmin, tmax, phase_between_front_and_hind) -> theta_x_global.(t), times, omega_x_global, phi_x_global, theta_min_x_global, theta_max_x_global, phase_between_front_and_hind)
thetas_x_back = lift((t, omega, phi, tmin, tmax, phase_between_front_and_hind) -> [theta_x_global(i + phase_between_front_and_hind/omega) for i in t], times, omega_x_global, phi_x_global, theta_min_x_global, theta_max_x_global, phase_between_front_and_hind)

angular_velocities_x_front = lift((t, omega, phi, tmin, tmax, phase_between_front_and_hind) -> angular_velocity_x_global.(t), times, omega_x_global, phi_x_global, theta_min_x_global, theta_max_x_global, phase_between_front_and_hind)
angular_velocities_x_back = lift((t, omega, phi, tmin, tmax, phase_between_front_and_hind) -> [angular_velocity_x_global(i + phase_between_front_and_hind/omega) for i in t], times, omega_x_global, phi_x_global, theta_min_x_global, theta_max_x_global, phase_between_front_and_hind)

thetas_y_front = lift((t, omega, phi, tmin, tmax, phase_between_front_and_hind) -> theta_y_local.(t), times, omega_y_local, phi_y_local, theta_min_y_local, theta_max_y_local, phase_between_front_and_hind)
thetas_y_back = lift((t, omega, phi, tmin, tmax, phase_between_front_and_hind) -> [theta_y_local(i + phase_between_front_and_hind/omega) for i in t], times, omega_y_local, phi_y_local, theta_min_y_local, theta_max_y_local, phase_between_front_and_hind)

angular_velocities_y_front = lift((t, omega, phi, tmin, tmax, phase_between_front_and_hind) -> angular_velocity_y_local.(t), times, omega_y_local, phi_y_local, theta_min_y_local, theta_max_y_local, phase_between_front_and_hind)
angular_velocities_y_back = lift((t, omega, phi, tmin, tmax, phase_between_front_and_hind) -> [angular_velocity_y_local(i + phase_between_front_and_hind/omega) for i in t], times, omega_y_local, phi_y_local, theta_min_y_local, theta_max_y_local, phase_between_front_and_hind)

# Update the plots with the calculated theta values
lines!(ax_thetas, times, thetas_x_front, label="Theta X Front Global", color=:red)
lines!(ax_thetas, times, thetas_x_back, label="Theta X Back Global", color=:orange)

lines!(ax_thetas, times, thetas_y_front, label="Theta Y Front Local", color=:blue)
lines!(ax_thetas, times, thetas_y_back, label="Theta Y Back Local", color=(:blue, 0.5))

lines!(ax_angular_velocities, times, angular_velocities_x_front, label="Angular Velocity X Front Global", color=:red)
lines!(ax_angular_velocities, times, angular_velocities_x_back, label="Angular Velocity X Back Global", color=(:red, 0.5))

lines!(ax_angular_velocities, times, angular_velocities_y_front, label="Angular Velocity Y Front Local", color=:blue)
lines!(ax_angular_velocities, times, angular_velocities_y_back, label="Angular Velocity Y Back Local", color=(:blue, 0.5))

# Add a legend to the plots
axislegend(ax_thetas, labelsize=10, patchsize=(3,2), padding=2, colgap=1, rowgap=1, patchlabelgap=1)
axislegend(ax_angular_velocities, labelsize=10, patchsize=(3,2), padding=2, colgap=1, rowgap=1, patchlabelgap=1)




# Create a scene for the DragonFly
set_theme!(theme_dark()) # Set the theme for the scene
s_dragonfly = Scene(camera=cam3d!, size=(500, 500))
# drawState!(s_dragonfly)
body_mesh = meshscatter!(s_dragonfly, body_pos, marker=body_model, markersize=1, color=shader) # Add the body position to the scene
frontleftwing_mesh = meshscatter!(s_dragonfly, frontleftwing_pos, marker=frontleftwing_model, markersize=1, color=shader)
frontrightwing_mesh = meshscatter!(s_dragonfly, frontrightwing_pos, marker=frontrightwing_model, markersize=1, color=shader)
hindleftwing_mesh = meshscatter!(s_dragonfly, hindleftwing_pos, marker=hindleftwing_model, markersize=1, color=shader)
hindrightwing_mesh = meshscatter!(s_dragonfly, hindrightwing_pos, marker=hindrightwing_model, markersize=1, color=shader)

# Optional Repositioning of the DragonFly
body_pos[] = Point3(0.25, 0.0, -1.0)


wingstates = [frontleftwing, frontrightwing, hindleftwing, hindrightwing]
for i in wingstates
    i[] = State()  # Reset the wing states to the initial state
end
reflect(q::Quaternion, normal::AbstractVector) = Quaternion(0, normal...) * q * Quaternion(0, normal...)

windowmanager.closeall()
windowmanager.display(controls, s_dragonfly; names=["Controls", "Scene"])  # Display the controls and the scene



t = Observable(0.0)
running = Observable(true)  # Flag to control the simulation loop



# Function to update the wings based on the current time

function update_wings(t)
    frontrightwing[] = State(conversions.axisangle2quat([1, 0, 0], -theta_x_global(t[])))
    hindrightwing[] = State(conversions.axisangle2quat([1, 0, 0], -theta_x_global(t[]+phase_between_front_and_hind[]/omega_x_global[])))
    
    # Apply local rotation around y-axis
    frontrightwing[] = State(conversions.coordinate_transform(conversions.axisangle2quat([0, 1, 0], theta_y_local(t[])), frontrightwing[].q)*frontrightwing[].q)
    hindrightwing[] = State(conversions.coordinate_transform(conversions.axisangle2quat([0, 1, 0], theta_y_local(t[]+phase_between_front_and_hind[]/omega_y_local[])), hindrightwing[].q)*hindrightwing[].q)

    # Reflect the left wings across the XZ plane
    frontleftwing[] = State(reflect(frontrightwing[].q, [0, 1, 0]))  # Reflect the quaternion across the XZ plane
    hindleftwing[] = State(reflect(hindrightwing[].q, [0, 1, 0]))  # Reflect the quaternion across the XZ plane
end

update_wings(t)  # Initial update to set the initial state of the wings


while isopen(controls.scene) && isopen(s_dragonfly) && running[]  # Main loop to keep the simulation running

    if !pause[]  # Check if the simulation is paused
        

        t[] = t[] + dt[]  # Update time observable

        # Update the states based on the angular velocities
        update_wings(t)
        sleep(dt[])  # Yield to allow the GUI to update
    else
        sleep(dt[])
    end
end
