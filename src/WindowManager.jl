#Imports
using GLMakie: Screen, activate!, display, closeall
using GLMakie.Makie: inline!


### helper function

"""
    display_windows(scenes...; names::Vector{String}=String[])

Display each given scene in a separate GLMakie window.

# Arguments
- `scenes...`: One or more Makie scenes to display.
- `names`: (Optional) A vector of window titles. If provided and its length matches the number of scenes, each window will be titled accordingly. Otherwise, windows will be titled "Window 1", "Window 2", etc.

# Example
```julia
display_windows(scene1, scene2; names=["Main View", "Side View"])
```
"""
function display_windows(scenes...; names::Vector{String}=String[])
    # Create a new window for each scene
    if length(scenes) == length(names)
        for (scene, name) in zip(scenes, names)
            activate!(title=name)
            win = Screen()
            display(win, scene)
        end
    else
        for (i, scene) in enumerate(scenes)
            activate!(title="Window $i")
            win = Screen()
            display(win, scene)
        end
    end
end

"""
    struct WindowManager

A utility struct for managing multiple GLMakie windows in Julia.

# Fields
- `display`: Display one or more Makie scenes in separate windows, with optional custom window titles.
- `closeall`: Close all open GLMakie windows.
- `inline!`: Control inline plotting in environments like Jupyter.

# Example
```julia
windowmanager = WindowManager()
windowmanager.display(scene1, scene2; names=["Main", "Side"])
windowmanager.inline!()
windowmanager.closeall()
```

# Documentation Notes
For more details on how to use the window manager, see the documentation for each field in the `WindowManager` struct.  
You can access these with the `?` operator in Julia, for example:

```julia
julia> 
help?> WindowManager.display
  display(scenes...; names::Vector{String}=String[])

  Display each given scene in a separate GLMakie window.

  Arguments
  ≡≡≡≡≡≡≡≡≡

    •  scenes...: One or more Makie scenes to display.

    •  names: (Optional) A vector of window titles. If provided and its length matches the number of scenes, each window will be titled
       accordingly. Otherwise, windows will be titled "Window 1", "Window 2", etc.

  Example
  ≡≡≡≡≡≡≡

  windowmanager.display(scene1, scene2; names=["Main View", "Side View"])
```
---
"""
struct WindowManager
    """
        display(scenes...; names::Vector{String}=String[])

    Display each given scene in a separate GLMakie window.

    # Arguments
    - `scenes...`: One or more Makie scenes to display.
    - `names`: (Optional) A vector of window titles. If provided and its length matches the number of scenes, each window will be titled accordingly. Otherwise, windows will be titled "Window 1", "Window 2", etc.

    # Example
    ```julia
    windowmanager.display(scene1, scene2; names=["Main View", "Side View"])
    ```
    """
    display

    """
        closeall()

    Close all open GLMakie windows.

    # Example
    ```julia
    windowmanager.closeall()
    ```
    """
    closeall

    """
        inline!()

    Control inline plotting in environments like Jupyter.

    # Example
    ```julia
    windowmanager.inline!()
    ```
    """
    inline!

    function WindowManager()
        new(display_windows, closeall, inline!)
    end
end

# Usage:
# windowmanager = WindowManager()
# windowmanager.display(scene1, scene2; names=["Main", "Side"])
# windowmanager.inline!()
# windowmanager.closeall()