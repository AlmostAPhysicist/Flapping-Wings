using GLMakie, FileIO, Dates

"""
    record_on_change_until(s::Scene, observable::Observable, until::Observable, ioref::Ref, filename::String; 
                          framerate::Real=30, shortcut::Bool=true)

Record a scene whenever an observable changes until a condition is met. Includes robust error handling
for IO errors and proper cleanup of resources.

# Arguments
- `s::Scene`: The scene to record
- `observable::Observable`: The observable to watch for changes
- `until::Observable`: Recording stops when this becomes false
- `ioref::Ref`: Reference to store the IO handle
- `filename::String`: Filename for the recording (timestamp will be added if no extension)
- `framerate::Real=30`: Frames per second for the recording
- `shortcut::Bool=true`: Whether to enable the 'r' key shortcut to stop recording
"""
function record_on_change_until(s::Scene, observable::Observable, until::Observable, ioref::Ref, filename::String; 
                               framerate::Real=30, shortcut::Bool=false)
    # Set the recording state to true
    until[] = true
    
    # Add timestamp to filename if no extension provided
    if !contains(filename, ".")
        timestamp = Dates.format(now(), "yyyy-mm-dd_HH-MM-SS")
        filename = "$(filename)_$(timestamp).mp4"
    end
    
    # Create a reference to track if recording is active
    is_recording = Ref(false)
    
    # Setup keyboard shortcut if enabled
    local kb_listener = nothing
    if shortcut
        kb_listener = on(events(s).keyboardbutton) do event
            if event.action == Keyboard.press && event.key == Keyboard.r
                until[] = false  # Stop recording when the shortcut key is pressed
                # @info "Recording stopped via keyboard shortcut"
            end
        end
    end

    @async begin
        try
            record(s, filename; framerate=Int(framerate)) do io
                ioref[] = io  # Store the io handle for use in the listener
                is_recording[] = true
                
                # Listener: record a frame only when observable changes and until is true
                observable_listener = on(observable) do new_frame
                    if until[] && ioref[] !== nothing && is_recording[]
                        try
                            recordframe!(ioref[])                        catch e
                            if isa(e, Base.IOError) || occursin("stream is closed", string(e))
                                @warn "Recording IO error: $(sprint(showerror, e))"
                                until[] = false  # Stop recording
                                is_recording[] = false
                            else
                                rethrow(e)
                            end
                        end
                    end
                end

                # Wait until recording is stopped or window is closed
                while isopen(s) && until[] && is_recording[]
                    yield()  # Yield to event loop, no need to sleep
                end

                # Clean up
                try
                    off(observable_listener)
                catch e
                    @warn "Error removing listener: $(sprint(showerror, e))"
                end
                
                # Clear references
                is_recording[] = false
                ioref[] = nothing
                
                # @info "Recording finished: $filename"
            end
        catch e
            @error "Recording error: $(sprint(showerror, e))"
            until[] = false
            ioref[] = nothing
            is_recording[] = false
        finally
            # Ensure keyboard listener is removed if it exists
            if kb_listener !== nothing
                try
                    off(kb_listener)
                catch e
                    @warn "Error removing keyboard listener: $(sprint(showerror, e))"
                end
            end
            
            # Force garbage collection to free resources
            GC.gc()
        end
    end
    
    return filename  # Return the filename for convenience
end

"""
    stop_recording!(until::Observable, ioref::Union{Ref, Nothing}=nothing)

Stop an active recording by setting the until observable to false.
Optionally can clear the IO reference to ensure proper cleanup.
"""
function stop_recording!(until::Observable, ioref::Union{Ref, Nothing}=nothing)
    until[] = false
    
    # If IO reference is provided, clear it
    if ioref !== nothing
        ioref[] = nothing
    end
    
    # Request garbage collection to free resources
    GC.gc()
    
    @info "Recording stopped programmatically"
end

