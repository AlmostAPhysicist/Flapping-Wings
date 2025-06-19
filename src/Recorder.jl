using GLMakie, FileIO



function record_on_change_until(s::Scene, observable::Observable, until::Observable, ioref::Ref, filename::String; framerate::Real=30, shortcut::Bool=true)
    # Set the recording state to true
    until[] = true

    if shortcut
        on(events(s).keyboardbutton) do event
            if event.action == Keyboard.press && event.key == Keyboard.r
                until[] = false  # Stop recording when the shortcut key is pressed
            end
        end
    end

    @async begin
        record(s, filename; framerate=Int(framerate)) do io
            ioref[] = io  # Store the io handle for use in the listener

            # Listener: record a frame only when observable changes and until is true
            observable_listener = on(observable) do new_frame
                if until[] && ioref[] !== nothing
                    recordframe!(ioref[])
                end
            end

            # Wait until recording is stopped or window is closed
            while isopen(s) && until[]
                yield()  # Yield to event loop, no need to sleep
            end

            # Clean up
            off(observable_listener)
            ioref[] = nothing
        end
    end
end

