function get_user_input(prompt::String)
    print(prompt)
    flush(stdout)
    if isopen(stdin) && isinteractive()
        try
            return strip(readline())
        catch e
            if isa(e, InterruptException)
                println("\nInterrupted! Exiting...")
                exit(1)
            else
                rethrow(e)
            end
        end
    else
        println("Error: This script requires an interactive terminal. Run it in a terminal with 'julia readline.jl'.")
        exit(1)
    end
end

println("Welcome to the interactive input example!")

while true
    input = get_user_input("Enter something (or 'quit' to exit): ")
    println("You entered: $input")

    if input == "quit"
        println("Exiting...")
        break
    end
end