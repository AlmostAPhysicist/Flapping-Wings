"""UNDERSTANDING FILE PATHS IN JULIA"""
# https://discourse.julialang.org/t/the-mismatch-between-pwd-and-dir-and-how-it-impacts-packages-like-glmakie/130036

# Here's a good discourse thread on the topic that I had started. I think it is a good way to understand the way file systems work in Julia.


# File Paths 
println("Current File (accessed with @__FILE__): \n\t", @__FILE__)
println("Current file's parent directory (accessed with @__DIR__): \n\t", @__DIR__)
println("@__DIR__ is equivalent to dirname(@__FILE__). dirname() can be used to get the parent directory of a file.\n")

println("Current working directory (accessed with pwd()): \n\t", pwd())
println("The current working directory is the directory from which Julia REPL was started, and it can be changed with `cd()`. By default, in VSCode, it is the root of the project (WHICH MAY NOT MATCH @__DIR__).")

path = "../Models/DragonFly/HighQuality/body.png" # This is a relative path to the body.png file in the Models directory.
joinedpath = joinpath(@__DIR__, path) # This is the path to the body.png file relative to the current file's directory.

using FileIO # FileIO is a package that provides functions for loading and saving files in various formats.

load(path) # results in an error since the path is relative to @__DIR__, not the current working directory (pwd()).
load(joinedpath) # Works since it provides an absolute path to the file.


# You can change the current working directory with `cd()` function.
# Example:
cd(@__DIR__)
load(path) # Now this works since the current working directory is now the same as the directory of the current file.
load(joinedpath) # This also works since the joined path is still a valid absolute path.
# Refering to the discourse, whenever you import or load a file or object, Julia will look for it in the current working directory (pwd()) and then in the directories specified in the LOAD_PATH.


# ALL FUNCTIONS, IMPORTS, ETC IN JULIA ARE DONE WITH RESPECT TO THE CURRENT WORKING DIRECTORY (pwd()). SO YOU MUST ALWAYS EITHER JOIN THE PATH WITH @__DIR__ OR CHANGE THE CURRENT WORKING DIRECTORY TO @__DIR__.
# THE ONE EXECPTION IS THE `include` FUNCTION. THE PATH GIVEN TO THE `include` FUNCTION IS RELATIVE TO THE CURRENT FILE'S DIRECTORY (@__DIR__). SO YOU CAN USE `include("file.jl")` WITHOUT CHANGING THE CURRENT WORKING DIRECTORY OR JOINING THE PATH WITH @__DIR__.

#------------------------------------------------------------------------------------------------

# --- How to use other files in the current file ---
#---------------------------------------------------

# If you want to include the entire code from a file, you can use the `include` function.
#Example:
include("webdemo.jl")
# This would import everything that is defined in `webdemo.jl` into the current file.
# This includes functions, types, variables, etc as well as code blocks that are executed and return a printed result. Essentially, calling include would run the code in `webdemo.jl` as if it were part of the current file.

# to use a modeule, you first import it's source file if it was defined and then call `using` on the module name.
# example: include("module.jl"); using .module;
# https://docs.julialang.org/en/v1/manual/modules/ the julia documentation is a great resource to learn more.