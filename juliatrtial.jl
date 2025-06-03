include("src/WindowManager.jl")

windowmanager = WindowManager()

using GLMakie: Scene

s1 = Scene()
s2 = Scene()
windowmanager.display(s1, s2; names=["Main", "Side"])


windowmanager.closeall()

