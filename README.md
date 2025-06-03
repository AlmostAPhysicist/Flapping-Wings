# Flapping Wing Flying Research: Quaternion Visualization & 3D Animation

This project provides interactive tools and visualizations for understanding 3D rotations, quaternions, and coordinate transformations, with a focus on applications in flapping wing flight research. It leverages [Julia](https://julialang.org/) and [Makie.jl](https://makie.juliaplots.org/stable/) for high-performance 3D graphics and animation.

---

## About

- **Quaternion and Rotation Visualization:**  
  Interactive Jupyter notebooks (see [`rotation-demo.ipynb`](rotation-demo.ipynb)) to visualize quaternion-based rotations, coordinate frames, and vector transformations.

- **3D Arrow and Mesh Rendering:**  
  Custom arrow and mesh rendering using Makie and GeometryBasics, including animated coordinate axes and wing models.

- **Animation and Recording:**  
  Smooth interpolation (slerp) between orientations, animated transitions, and video recording of 3D scenes (see `EulerAngles.mp4`, `EulerAnglesWithWing.mp4`).

- **Window Management Utilities:**  
  [`src/WindowManager.jl`](src/WindowManager.jl) provides a `WindowManager` struct for managing multiple GLMakie windows, including inline/standalone display and batch closing.

- **3D Model Integration:**  
  STL and 3DS models (see [`Models/`](Models/)) can be loaded and animated in the scene.

---


## Installation Note

1. **Install Julia** (v1.6 or newer recommended):  
   https://julialang.org/downloads/

2. **Install required Julia packages:**  
   Open Julia and run:
   ```julia
   using Pkg
   Pkg.add(["Makie", "GLMakie", "GeometryBasics", "FileIO", "LinearAlgebra", "Quaternions"])
   ```
   These are all the dependencies for the simulation

3. **(Optional) Jupyter Notebook support:**  
   Install [IJulia](https://github.com/JuliaLang/IJulia.jl) for notebook demos:
   ```julia
   Pkg.add("IJulia")
   ```

I personally don't use it but several people online use this for Julia Notebooks

---

Refer to the `MakieTutorial.ipynb` I made for some Julia and Makie basics.


---

## Demo Videos

<video controls width="640">
  <source src="./slerp_with_state_high_quality.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

<video controls width="640">
  <source src="./EulerAngles.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

<video controls width="640">
  <source src="./EulerAnglesWithWing.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

---
