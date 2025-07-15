using GLMakie, GeometryBasics, LinearAlgebra

struct Dodecagon
    vertices::Vector{Point3{Float64}}
    faces::Vector{NgonFace{5, Int}}
    face_normals::Vector{Vec3{Float64}}
    face_centroids::Vector{Point3{Float64}}
    face_areas::Vector{Float64}
end

function recompute_mesh_with_normals_for_flat_shading(mesh::GeometryBasics.Mesh)
    # Extract vertices and faces from the original mesh
    original_coords = mesh.vertex_attributes[1]
    original_faces = mesh.faces

    # Compute the centroid of all vertices (approximate center of the polyhedron)
    centroid = sum(original_coords) / length(original_coords)

    new_coords = Point3f[]
    new_normals = Vec3f[]
    new_faces = NgonFace{length(original_faces[1]), Int}[]  # Adjust N as needed for your mesh

    for face in original_faces
        # Get the vertices of the face
        verts = [original_coords[idx] for idx in face]
        # Compute the face normal (using the first three vertices)
        edge1 = verts[2] - verts[1]
        edge2 = verts[3] - verts[1]
        normal = normalize(cross(Vec3f(edge1), Vec3f(edge2)))

        # Compute the face centroid
        face_centroid = sum(verts) / length(verts)

        # Determine if the normal points outward (away from the overall centroid)
        # If the dot product of (face_centroid - centroid) and normal is negative,
        # the normal points inward, so negate it
        if dot(face_centroid - centroid, normal) < 0
            normal = -normal
        end

        # Duplicate the vertices for this face and assign the same normal to all
        base_idx = length(new_coords) + 1
        for v in verts
            push!(new_coords, v)
            push!(new_normals, normal)
        end

        # Define the new face with the new vertex indices
        push!(new_faces, NgonFace{length(verts), Int}(base_idx:base_idx+length(verts)-1))
    end

    # Create the new mesh with explicit normals
    return GeometryBasics.Mesh(new_coords, new_faces; normals=new_normals)
end

function get_dodecagon_vertices_and_faces(pos::AbstractVector{<:Real}=Point3{Float64}(0.0, 0.0, 0.0), radius::Real=1.0)

    @assert length(pos) == 3 "Position must be a 3D vector."
    @assert radius > 0 "Radius must be a positive number."

    # Golden ratio
    phi = (1 + sqrt(5)) / 2

    # Define the 20 vertices based on the Fortran output table
    vertices = Point3f[
        (1/phi, 0.0, phi),      # 1
        (phi, 1/phi, 0.0),      # 2
        (0.0, phi, 1/phi),      # 3
        (-1/phi, 0.0, -phi),     # 4
        (-phi, 1/phi, 0.0),     # 5
        (0.0, phi, -1/phi),     # 6
        (1/phi, 0.0, -phi),     # 7
        (phi, -1/phi, 0.0),     # 8
        (0.0, -phi, -1/phi),     # 9
        (-1/phi, 0.0, phi),    # 10
        (-phi, -1/phi, 0.0),    # 11
        (0.0, -phi, 1/phi),    # 12
        (1.0, 1.0, 1.0),        # 13
        (1.0, 1.0, -1.0),       # 14
        (-1.0, 1.0, -1.0),       # 15
        (-1.0, 1.0, 1.0),       # 16
        (1.0, -1.0, -1.0),      # 17
        (1.0, -1.0, 1.0),      # 18
        (-1.0, -1.0, 1.0),      # 19
        (-1.0, -1.0, -1.0)      # 20
    ]

    # Define the 12 faces based on the Fortran output table (each face has 5 vertices)
    faces = NgonFace{5, Int}[
        [1, 10, 16, 3, 13],  # Face 1
        [1, 10, 19, 12, 18], # Face 2
        [1, 13, 2, 8, 18],   # Face 3
        [2, 8, 17, 7, 14],   # Face 4
        [2, 13, 3, 6, 14],   # Face 5
        [3, 6, 15, 5, 16],  # Face 6
        [4, 7, 14, 6, 15],  # Face 7
        [4, 7, 17, 9, 20], # Face 8
        [4, 15, 5, 11, 20],   # Face 9
        [5, 11, 19, 10, 16],   # Face 10
        [8, 17, 9, 12, 18],  # Face 11
        [9, 12, 19, 11, 20]  # Face 12
    ]

    # Scale and translate vertices
    function lerp(a, b, t)
        return a + (b - a) * t
    end
    vertices = [pos + (v .* radius/sqrt(3)*lerp(sqrt((15*(3+sqrt(5)))/(25+11*sqrt(5))), 1, 1/phi)) for v in vertices]

    return vertices, faces
end

function get_dodecagon_mesh(pos::AbstractVector{<:Real}=Point3{Float64}(0.0, 0.0, 0.0), radius::Real=1.0)

    @assert length(pos) == 3 "Position must be a 3D vector."
    @assert radius > 0 "Radius must be a positive number."

    vertices, faces = get_dodecagon_vertices_and_faces(pos, radius)

    # Create the mesh with normals
    mesh = GeometryBasics.Mesh(vertices, faces; normals=Vec3f[])
    return recompute_mesh_with_normals_for_flat_shading(mesh)
end

function Dodecagon(pos::AbstractVector{<:Real}=Point3{Float64}(0.0, 0.0, 0.0), radius::Real=1.0)

    @assert length(pos) == 3 "Position must be a 3D vector."
    @assert radius > 0 "Radius must be a positive number."

    vertices, faces = get_dodecagon_vertices_and_faces(pos, radius)

    # Compute face centroids, normals and face area
    face_normals = Vector{Vec3{Float64}}()
    face_centroids = Vector{Point3{Float64}}()
    face_areas = Vector{Float64}()
    for face in faces
        tail = pos  # Initialize normal tail 

        
        v = vertices[face] # Get the vertices of the face
        face_centroid = sum(v) / length(v)  # Compute the centroid of the face
        face_normal = normalize(face_centroid - tail)  # Compute the face normal
        face_area = 1/2 * 5 * norm(v[1]-face_centroid)^2 * sin(2 * pi / 5)  # Area of a regular pentagon
        push!(face_normals, face_normal)
        push!(face_centroids, face_centroid)
        push!(face_areas, face_area)

    end

    return Dodecagon(vertices, faces, face_normals, face_centroids, face_areas)
end
Main.mesh(d::Dodecagon) = recompute_mesh_with_normals_for_flat_shading(GeometryBasics.Mesh(d.vertices, d.faces))

