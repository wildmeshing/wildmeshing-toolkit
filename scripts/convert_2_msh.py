import meshio as mio
import glob
import igl

for ext in [".obj", ".ply", ".stl"]:
    for file in glob.glob(f"./**/*{ext}", recursive=True):
        if ext == ".obj":
            v, f = igl.read_triangle_mesh(file)
            m = mio.Mesh(v, [("triangle", f)])
        else:
            m = mio.read(file)

        m.write(
            file.replace(ext, ".msh"),
            file_format="gmsh",
            binary=True
        )

for ext in [".mesh"]:
    for file in glob.glob(f"./**/*{ext}", recursive=True):
        m = mio.read(file)
        T = []
        for c in m.cells:
            if c.type == "tetra":
                T = c.data
                break
        if len(T) == 0:
            print(f"{file} doesnt contain any tets, skipping")
            continue
        m = mio.Mesh(m.points, [("tetra", T)])

        m.write(
            file.replace(ext, "_3d.msh"),
            file_format="gmsh",
            binary=True
        )
