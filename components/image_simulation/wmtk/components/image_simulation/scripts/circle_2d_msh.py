import numpy as np
from scipy.spatial import Delaunay
import argparse


"""
Generate a square(ish) 2d mesh centered at the origin. All tris completely in the
given circle are tagged '1', all other tris are tagged '0'. Output saved as .msh (2D)

Usage:
python circle_2d_mesh.py --o output_mesh.msh --L 10.0 --spacing 1.0 --R 2.5

Arguments:
--o: Path to the output mesh file (MSH format).
--L: Side length for bounding box of mesh (mesh spans about -L/2 to L/2 in x and y)
--spacing: Distance between points in uniform hexagonal lattice
--R: Radius of circle, all tris completely inside are considered inside
"""


def hex_lattice_points(L, spacing):
    dx = spacing
    dy = np.sqrt(3.0) * 0.5 * dx

    j_min = int(np.ceil(-L / 2.0 / dy))
    j_max = int(np.floor(L / 2.0 / dy))

    pts = []
    for j in range(j_min, j_max + 1):
        y = j * dy
        shift = 0.5 * dx if (j % 2) != 0 else 0.0

        i_min = int(np.ceil((-L / 2.0 - shift) / dx))
        i_max = int(np.floor((L / 2.0 - shift) / dx))

        for i in range(i_min, i_max + 1):
            x = i * dx + shift
            pts.append((x, y))

    return np.array(pts, dtype=float)


def write_msh(fname, pts, simplices, tags):
    with open(fname, "w") as f:
        # format
        f.write("$MeshFormat\n4.1 0 8\n$EndMeshFormat\n")

        # entities
        f.write("$Entities\n0 0 1 0\n")
        xmin, ymin = pts.min(axis=0)
        xmax, ymax = pts.max(axis=0)
        f.write(f"1 {xmin} {ymin} 0 {xmax} {ymax} 0 0 0\n")
        f.write("$EndEntities\n")

        # nodes
        f.write("$Nodes\n")
        num_nodes = pts.shape[0]
        f.write(f"1 {num_nodes} 1 {num_nodes}\n")
        f.write(f"2 1 0 {num_nodes}\n")
        for i in range(1, num_nodes + 1):
            f.write(f"{i}\n")
        for x, y in pts:
            f.write(f"{x} {y} 0\n")
        f.write("$EndNodes\n")

        # elements
        f.write("$Elements\n")
        num_elems = simplices.shape[0]
        f.write(f"1 {num_elems} 1 {num_elems} \n")
        f.write(f"2 1 2 {num_elems}\n")
        for eid, tri_nodes in enumerate(simplices, start=1):
            n1, n2, n3 = (tri_nodes + 1).tolist()
            f.write(f"{eid} {n1} {n2} {n3}\n")
        f.write("$EndElements\n")

        # labels
        f.write("$ElementData\n")
        f.write("1\n\"tag_0\"\n1\n0.0\n3\n0\n1\n")
        f.write(f"{num_elems}\n")
        for eid, tag in enumerate(tags, start=1):
            f.write(f"{eid} {int(tag)}\n")
        f.write("$EndElementData\n")


def generate_hex_square_mesh(out_fpath, L, spacing, r):
    pts = hex_lattice_points(L, spacing)
    if pts.shape[0] < 3:
        raise ValueError("Not enough points to mesh. Increase L or decrease spacing.")

    tri = Delaunay(pts)  # automatically gives positive orientation
    simplices = tri.simplices

    tri_pts = pts[simplices]
    r2s = tri_pts[:, :, 0]**2 + tri_pts[:, :, 1]**2
    max_r2s = np.max(r2s, axis=1)
    tags = (max_r2s <= r**2).astype(int)

    write_msh(out_fpath, pts, simplices, tags)


if __name__ == "__main__":
    # parse arguments
    parser = argparse.ArgumentParser(description="Generate a square(ish) 2D MSH of a tagged circle.")
    parser.add_argument(
        "--o",
        type=str,
        help="Path to the output mesh file (MSH format).",
    )
    parser.add_argument(
        "--L",
        type=float,
        help="Side length of bounding box."
    )
    parser.add_argument(
        "--spacing",
        type=float,
        help="Distance between points in uniform hexagonal lattice."
    )
    parser.add_argument(
        "--R",
        type=float,
        help="Radius of circle to be tagged."
    )

    args = parser.parse_args()
    # make sure all arguments provided
    assert((args.o is not None) and (args.L is not None) and (args.spacing is not None) and (args.R is not None))
    generate_hex_square_mesh(args.o, args.L, args.spacing, args.R)