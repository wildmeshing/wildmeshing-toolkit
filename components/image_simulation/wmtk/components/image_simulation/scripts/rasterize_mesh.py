import igl
import numpy as np
import argparse
import nrrd

"""
Rasterize a mesh into a voxel grid using libigl's winding number function.
Saves the resulting voxel grid as an ASCII RAW file with padding.

Usage:
python rasterize_mesh.py --i input_mesh.obj --o output_volume.raw --nx 50 --ny 50 --nz 50

Arguments:
--i: Path to the input mesh file (OBJ format). Multiple files can be provided.
--o: Path to the output volume file (RAW format). If the filename ends with .nrrd, the output will be saved in NRRD format.
--nx: Dimension of the voxel grid in the X direction (optional, default = 50).
--ny: Dimension of the voxel grid in the Y direction (optional, default = 50).
--nz: Dimension of the voxel grid in the Z direction (optional, default = 50).
"""


def add_padding(array):
    dim1, dim2, dim3 = array.shape
    padded_array = np.zeros((dim1 + 2, dim2 + 2, dim3 + 2), dtype=array.dtype)
    padded_array[1:-1, 1:-1, 1:-1] = array
    return padded_array


def write_array_data_ascii(filename, data):
    dim1, dim2, dim3 = data.shape

    with open(filename, "w") as f:
        f.write(str(dim1) + " ")
        f.write(str(dim2) + " ")
        f.write(str(dim3))
        f.write("\n\n")
        for i in range(data.shape[0]):
            for j in range(data.shape[1]):
                for k in range(data.shape[2]):
                    v = int(data[i][j][k])
                    f.write(str(v) + " ")
                f.write("\n")
            f.write("\n")


if __name__ == "__main__":
    # parse arguments

    parser = argparse.ArgumentParser(description="Rasterize a mesh into a voxel grid.")
    parser.add_argument(
        "--i",
        type=str,
        nargs="+",
        default=[],
        help="Path to the input mesh file (OBJ format).",
    )
    parser.add_argument(
        "--o", type=str, help="Path to the output volume file (RAW format)."
    )
    parser.add_argument(
        "--nx",
        type=int,
        default=50,
        help="Dimension of the voxel grid in the X direction.",
    )
    parser.add_argument(
        "--ny",
        type=int,
        default=50,
        help="Dimension of the voxel grid in the Y direction.",
    )
    parser.add_argument(
        "--nz",
        type=int,
        default=50,
        help="Dimension of the voxel grid in the Z direction.",
    )

    args = parser.parse_args()
    n = np.array([args.nx, args.ny, args.nz])

    if len(args.i) == 0:
        print("Please provide one or multiple input meshes file using --i")
        exit(1)

    # get bbox from all meshes
    all_V = []
    all_F = []
    for mesh_file in args.i:
        V, F = igl.read_triangle_mesh(mesh_file)
        # Ensure arrays are the expected dtype and memory layout (row-major / C-contiguous).
        # pybind11 / libigl bindings often require matching memory layout between inputs.
        V = np.ascontiguousarray(V, dtype=np.float64)
        F = np.ascontiguousarray(F, dtype=np.int32)

        all_V.append(V)
        all_F.append(F)

    all_V_stacked = np.vstack(all_V)

    # get bbox from vertices
    bbox_min = np.min(all_V_stacked, axis=0)
    bbox_max = np.max(all_V_stacked, axis=0)
    diag = bbox_max - bbox_min
    # increase bbox by 5% in each direction
    bbox_min -= 0.05 * diag
    bbox_max += 0.05 * diag

    dim = bbox_max - bbox_min
    spacing = dim / n

    grid_points = (
        np.mgrid[
            bbox_min[0] : bbox_max[0] : spacing[0],
            bbox_min[1] : bbox_max[1] : spacing[1],
            bbox_min[2] : bbox_max[2] : spacing[2],
        ]
        .reshape(3, -1)
        .T
    )
    # Make sure the query points are C-contiguous and float64 (row-major)
    grid_points = np.ascontiguousarray(grid_points, dtype=np.float64)

    volume = np.zeros(n, dtype=np.uint8)
    volume = add_padding(volume)

    for i in range(0, len(all_V)):
        V = all_V[i]
        F = all_F[i]

        inside = igl.winding_number(V, F, grid_points) > 0.5

        vol = inside.reshape(n).astype(np.uint8)
        vol = add_padding(vol)
        assert vol.shape == volume.shape
        vol = (i + 1) * vol  # label the volume with mesh index + 1

        volume = np.maximum(volume, vol)  # combine volumes

    if args.o.endswith(".nrrd"):
        nrrd.write(args.o, volume)
    else:
        write_array_data_ascii(args.o, volume)

    print(f"=== Finished rasterization and wrote {args.o} ===")
