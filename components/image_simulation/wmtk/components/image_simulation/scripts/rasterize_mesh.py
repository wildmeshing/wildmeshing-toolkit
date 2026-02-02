import igl
import numpy as np
import argparse

"""
Rasterize a mesh into a voxel grid using libigl's winding number function.
Saves the resulting voxel grid as an ASCII RAW file with padding.

Usage:
python rasterize_mesh.py --i input_mesh.obj --o output_volume.raw --nx 50 --ny 50 --nz 50

Arguments:
--i: Path to the input mesh file (OBJ format). Multiple files can be provided.
--o: Path to the output volume file (RAW format). If the filename ends with .nrrd, the output will be saved in NRRD format.
--na: Dimension of the voxel grid in the direction with smallest extent (optional). The other dimensions are set adaptively to maintain aspect ratio 1.
--spacing: Voxel spacing (optional). If not provided, spacing is computed based on other parameters.
--nx: Dimension of the voxel grid in the X direction (optional).
--ny: Dimension of the voxel grid in the Y direction (optional).
--nz: Dimension of the voxel grid in the Z direction (optional).
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
        "--na",
        type=int,
        help="Dimension of the voxel grid in the direction with smallest extent.",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        help="Voxel spacing. If not provided, spacing is computed based on other parameters.",
    )
    parser.add_argument(
        "--nx",
        type=int,
        help="Dimension of the voxel grid in the X direction.",
    )
    parser.add_argument(
        "--ny",
        type=int,
        help="Dimension of the voxel grid in the Y direction.",
    )
    parser.add_argument(
        "--nz",
        type=int,
        help="Dimension of the voxel grid in the Z direction.",
    )

    args = parser.parse_args()
    ## make sure either all of nx, ny, nz are provided or none of them are provided
    assert ((args.nx is None and args.ny is None and args.nz is None)
            or (args.nx is not None and args.ny is not None and args.nz is not None and args.spacing is None)), "Either provide all of --nx, --ny, --nz or none of them. Don't provide --spacing with them."
    assert args.na is None or args.spacing is None, "Provide either --na or --spacing, not both."
    
    
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

    if args.nx is None:
        if args.spacing is not None:
            spacing = args.spacing
        elif args.na is not None:
            spacing = dim.min() / args.na
        else:
            assert False, "Either --spacing or --na must be provided."
        n = np.ceil(dim / spacing).astype(int)
    else:
        spacing = np.array([dim[0] / args.nx, dim[1] / args.ny, dim[2] / args.nz])
        n = np.array([args.nx, args.ny, args.nz])


        
    

    if len(args.i) == 0:
        print("Please provide one or multiple input meshes file using --i")
        exit(1)

    # Use complex number syntax to ensure we get exactly n[i] points in each dimension
    grid_points = (
        np.mgrid[
            bbox_min[0] : bbox_max[0] : n[0]*1j,
            bbox_min[1] : bbox_max[1] : n[1]*1j,
            bbox_min[2] : bbox_max[2] : n[2]*1j,
        ]
        .reshape(3, -1)
        .T
    )
    
    # Make sure the query points are C-contiguous and float64 (row-major)
    grid_points = np.ascontiguousarray(grid_points, dtype=np.float64)

    print(grid_points.shape)

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
        import nrrd
        nrrd.write(args.o, volume)
    elif args.o.endswith(".mat"):
        import scipy.io

        scipy.io.savemat(args.o, {"volume": volume})
    elif args.o.endswith(".h5"):
        import h5py

        with h5py.File(args.o, "w") as f:
            f.create_dataset("volume", data=volume)
    else:
        write_array_data_ascii(args.o, volume)

    print(f"=== Finished rasterization and wrote {args.o} === with spacing {spacing} and size {n}")
