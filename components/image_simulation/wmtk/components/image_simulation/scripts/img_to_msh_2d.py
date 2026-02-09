import numpy as np
import argparse
import imageio.v2 as imageio


"""
Convert a grayscale image (PNG format) to a 2d mesh (MSH format). Grayscale values are
read as tags (uint8). Image is centered at the origin

Usage:
python img_to_msh.py --i input_image.png --o output_mesh.msh --pxw 1.0

Arguments:
--i: Path to the input image file (PNG format).
--o: Path to the output mesh file (MSH format).
--pxw: Pixel width/height.
"""


"""
local pixel vertex ids
0 ------- 1
|         |
|         |
|         |
2 ------- 3
"""
def px_id_map(y_im, x_im, local_vi, W):
    if local_vi in [0, 1]:
        return y_im*(W+1) + x_im + local_vi
    elif local_vi in [2, 3]:
        return (y_im + 1)*(W+1) + x_im + local_vi - 2
    else:
        raise ValueError(f"invalid local vertex id [{local_vi}]")


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


def read_img(path):
    img = imageio.imread(path)
    if img.ndim == 3:
        img = img[..., 0]
    return img.astype(np.uint8)


def coords(global_vid, W, H, pxw):
    x_ind = global_vid % (W+1)
    y_ind = global_vid // (W+1)
    x = (x_ind - (W/2))*pxw
    y = ((H/2) - y_ind)*pxw
    return x, y


def form_mesh(im_arr, px_w):
    H = im_arr.shape[0]
    W = im_arr.shape[1]
    origin = np.array([W/2, H/2]) * px_w
    linx = (np.arange(0, W) + 0.5) * px_w
    liny = (np.arange(0, H) + 0.5) * px_w
    X, Y = np.meshgrid(linx, liny)

    # coords = np.flip(np.stack([X, Y], axis=-1), axis=0) - origin
    # print(coords.shape)
    # print(coords[0, 0])
    # print(coords[149, 0])
    # print(coords[0, 99])

    V = np.zeros([(W+1)*(H+1), 2], dtype=np.float64)
    for i in range((W+1)*(H+1)):
        x, y = coords(i, W, H, px_w)
        V[i, 0] = x
        V[i, 1] = y
    
    F = np.zeros([W*H*2, 3], dtype=np.uint32)
    T = np.zeros([W*H*2], dtype=np.uint8)
    index = 0
    for i in range(H):
        for j in range(W):
            v0 = px_id_map(i, j, 0, W)
            v1 = px_id_map(i, j, 1, W)
            v2 = px_id_map(i, j, 2, W)
            v3 = px_id_map(i, j, 3, W)

            F[index, :] = np.array([v2, v3, v0])
            T[index] = im_arr[i, j]
            index += 1
            F[index, :] = np.array([v3, v1, v0])
            T[index] = im_arr[i, j]
            index += 1
    return V, F, T


if __name__ == "__main__":
    # parse arguments
    parser = argparse.ArgumentParser(description="Convert an image to a tagged MSH file.")
    parser.add_argument(
        "--i",
        type=str,
        help="Path to the input image file (PNG format).",
    )
    parser.add_argument(
        "--o",
        type=str,
        help="Path to the output mesh file (MSH format).",
    )
    parser.add_argument(
        "--pxw",
        type=float,
        help="Pixel width/height"
    )

    args = parser.parse_args()
    # make sure all arguments provided
    assert((args.i is not None) and (args.o is not None) and (args.pxw is not None))
    
    im_arr = read_img(args.i)
    V, F, T = form_mesh(im_arr, args.pxw)
    write_msh(args.o, V, F, T)
