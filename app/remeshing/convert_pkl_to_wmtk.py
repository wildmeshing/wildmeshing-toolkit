import igl
import pickle
import numpy as np
import argparse
import json

# from abs.utils import * # was used to save obj mesh but IGL can do that as well


# default paths
fused_path = "fused.pkl"
mesh_path = "mesh.obj"
# feature_edge_vertex_path = "feature_edge_vertex.json"
# feature_vertex_edge_path = "feature_vertex_edge.json"
# corner_path = "corner.json"
patches_path = "features.json"
remesh_json_path = "remesh.json"

if __name__ == "__main__":
    # read arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=str, help="output directory")
    parser.add_argument(
        "--fused", type=str, default=None, help="input fused pkl file"
    )
    parser.add_argument(
        "--mesh", type=str, default=None, help="output mesh obj file"
    )
    # parser.add_argument(
    #     "--feature_edge_vertex", type=str, default=feature_edge_vertex_path
    # )
    # parser.add_argument(
    #     "--feature_vertex_edge", type=str, default=feature_vertex_edge_path
    # )
    # parser.add_argument("--corner", type=str, default=corner_path)
    parser.add_argument(
        "--patches", type=str, default=None, help="output patches json file"
    )
    parser.add_argument(
        "--remesh_json",
        type=str,
        default=None,
        help="output remeshing config json file (for -j remesh.json)",
    )
    args = parser.parse_args()
    if args.fused:
        fused_path = args.fused
        print("using fused path: ", args.fused)
    else:
        fused_path = args.output_dir + "/" + fused_path
    
    if args.mesh:
        mesh_path = args.mesh
    else:
        mesh_path = args.output_dir + "/" + mesh_path
    # feature_edge_vertex_path = args.feature_edge_vertex
    # feature_vertex_edge_path = args.feature_vertex_edge
    # corner_path = args.corner
    if args.patches:
        patches_path = args.patches
    else:
        patches_path = args.output_dir + "/" + patches_path

    if args.remesh_json:
        remesh_json_path = args.remesh_json
    else:
        remesh_json_path = args.output_dir + "/" + remesh_json_path
    

    with open(fused_path, "rb") as file:
        data = pickle.load(file)

        V = np.array(data[0])
        F = np.array(data[1])

        # patch: is the face fid: triangle id
        patch2fid = data[2]

        # feature edge in the CAD and [list of vertex ids in mesh]
        edge2vidschain = data[3]

        # corner is the CAD vertex and the vids is the vertex id in the mesh
        corner2vids = data[4]

    # save_obj_mesh(mesh_path, V, F)
    igl.write_obj(mesh_path, V, F)
    print("writing mesh to ", mesh_path)

    seg2edge = {}
    for k in edge2vidschain:
        for i in range(len(edge2vidschain[k]) - 1):
            seg = (edge2vidschain[k][i], edge2vidschain[k][i + 1])
            if seg[1] < seg[0]:
                seg = (seg[1], seg[0])
            seg2edge[seg] = k

    # with open(feature_edge_vertex_path, "w") as f:
    #     json.dump(edge2vidschain, f, indent=4)

    seg2edge_tmp = {f"{k[0]},{k[1]}": v for k, v in seg2edge.items()}

    fid2patch = {}
    for p in patch2fid:
        for fid in patch2fid[p]:
            fid2patch[fid] = p

    # with open(feature_vertex_edge_path, "w") as f:
    #     json.dump(seg2edge_tmp, f, indent=2)

    # with open(corner_path, "w") as f:
    #     json.dump(corner2vids, f, indent=4)

    corners = {}
    for c in corner2vids:
        corners[c] = corner2vids[c]

    patches_json = {}
    patches_json["fid2patch"] = fid2patch
    patches_json["edge2seg"] = seg2edge_tmp
    patches_json["corner_vids"] = corners

    with open(patches_path, "w") as f:
        print("writing patche info to ", patches_path)
        json.dump(patches_json, f, indent=4)

    # Write a default remeshing config next to the produced mesh/patches so the
    # remeshing app can be called with: `-j remesh.json`.
    remesh_config = {
        "input": mesh_path,
        "output": args.output_dir + "/remeshed",
        "patches": patches_path,
        "log_file": "",
        "report": "",
        "num_threads": 0,
        "max_iterations": 3,
        "eps_rel": 0.001,
        "use_sample_envelope": True,
        "freeze_boundary": False,
        "length_factor": -1.0,
        "length_abs": -1.0,
        "length_rel": 0.05,
        "DEBUG_output": False,
    }

    with open(remesh_json_path, "w") as f:
        print("writing remesh config to ", remesh_json_path)
        json.dump(remesh_config, f, indent=4)

    print("===== conversion done =====")
