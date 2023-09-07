#!/usr/bin/env python
# -*- coding: utf-8 -*-

# System libs
from pathlib import Path
import subprocess
from enum import Enum

# Third-party libs
# from rich import print

import json

class Method(Enum):
    Adaptive = 0
    Uniform = 1
    Adaptive_Tess = 2
    Area_Accuracy = 3


def displace_mesh(input_mesh: Path, intput_heightmap: Path, output_folder: Path, method: Method):
    meshname = input_mesh.name.removesuffix(".obj").lower()
    texname = intput_heightmap.name.removesuffix("_2048_height.exr")
    output_mesh = output_folder.joinpath(f"{meshname}/{texname}.ply")
    output_mesh.parent.mkdir(parents=True, exist_ok=True)
    logfile = output_folder.joinpath(f"{meshname}/{texname}.log")

    displacement_cli = (
        "/Users/jedumas/workspace/adobe/lagrange/lagrange-fresh/build/continuous-release/examples/displacement_demo_cli"
    )
    args = [
        displacement_cli,
        "--input",
        input_mesh,
        output_mesh,
        "--level",
        "1",
        "--log-file",
        logfile,
        "--no-heightmap-edge-drawing",
        "--height",
        intput_heightmap,
        "--max-height",
        "0.1",
        "--relative-height",
    ]
    if method == Method.Uniform:
        if meshname == "unit_square":
            num_subdiv = 2024
        else:
            num_subdiv = 16
        args.extend(
            [
                "--engine",
                "uniform",
                "--uniform-fixed-subdiv",
                str(num_subdiv),
                "--run-decimation",
                "--max-facets",
                "30000",
            ]
        )
    subprocess.run(args, check=False)

def write_config_json(input_mesh: Path, texture: Path, output_folder: Path, method: Method):
    meshname = input_mesh.name.removesuffix(".obj").lower()
    # texname = texture.name.removesuffix("_2048_height.exr")
    texname = texture.name.removesuffix(".exr")
    output_folder = output_folder.joinpath(f"{texname}/")
    output_mesh = output_folder.joinpath(f"{meshname}/{texname}.obj")
    output_mesh.parent.mkdir(parents=True, exist_ok=True)
    logfile = output_folder.joinpath(f"{meshname}/{texname}.json")
     
    image_folder = Path("/mnt/ssd2/yunfan/adaptive_tessellation/textures/3d_mesh/ninja/position_normal_lowres/")
    all_images = list(image_folder.rglob("*.exr"))
    image_list= []
    for image in all_images:
        image_list.append(str(image))
    config_dic = {
    "input_file": str(input_mesh),
    "output_file": str(output_mesh),
    "output_folder": str(output_folder),
    "output_json": str(logfile),
    "image_path": str(texture),
    "image_size": 128,
    "wrapping_mode": 1,
    "target_edge_length": 0.002,
    "target_accuracy": 0.000000001,
    "sampling_mode": 0,
    "displacement_mode": 0,
    "displacement_mesh_images": image_list,
    "edge_len_type": 6,
    "energy_type": 4,
    "boundary_parameter_on": True,
    "max_iter": 3
    }
    config_folder =Path("/mnt/ssd2/yunfan/adaptive_tessellation/config")
    methodname = str(method).lower().removeprefix("method.")

    config_json = config_folder.joinpath(f"{meshname}/{methodname}/{texname}_config.json")
    with open(config_json, "w") as outfile:
        # subprocess.run(["sudo", "chmod", "-R", "775",  config_folder], check=False)
        json.dump(config_dic, outfile)


def displace_all(method: Method):
    all_meshes = [
        Path(x)
        for x in [
            "/mnt/ssd2/yunfan/adaptive_tessellation/inputs/ninjaHead_Low.obj"
        ]
    ]
    # texture_folder = Path("/mnt/ssd2/yunfan/adaptive_tessellation/textures/")
    texture_folder = Path("/mnt/ssd2/yunfan/adaptive_tessellation/textures/3d_mesh/ninja/height_image_lowres/")

    # all_textures = list(texture_folder.rglob("*2048_height.exr"))
    all_textures = list(texture_folder.rglob("*.exr"))
    # for method in Method:
    methodname = str(method).lower().removeprefix("method.")
    output_folder = Path(f"/mnt/ssd2/yunfan/adaptive_tessellation/results/{methodname}")
    
    for mesh in all_meshes:
        for texture in all_textures:
            write_config_json(mesh, texture, output_folder, method)
            # displace_mesh(mesh, texture, output_folder, method)
            
def displace_mesh(input_config: Path):
    displacement_cli = ("/home/yunfan/wildmeshing-toolkit/build_release/app/adaptive_tessellation/adaptive_tessellation")
    for config in input_config.iterdir():
        print(str(config))
        args = [
        displacement_cli,
        "--config",
            str(config)]
        
        # print(" ".join(args).replace("build_release", "build_debug"))
        
        subprocess.run(args, check=True)


if __name__ == "__main__":
    method = 4
    displace_all(method)
    # displace_mesh(Path("/mnt/ssd2/yunfan/adaptive_tessellation/config/square/",str(method)))
    

