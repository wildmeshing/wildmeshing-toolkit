#!/usr/bin/env python3
import os
import sys
import subprocess
from pathlib import Path

# -------------------------------------------------
# Args
# -------------------------------------------------
if len(sys.argv) != 3 and len(sys.argv) != 4:
    print(
        "Usage:\n"
        "  python remesh_one_model.py <ABC/fusion> <model_name> <project_dir>\n"
    )
    sys.exit(1)

DATASET = sys.argv[1]
MODEL_NAME = sys.argv[2]
if len(sys.argv) >= 4:
    PROJ_DIR = sys.argv[3]
else:
    if DATASET == "ABC":
        PROJ_DIR = "/scratch/yz6675/felix/CAD_2025/"
    elif DATASET == "fusion":
        PROJ_DIR = "/scratch/yz6675/CAD_2024/"
    elif DATASET == "test":
        PROJ_DIR= "/Users/yunfanzhou/CAD_2024/"
    else:
        raise RuntimeError("Please provide project dir")


# -------------------------------------------------
# Paths (adjust once)
# -------------------------------------------------
if DATASET == "ABC":
    RESULT_ROOT = os.path.join(
        PROJ_DIR, "result_abc_0", "abc_0000_step_v00"
    )
    CAD_INPUT_ROOT = os.path.join(
        PROJ_DIR, "abc_0000_step_v00"
    )
elif DATASET == "fusion":
    RESULT_ROOT = os.path.join(
        PROJ_DIR, "myresult/fusion/a1.0.0_00/"
    )
    CAD_INPUT_ROOT = os.path.join(
        PROJ_DIR, "data/fusion/a1.0.0_00/"
    )
elif DATASET == "test":
    PROJ_ROOT = "/Volumes/myssd/CAD_2024_ssd/"
    RESULT_ROOT = os.path.join(PROJ_ROOT, "my_results/abc_test/") 
    CAD_INPUT_ROOT = os.path.join(PROJ_ROOT, "abc_20_data/")
    print("testing mode with hard-coded CAD_INPUT_ROOT and RESULT_ROOT")
else:
    raise RuntimeError("Unknown dataset: " + DATASET)
MODEL_RESULT_DIR = os.path.join(RESULT_ROOT, MODEL_NAME)

FUSED_PKL = os.path.join(MODEL_RESULT_DIR, "fused.pkl")
REMESH_JSON = os.path.join(MODEL_RESULT_DIR, "remesh.json")
REMESH_DONE = os.path.join(MODEL_RESULT_DIR, "remesh.done")
CONVERT_SCRIPT = os.path.join(
    PROJ_DIR,
    "remeshing",
    "wildmeshing-toolkit",
    "app",
    "remeshing",
    "convert_pkl_to_wmtk.py",
)

REMESH_APP = os.path.join(
    PROJ_DIR,
    "remeshing",
    "wildmeshing-toolkit",
    "build_stable",
    "app",
    "remeshing_app",
)

POST_PROCESS_APP = os.path.join(
        PROJ_DIR,
        "cad_meshing",
        "metaprogramming",
        "convert_remeshed_vtus.py",
)

# -------------------------------------------------
def run(cmd, cwd=None):
    print(">>>", " ".join(cmd))
    subprocess.check_call(cmd, cwd=cwd)

# -------------------------------------------------
print(f"=== Remeshing {MODEL_NAME} ===")

# -------------------------
# Stage 0: Sanity checks
# -------------------------
if not os.path.isdir(MODEL_RESULT_DIR):
    raise RuntimeError(f"Missing model result dir: {MODEL_RESULT_DIR}")

if not os.path.exists(FUSED_PKL):
    print("[Skip] fused.pkl not found")
    sys.exit(0)

# -------------------------
# Stage 1: fused.pkl -> remesh.json
# -------------------------
if not os.path.exists(REMESH_JSON):
    run([
        "python",
        CONVERT_SCRIPT,
        MODEL_RESULT_DIR,
    ])
else:
    print("[Stage 1] remesh.json exists, skipped")

if not os.path.exists(REMESH_JSON):
    raise RuntimeError("Failed to generate remesh.json")

# -------------------------
# Stage 2: remeshing_app
# -------------------------
if not os.path.exists(REMESH_DONE):
    run([
        REMESH_APP,
        "-j",
        REMESH_JSON,
    ])

    # success marker
    Path(REMESH_DONE).touch()
else:
    print("[Stage 2] remeshing already done, skipped")

print(f"=== Done remeshing {MODEL_NAME} ===")

# -------------------------
# Stage 3: verify and render
# -------------------------
if os.path.exists(REMESH_DONE):
    run(["python",
        POST_PROCESS_APP,
        MODEL_NAME,
        CAD_INPUT_ROOT,
        RESULT_ROOT])

print(f"=== Done remeshing post-processing {MODEL_NAME} ===")