#!/usr/bin/env python3
"""Run the pinned thick-shell executable and prismatic_mesh on shared fixtures."""

import argparse
import hashlib
import json
import math
import pathlib
import platform
import shutil
import subprocess
import sys


def selected(case, i, j, triangle):
    mode = case.get("selection", "all")
    nx, ny = case["nx"], case["ny"]
    if mode == "all":
        return True
    if mode == "first":
        return i == 0 and j == 0 and triangle == 0
    if mode == "border":
        return i == 0 or j == 0 or i + 1 == nx or j + 1 == ny
    if mode == "ends":
        return i == 0 or i + 1 == nx
    if mode == "missing_center":
        return not (i == nx // 2 and j == ny // 2)
    if mode == "checker":
        return (i + j + triangle) % 2 == 0
    raise ValueError(f"unknown selection mode {mode}")


def grid_fixture(case, x_offset=0.0, z_levels=(-1.0, 0.0, 1.0), shell_layers=(1,)):
    nx, ny = case["nx"], case["ny"]
    skew = case.get("skew", 0.0)
    warp = case.get("warp", 0.0)
    crease = case.get("crease", 0.0)

    nodes = []
    node_id = {}
    for layer, base_z in enumerate(z_levels):
        for j in range(ny + 1):
            for i in range(nx + 1):
                x = x_offset + i / max(1, nx) + skew * j / max(1, ny)
                y = j / max(1, ny)
                z = base_z
                if layer == 1:
                    z += warp * math.sin(math.pi * x) * math.sin(math.pi * y)
                    z += crease * abs(x - 0.5)
                tag = len(nodes) + 1
                nodes.append((tag, x, y, z))
                node_id[(layer, i, j)] = tag

    tets = []
    tet_layers = []
    for layer in range(len(z_levels) - 1):
        for j in range(ny):
            for i in range(nx):
                a = node_id[(layer, i, j)]
                b = node_id[(layer, i + 1, j)]
                c = node_id[(layer, i + 1, j + 1)]
                d = node_id[(layer, i, j + 1)]
                A = node_id[(layer + 1, i, j)]
                B = node_id[(layer + 1, i + 1, j)]
                C = node_id[(layer + 1, i + 1, j + 1)]
                D = node_id[(layer + 1, i, j + 1)]
                layer_tets = [
                    (a, b, c, C),
                    (a, c, d, C),
                    (a, d, D, C),
                    (a, D, A, C),
                    (a, A, B, C),
                    (a, B, b, C),
                ]
                tets.extend(layer_tets)
                tet_layers.extend([layer] * len(layer_tets))

    shell_faces = []
    for shell_layer in shell_layers:
        for j in range(ny):
            for i in range(nx):
                a = node_id[(shell_layer, i, j)]
                b = node_id[(shell_layer, i + 1, j)]
                c = node_id[(shell_layer, i + 1, j + 1)]
                d = node_id[(shell_layer, i, j + 1)]
                if selected(case, i, j, 0):
                    shell_faces.append((a, b, c))
                if selected(case, i, j, 1):
                    shell_faces.append((a, c, d))
    return nodes, tets, tet_layers, shell_faces


def merge_fixtures(fixtures):
    nodes, tets, tet_layers, faces = [], [], [], []
    for fixture_nodes, fixture_tets, fixture_layers, fixture_faces in fixtures:
        offset = len(nodes)
        nodes.extend(
            (len(nodes) + 1, x, y, z) for _, x, y, z in fixture_nodes
        )
        tets.extend(tuple(v + offset for v in tet) for tet in fixture_tets)
        faces.extend(tuple(v + offset for v in face) for face in fixture_faces)
        tet_layers.extend(fixture_layers)
    return nodes, tets, tet_layers, faces


def cube_fixture(missing_top=False):
    coordinates = (-1.0, 0.0, 1.0, 2.0)
    nodes, node_id = [], {}
    for k, z in enumerate(coordinates):
        for j, y in enumerate(coordinates):
            for i, x in enumerate(coordinates):
                node_id[(i, j, k)] = len(nodes) + 1
                nodes.append((len(nodes) + 1, x, y, z))
    tets = []
    for k in range(3):
        for j in range(3):
            for i in range(3):
                a, b = node_id[(i, j, k)], node_id[(i + 1, j, k)]
                c, d = node_id[(i + 1, j + 1, k)], node_id[(i, j + 1, k)]
                A, B = node_id[(i, j, k + 1)], node_id[(i + 1, j, k + 1)]
                C, D = node_id[(i + 1, j + 1, k + 1)], node_id[(i, j + 1, k + 1)]
                tets.extend([(a, b, c, C), (a, c, d, C), (a, d, D, C), (a, D, A, C), (a, A, B, C), (a, B, b, C)])
    positions = {tag: (x, y, z) for tag, x, y, z in nodes}
    incidence = {}
    for tet in tets:
        for face in ((tet[0], tet[1], tet[2]), (tet[0], tet[1], tet[3]), (tet[0], tet[2], tet[3]), (tet[1], tet[2], tet[3])):
            incidence[tuple(sorted(face))] = incidence.get(tuple(sorted(face)), 0) + 1
    faces = []
    for face, count in incidence.items():
        if count != 2:
            continue
        points = [positions[v] for v in face]
        on_planes = [all(abs(point[axis] - value) < 1e-12 for point in points) for axis in range(3) for value in (0.0, 1.0)]
        if not any(on_planes) or not all(-1e-12 <= coordinate <= 1.0 + 1e-12 for point in points for coordinate in point):
            continue
        plane = on_planes.index(True)
        axis, value = divmod(plane, 2)
        if missing_top and axis == 2 and value == 1:
            continue
        oriented = list(face)
        a, b, c = (positions[v] for v in oriented)
        normal = (
            (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]),
            (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]),
            (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]),
        )
        center = tuple(sum(point[k] for point in points) / 3 for k in range(3))
        if sum(normal[k] * (center[k] - 0.5) for k in range(3)) < 0:
            oriented[1], oriented[2] = oriented[2], oriented[1]
        faces.append(tuple(oriented))
    return nodes, tets, [0] * len(tets), faces


def tetrahedron_fixture():
    points = [(0, 0, 0), (1, 0, 0), (0.5, 0.866025403784, 0), (0.5, 0.288675134595, 0.816496580928)]
    faces = [(1, 3, 2), (1, 2, 4), (2, 3, 4), (3, 1, 4)]
    nodes = [(i + 1, *point) for i, point in enumerate(points)]
    tets = [(1, 2, 3, 4)]
    for face in faces:
        a, b, c = (points[v - 1] for v in face)
        opposite = next(v for v in range(1, 5) if v not in face)
        center = tuple((a[k] + b[k] + c[k]) / 3 for k in range(3))
        normal = (
            (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]),
            (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]),
            (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]),
        )
        toward_opposite = sum(normal[k] * (points[opposite - 1][k] - center[k]) for k in range(3))
        if toward_opposite > 0:
            normal = tuple(-value for value in normal)
        length = math.sqrt(sum(value * value for value in normal))
        apex = tuple(center[k] + 0.8 * normal[k] / length for k in range(3))
        nodes.append((len(nodes) + 1, *apex))
        tets.append((*face, len(nodes)))
    return nodes, tets, [0] * len(tets), faces


def fan_fixture():
    segments = 6
    nodes, node_id = [], {}
    for layer, z in enumerate((-1.0, 0.0, 1.0)):
        node_id[(layer, -1)] = len(nodes) + 1
        nodes.append((len(nodes) + 1, 0.0, 0.0, z))
        for i in range(segments):
            angle = 2 * math.pi * i / segments
            node_id[(layer, i)] = len(nodes) + 1
            nodes.append((len(nodes) + 1, math.cos(angle), math.sin(angle), z))
    tets, layers = [], []
    for layer in (0, 1):
        for i in range(segments):
            j = (i + 1) % segments
            c, a, b = node_id[(layer, -1)], node_id[(layer, i)], node_id[(layer, j)]
            C, A, B = node_id[(layer + 1, -1)], node_id[(layer + 1, i)], node_id[(layer + 1, j)]
            tets.extend([(c, a, b, C), (a, b, C, A), (b, C, A, B)])
            layers.extend([layer] * 3)
    faces = [
        (node_id[(1, -1)], node_id[(1, i)], node_id[(1, (i + 1) % segments)])
        for i in range(segments)
    ]
    return nodes, tets, layers, faces


def cylinder_fixture(case):
    nx, ny = case["nx"], case["ny"]
    nodes, node_id = [], {}
    for layer, radius in enumerate((0.7, 1.0, 1.3)):
        for j in range(ny + 1):
            for i in range(nx + 1):
                angle = 0.75 * math.pi * i / nx
                tag = len(nodes) + 1
                nodes.append((tag, radius * math.cos(angle), radius * math.sin(angle), j / ny))
                node_id[(layer, i, j)] = tag
    tets, tet_layers = [], []
    for layer in (0, 1):
        for j in range(ny):
            for i in range(nx):
                a, b = node_id[(layer, i, j)], node_id[(layer, i + 1, j)]
                c, d = node_id[(layer, i + 1, j + 1)], node_id[(layer, i, j + 1)]
                A, B = node_id[(layer + 1, i, j)], node_id[(layer + 1, i + 1, j)]
                C, D = node_id[(layer + 1, i + 1, j + 1)], node_id[(layer + 1, i, j + 1)]
                cell = [(a, b, c, C), (a, c, d, C), (a, d, D, C), (a, D, A, C), (a, A, B, C), (a, B, b, C)]
                tets.extend(cell)
                tet_layers.extend([layer] * len(cell))
    faces = []
    for j in range(ny):
        for i in range(nx):
            a, b = node_id[(1, i, j)], node_id[(1, i + 1, j)]
            c, d = node_id[(1, i + 1, j + 1)], node_id[(1, i, j + 1)]
            faces.extend([(a, b, c), (a, c, d)])
    return nodes, tets, tet_layers, faces


def nonmanifold_t_fixture():
    surface_points = [
        (-1, 0, 0), (0, 0, 0), (1, 0, 0), (-1, 1, 0), (0, 1, 0), (1, 1, 0),
        (0, 0, -1), (0, 0, 1), (0, 1, -1), (0, 1, 1),
    ]
    surface_faces = [(1, 2, 5), (1, 5, 4), (2, 3, 6), (2, 6, 5), (7, 2, 5), (7, 5, 9), (2, 8, 10), (2, 10, 5)]
    nodes = [(i + 1, *point) for i, point in enumerate(surface_points)]
    tets, layers = [], []
    for face in surface_faces:
        a, b, c = (surface_points[v - 1] for v in face)
        ux = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1])
        uy = (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2])
        uz = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        length = math.sqrt(ux * ux + uy * uy + uz * uz)
        center = tuple((a[k] + b[k] + c[k]) / 3 for k in range(3))
        apexes = []
        for sign in (-1, 1):
            tag = len(nodes) + 1
            nodes.append((tag, *(center[k] + sign * 0.4 * (ux, uy, uz)[k] / length for k in range(3))))
            apexes.append(tag)
        tets.extend([(*face, apexes[0]), (*face, apexes[1])])
        layers.extend([0, 1])
    return nodes, tets, layers, surface_faces


def fixture_for_case(case):
    geometry = case.get("geometry", "grid")
    if geometry == "fan":
        return fan_fixture()
    if geometry == "tetrahedron":
        return tetrahedron_fixture()
    if geometry == "cube":
        return cube_fixture(False)
    if geometry == "open_cube":
        return cube_fixture(True)
    if geometry == "cylinder":
        return cylinder_fixture(case)
    if geometry == "disconnected":
        return merge_fixtures([grid_fixture(case), grid_fixture(case, x_offset=2.0)])
    if geometry == "nearby_sheets":
        return grid_fixture(case, z_levels=(-0.3, 0.0, 0.06, 0.36), shell_layers=(1, 2))
    if geometry == "nonmanifold_t":
        return nonmanifold_t_fixture()
    distance = case.get("background_distance", 1.0)
    return grid_fixture(case, z_levels=(-distance, 0.0, distance))


def generate_case(case, directory):
    nodes, tets, tet_layers, shell_faces = fixture_for_case(case)
    positions = {tag: (x, y, z) for tag, x, y, z in nodes}
    oriented_tets = []
    for tet in tets:
        a, b, c, d = (positions[v] for v in tet)
        ab = tuple(b[k] - a[k] for k in range(3))
        ac = tuple(c[k] - a[k] for k in range(3))
        ad = tuple(d[k] - a[k] for k in range(3))
        determinant = (
            ab[0] * (ac[1] * ad[2] - ac[2] * ad[1])
            - ab[1] * (ac[0] * ad[2] - ac[2] * ad[0])
            + ab[2] * (ac[0] * ad[1] - ac[1] * ad[0])
        )
        if abs(determinant) < 1e-14:
            raise ValueError(f"degenerate fixture tetrahedron in {case['name']}: {tet}")
        oriented_tets.append(tet if determinant > 0 else (tet[0], tet[1], tet[3], tet[2]))
    tets = oriented_tets

    old_msh_path = directory / "background.msh"
    annotated_msh_path = directory / "annotated.msh"
    old_volume_elements = []
    annotated_elements = []
    element_tag = 1
    for tet, layer in zip(tets, tet_layers):
        old_volume_elements.append((element_tag, 4, 1, 1, tet))
        physical = 3 if case.get("solid") and layer == 0 else 1
        annotated_elements.append((element_tag, 4, physical, physical, tet))
        element_tag += 1
    for face in shell_faces:
        annotated_elements.append((element_tag, 2, 2, 2, face))
        element_tag += 1

    def write_msh(path, elements, include_shell_group, include_solid_group=False):
        with path.open("w", encoding="utf8") as stream:
            stream.write("$MeshFormat\n2.2 0 8\n$EndMeshFormat\n")
            if include_shell_group:
                group_count = 3 if include_solid_group else 2
                stream.write(f"$PhysicalNames\n{group_count}\n")
                stream.write('3 1 "ambient"\n2 2 "shell"\n')
                if include_solid_group:
                    stream.write('3 3 "solid"\n')
                stream.write("$EndPhysicalNames\n")
            else:
                stream.write('$PhysicalNames\n1\n3 1 "ambient"\n$EndPhysicalNames\n')
            stream.write(f"$Nodes\n{len(nodes)}\n")
            for tag, x, y, z in nodes:
                stream.write(f"{tag} {x:.17g} {y:.17g} {z:.17g}\n")
            stream.write("$EndNodes\n")
            stream.write(f"$Elements\n{len(elements)}\n")
            for tag, element_type, physical, entity, vertices in elements:
                values = " ".join(str(v) for v in vertices)
                stream.write(f"{tag} {element_type} 2 {physical} {entity} {values}\n")
            stream.write("$EndElements\n")

    write_msh(old_msh_path, old_volume_elements, False)
    write_msh(annotated_msh_path, annotated_elements, True, case.get("solid", False))

    used = sorted({v for face in shell_faces for v in face})
    obj_index = {tag: i + 1 for i, tag in enumerate(used)}
    node_position = {tag: (x, y, z) for tag, x, y, z in nodes}
    obj_path = directory / "shell.obj"
    with obj_path.open("w", encoding="utf8") as stream:
        for tag in used:
            x, y, z = node_position[tag]
            stream.write(f"v {x:.17g} {y:.17g} {z:.17g}\n")
        for face in shell_faces:
            stream.write("f " + " ".join(str(obj_index[v]) for v in face) + "\n")
    return old_msh_path, annotated_msh_path, obj_path


def run(command, cwd, log_path, timeout):
    try:
        completed = subprocess.run(
            command,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        if isinstance(output, bytes):
            output = output.decode("utf8", errors="replace")
        log_path.write_text(output, encoding="utf8")
        raise RuntimeError(
            f"command timed out after {timeout}s: {' '.join(map(str, command))}; "
            f"see {log_path}"
        ) from error
    log_path.write_text(completed.stdout, encoding="utf8")
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(map(str, command))}; "
            f"see {log_path}"
        )


def locate_output(requested, alternates=()):
    candidates = [requested, requested.with_suffix(".msh"), *alternates]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise RuntimeError(f"expected output was not created: {requested}")

def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def command_output(command):
    completed = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    return completed.stdout.strip() if completed.returncode == 0 else "unavailable"


def unstable_output_runs(compare_binary, outputs):
    def canonical_hash(path):
        comparison = subprocess.run(
            [str(compare_binary), str(path), str(path), "exact"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        try:
            return json.loads(comparison.stdout)["old"]["canonical_hash"]
        except (json.JSONDecodeError, KeyError):
            return None

    if not outputs:
        return []
    baseline = canonical_hash(outputs[0])
    unstable = []
    for index in range(1, len(outputs)):
        current = canonical_hash(outputs[index])
        if baseline is None or current is None or current != baseline:
            unstable.append(index)
    return unstable


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--old-binary", type=pathlib.Path, required=True)
    parser.add_argument("--new-binary", type=pathlib.Path, required=True)
    parser.add_argument("--compare-binary", type=pathlib.Path, required=True)
    parser.add_argument(
        "--manifest",
        type=pathlib.Path,
        default=pathlib.Path(__file__).with_name("manifest.json"),
    )
    parser.add_argument("--work-dir", type=pathlib.Path, required=True)
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--case", action="append", dest="case_names")
    args = parser.parse_args()

    args.old_binary = args.old_binary.resolve()
    args.new_binary = args.new_binary.resolve()
    args.compare_binary = args.compare_binary.resolve()
    args.manifest = args.manifest.resolve()
    args.work_dir = args.work_dir.resolve()
    manifest = json.loads(args.manifest.read_text(encoding="utf8"))
    repository = pathlib.Path(__file__).resolve().parents[3]
    new_commit = command_output(["git", "-C", str(repository), "rev-parse", "HEAD"])
    compiler = command_output(["c++", "--version"]).splitlines()[0]
    args.work_dir.mkdir(parents=True, exist_ok=True)
    reports = []
    failed = False

    for case in manifest["cases"]:
        if args.case_names and case["name"] not in args.case_names:
            continue
        case_dir = args.work_dir / case["name"]
        if case_dir.exists():
            shutil.rmtree(case_dir)
        case_dir.mkdir(parents=True)
        old_input_msh, new_input_msh, shell_obj = generate_case(case, case_dir)

        old_outputs = []
        new_outputs = []
        old_run_errors = []
        new_run_errors = []
        for run_index in range(args.runs):
            run_dir = case_dir / f"run_{run_index}"
            run_dir.mkdir()
            old_requested = run_dir / "old_output.msh"
            new_requested = run_dir / "new_output.msh"
            old_config = {
                "input": str(old_input_msh),
                "tag_input": str(shell_obj),
                "thickness": 0.08,
                "relative_thickness": False,
                # The abandoned app only permits "out" for a lower-dimensional
                # tag_input. The comparator records the resulting one-sided
                # reference explicitly; it is not treated as a correctness oracle.
                "side": "out",
                "output": str(old_requested),
                "report": str(run_dir / "old_report.json"),
                "log_file": str(run_dir / "old_internal.log"),
            }
            new_config = {
                "application": "prismatic_mesh",
                "input": str(new_input_msh),
                "output": str(new_requested),
                "shells": [{"group": "shell", "thickness": 0.08}],
                "solid_groups": ["solid"] if case.get("solid") else [],
                "report": str(run_dir / "new_report.json"),
            }
            old_json = run_dir / "old.json"
            new_json = run_dir / "new.json"
            old_json.write_text(json.dumps(old_config, indent=2), encoding="utf8")
            new_json.write_text(json.dumps(new_config, indent=2), encoding="utf8")
            try:
                run(
                    [str(args.old_binary), "-j", str(old_json)],
                    run_dir,
                    run_dir / "old.log",
                    args.timeout,
                )
                old_outputs.append(
                    locate_output(old_requested, [run_dir / "output.msh"])
                )
            except RuntimeError as error:
                old_run_errors.append({"run": run_index, "error": str(error)})
            try:
                run(
                    [str(args.new_binary), "-j", str(new_json)],
                    run_dir,
                    run_dir / "new.log",
                    args.timeout,
                )
                new_outputs.append(locate_output(new_requested))
            except RuntimeError as error:
                new_run_errors.append({"run": run_index, "error": str(error)})

        if old_run_errors or new_run_errors:
            old_output_instability = unstable_output_runs(args.compare_binary, old_outputs)
            new_output_instability = unstable_output_runs(args.compare_binary, new_outputs)
            error_report = {
                "case": case["name"],
                "declared_tier": case["tier"],
                "effective_tier": "uncomparable",
                "equivalent": False,
                "failures": [entry["error"] for entry in old_run_errors + new_run_errors],
                "old_run_errors": old_run_errors,
                "new_run_errors": new_run_errors,
                "old_unstable_runs":
                    [entry["run"] for entry in old_run_errors] + old_output_instability,
                "new_unstable_runs":
                    [entry["run"] for entry in new_run_errors] + new_output_instability,
            }
            # Preserve metrics for whichever implementation succeeded. This is
            # important for old-code defect cases: a reference assertion must
            # not hide whether the current implementation produced valid data.
            for label, outputs in (("old", old_outputs), ("new", new_outputs)):
                if not outputs:
                    continue
                metrics = subprocess.run(
                    [str(args.compare_binary), str(outputs[0]), str(outputs[0]), "exact"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    check=False,
                )
                try:
                    error_report[label] = json.loads(metrics.stdout)[label]
                except (json.JSONDecodeError, KeyError):
                    pass
            reports.append(error_report)
            failed = True
            print(
                f"{case['name']}: ERROR "
                f"(old {len(old_run_errors)}/{args.runs}, new {len(new_run_errors)}/{args.runs})"
            )
            continue

        tier = case["tier"]
        instability = unstable_output_runs(args.compare_binary, old_outputs)
        new_instability = unstable_output_runs(args.compare_binary, new_outputs)
        if tier == "exact" and instability:
            tier = "semantic"

        comparison = subprocess.run(
            [
                str(args.compare_binary),
                str(old_outputs[0]),
                str(new_outputs[0]),
                tier,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        try:
            report = json.loads(comparison.stdout)
        except json.JSONDecodeError:
            report = {"equivalent": False, "failures": [comparison.stdout]}
        report.update(
            {
                "case": case["name"],
                "declared_tier": case["tier"],
                "effective_tier": tier,
                "old_unstable_runs": instability,
                "new_unstable_runs": new_instability,
                "requires_manifest_update": case["tier"] == "exact" and bool(instability),
                "old_config": str((case_dir / "run_0" / "old.json").resolve()),
                "new_config": str((case_dir / "run_0" / "new.json").resolve()),
            }
        )
        reports.append(report)
        failed = (
            failed
            or comparison.returncode != 0
            or bool(new_instability)
            or (case["tier"] == "exact" and bool(instability))
        )
        print(f"{case['name']}: {'PASS' if comparison.returncode == 0 else 'FAIL'} ({tier})")

    summary = {
        "old_commit": manifest["old_commit"],
        "new_commit": new_commit,
        "platform": platform.platform(),
        "compiler": compiler,
        "runs": args.runs,
        "timeout_seconds": args.timeout,
        "old_binary": str(args.old_binary.resolve()),
        "old_binary_sha256": file_sha256(args.old_binary),
        "new_binary": str(args.new_binary.resolve()),
        "new_binary_sha256": file_sha256(args.new_binary),
        "compare_binary": str(args.compare_binary.resolve()),
        "manifest": str(args.manifest.resolve()),
        "reports": reports,
    }
    (args.work_dir / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf8"
    )
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
