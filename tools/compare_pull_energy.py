#!/usr/bin/env python3
"""Compare the two surface-pull energies by sweeping their weight.

Smoothing minimises `w_amips * AMIPS + w_envelope * pull`, with `w_envelope = 1 - w_amips`,
so lowering w_amips raises the pull's weight relative to the quality term. This runs the same
model at a range of w_amips under both pulls and reports how far the output surface ends up
from the input.

    envelope        penalises the distance to the input as a SET, recomputing the nearest
                    point every evaluation; a vertex slides along the input for free.
    spring          captures the nearest point once per solve and springs the vertex to that
                    fixed point. No sliding.
    spring_refresh  the spring with its target re-captured at every accepted iterate.
    exact           (triwild only) the true distance with its true region-wise Hessian.

Select with --modes; each run sets the pull_mode config parameter.

The point of the sweep is the SHAPE of each curve, not any single number. A well-behaved
quadratic penalty gives one decade of deviation per decade of weight; a curve that flattens
is one where something other than the force balance is setting the deviation.

Usage
-----
    # 2D, the model the PR quotes (~1s per run, 14 runs)
    python3 tools/compare_pull_energy.py --app triwild --input data/models/triwild20k_202090.obj

    # 3D
    python3 tools/compare_pull_energy.py --app tetwild --input data/models/sphere.obj

    # wider or narrower sweep, and a plot
    python3 tools/compare_pull_energy.py --app triwild --input <model.obj> \
        --weights 1e-2 1e-4 1e-6 --svg /tmp/sweep.svg

Options:
    --app        triwild (2D) or tetwild (3D)
    --input      input model
    --weights    w_amips values to sweep (default 1e-1 ... 1e-7)
    --outdir     where configs/reports go (default a temp dir)
    --wmtk-app   path to wmtk_app (default build/app/wmtk_app)
    --extra      extra JSON config keys, e.g. --extra '{"stop_energy": 20}'
    --svg        also write a log-log plot there

Only triwild reports the vertex-level figures; tetwild reports `hausdorff` alone, and the
table adapts. Columns:

    vtx max/mean    distance from the output's tracked VERTICES to the input. This is what
                    the pull term actually controls -- smoothing places vertices.
    edge max/mean   the same sampled along the tracked EDGES, so it also carries the
                    discretization error of chords across a curve.
"""
import argparse
import json
import math
import pathlib
import subprocess
import sys
import tempfile

DEFAULT_WEIGHTS = ["1e-1", "1e-2", "1e-3", "1e-4", "1e-5", "1e-6", "1e-7"]
DEFAULT_MODES = ["envelope", "spring", "spring_refresh"]


def run_one(app_bin, outdir, app, model, w_amips, mode, extra):
    d = outdir / f"{mode}_{w_amips}"
    d.mkdir(parents=True, exist_ok=True)
    cfg = {
        "application": app,
        "input": [str(pathlib.Path(model).resolve())],
        "num_threads": 0,
        "w_amips": float(w_amips),
        "pull_mode": mode,
        "DEBUG_hausdorff": True,
        "output": str(d / "out"),
        "report": str(d / "report.json"),
        "log_file": str(d / "log.txt"),
    }
    cfg.update(extra)
    (d / "config.json").write_text(json.dumps(cfg, indent=2))

    proc = subprocess.run(
        [str(app_bin), "-j", str(d / "config.json")], capture_output=True, text=True
    )
    if proc.returncode != 0:
        print(f"  FAILED ({mode}, w_amips={w_amips});"
              f" see {d / 'log.txt'}", file=sys.stderr)
        return None
    return json.loads((d / "report.json").read_text())


def table(results, weights, modes, has_vertex):
    for mode in modes:
        print(f"\n===== {mode.upper()} =====")
        if has_vertex:
            head = (f"{'w_amips':>8} | {'vtx max':>11} {'vtx mean':>11} "
                    f"| {'edge max':>11} {'edge mean':>11} | {'max_E':>7}")
        else:
            head = f"{'w_amips':>8} | {'hausdorff':>11} | {'max_E':>7}"
        print(head)
        for w in weights:
            r = results[(mode, w)]
            if r is None:
                print(f"{w:>8} |  (failed)")
                continue
            if has_vertex:
                print(f"{w:>8} | {r['hausdorff_vertex']:>11.4e} "
                      f"{r['hausdorff_vertex_mean']:>11.4e} | {r['hausdorff']:>11.4e} "
                      f"{r['hausdorff_mean']:>11.4e} | {r['max_energy']:>7.2f}")
            else:
                print(f"{w:>8} | {r['hausdorff']:>11.4e} | {r['max_energy']:>7.2f}")

    key = "hausdorff_vertex_mean" if has_vertex else "hausdorff"
    print(f"\n===== shape of each curve ({key}) =====")
    print("A quadratic penalty gives ~10x per decade of weight; ~1x means a floor.")
    for mode in modes:
        vals = [results[(mode, w)] for w in weights]
        if any(v is None for v in vals):
            continue
        ratios = [vals[i - 1][key] / vals[i][key] for i in range(1, len(vals))]
        pretty = "  ".join(f"{r:5.2f}x" for r in ratios)
        print(f"  {mode:>9}: {pretty}   (total {vals[0][key] / vals[-1][key]:.3g}x)")


def write_svg(path, results, weights, modes, key, ylabel):
    W, H, L, R, T, B = 860, 520, 92, 220, 56, 64
    PW, PH = W - L - R, H - T - B
    pts_all = [results[(m, w)][key] for m in modes for w in weights
               if results[(m, w)] is not None]
    if not pts_all:
        return
    y0 = math.floor(math.log10(min(pts_all))) - 0.3
    y1 = math.ceil(math.log10(max(pts_all))) + 0.3
    x0 = math.log10(1 / float(weights[0])) - 0.3
    x1 = math.log10(1 / float(weights[-1])) + 0.3
    px = lambda x: L + (math.log10(x) - x0) / (x1 - x0) * PW
    py = lambda y: T + (y1 - math.log10(y)) / (y1 - y0) * PH

    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
         f'viewBox="0 0 {W} {H}" font-family="Helvetica,Arial,sans-serif">',
         f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
         f'<text x="{L}" y="30" font-size="17" font-weight="600" fill="#16181d">'
         f'Surface pull: envelope vs fixed-target spring</text>']
    for e in range(int(math.ceil(x0)), int(x1) + 1):
        x = px(10.0 ** e)
        o.append(f'<line x1="{x:.1f}" y1="{T}" x2="{x:.1f}" y2="{T+PH}" stroke="#e8eaee"/>')
        o.append(f'<text x="{x:.1f}" y="{T+PH+20}" font-size="11.5" fill="#5b6270" '
                 f'text-anchor="middle">10^{e}</text>')
    for e in range(int(math.ceil(y0)), int(y1) + 1):
        y = py(10.0 ** e)
        o.append(f'<line x1="{L}" y1="{y:.1f}" x2="{L+PW}" y2="{y:.1f}" stroke="#e8eaee"/>')
        o.append(f'<text x="{L-10}" y="{y+4:.1f}" font-size="11.5" fill="#5b6270" '
                 f'text-anchor="end">1e{e}</text>')
    o.append(f'<line x1="{L}" y1="{T+PH}" x2="{L+PW}" y2="{T+PH}" stroke="#9aa1ad"/>')
    o.append(f'<line x1="{L}" y1="{T}" x2="{L}" y2="{T+PH}" stroke="#9aa1ad"/>')
    o.append(f'<text x="{L+PW/2:.0f}" y="{H-18}" font-size="12.5" fill="#3a3f4a" '
             f'text-anchor="middle">pull weight / AMIPS weight</text>')
    o.append(f'<text transform="translate(24,{T+PH/2:.0f}) rotate(-90)" font-size="12.5" '
             f'fill="#3a3f4a" text-anchor="middle">{ylabel}</text>')

    palette = ["#1f6fb4", "#c4452f", "#3a8f5d", "#8455a5"]
    for mode, color in zip(modes, palette):
        xs, ys = [], []
        for w in weights:
            r = results[(mode, w)]
            if r is not None:
                xs.append(1 / float(w))
                ys.append(r[key])
        if not xs:
            continue
        o.append('<polyline points="' + " ".join(f"{px(x):.1f},{py(y):.1f}"
                 for x, y in zip(xs, ys)) + f'" fill="none" stroke="{color}" '
                 'stroke-width="2.2" stroke-linejoin="round"/>')
        for x, y in zip(xs, ys):
            o.append(f'<circle cx="{px(x):.1f}" cy="{py(y):.1f}" r="3.4" fill="#fff" '
                     f'stroke="{color}" stroke-width="2"/>')
        o.append(f'<text x="{px(xs[-1])+12:.1f}" y="{py(ys[-1])+4:.1f}" font-size="12" '
                 f'font-weight="600" fill="{color}">{mode}</text>')
        o.append(f'<text x="{px(xs[-1])+12:.1f}" y="{py(ys[-1])+19:.1f}" font-size="11" '
                 f'fill="#6b717c">{ys[-1]:.2e}</text>')
    o.append("</svg>")
    pathlib.Path(path).write_text("\n".join(o))
    print(f"\nwrote {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--app", choices=["triwild", "tetwild"], default="triwild")
    ap.add_argument("--input", required=True)
    ap.add_argument("--weights", nargs="+", default=DEFAULT_WEIGHTS)
    ap.add_argument(
        "--modes",
        nargs="+",
        default=DEFAULT_MODES,
        help="pull_mode values to sweep; 'exact' is triwild-only")
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--wmtk-app", default="build/app/wmtk_app")
    ap.add_argument("--extra", default="{}")
    ap.add_argument("--svg", default=None)
    a = ap.parse_args()

    app_bin = pathlib.Path(a.wmtk_app).resolve()
    if not app_bin.exists():
        sys.exit(f"wmtk_app not found at {app_bin}; build it or pass --wmtk-app")
    outdir = pathlib.Path(a.outdir).resolve() if a.outdir else \
        pathlib.Path(tempfile.mkdtemp(prefix="pull_sweep_"))
    extra = json.loads(a.extra)
    print(f"{a.app}  {a.input}\noutput -> {outdir}\n")

    results = {}
    for mode in a.modes:
        for w in a.weights:
            print(f"  running {mode:>9}  w_amips={w} ...", flush=True)
            results[(mode, w)] = run_one(app_bin, outdir, a.app, a.input, w, mode, extra)

    ok = [r for r in results.values() if r is not None]
    if not ok:
        sys.exit("every run failed")
    has_vertex = "hausdorff_vertex" in ok[0]
    table(results, a.weights, a.modes, has_vertex)
    if a.svg:
        key = "hausdorff_vertex_mean" if has_vertex else "hausdorff"
        write_svg(a.svg, results, a.weights, a.modes, key,
                  "mean vertex distance to input" if has_vertex else "hausdorff")


if __name__ == "__main__":
    main()
