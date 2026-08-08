#!/usr/bin/env bash
# Launch / control the triwild 2D sweep inside tmux, so it survives a dropped
# connection. (On kirby lingering is enabled for this user, so the tmux server and the
# per-model systemd scopes outlive the ssh session.) Set TRIWILD_ROOT to point at the
# sweep root -- build/, data/ and runs/ hang off it; see README.md.
#
#   ./sweep2d.sh quick    300 models spread across the size range -> runs/quick300
#   ./sweep2d.sh full     all ~19,700 models, in filename order  -> runs/full
#   ./sweep2d.sh stop     clean stop: drain, report, exit (resumable)
#   ./sweep2d.sh attach   attach to the running sweep's tmux session
#   ./sweep2d.sh status   one-line progress for both runs
#   ./sweep2d.sh report [quick|full]   regenerate the report without running anything
#
# Resume is just re-running the same subcommand: models already in success/ or
# failure/ are skipped.
#
# Separate session name and separate TRIWILD_* variables from sweep.sh, so a 2D and a
# 3D sweep can coexist on kirby without either one's settings leaking into the other.
set -euo pipefail

ROOT=${TRIWILD_ROOT:-/u/3/daniele/triwild-sweep}
export TRIWILD_ROOT="$ROOT"
RUNNER="$ROOT/scripts/run_triwild_sweep.py"
SESSION=triwild-sweep

export TRIWILD_PARALLEL=${TRIWILD_PARALLEL:-16}
export TRIWILD_THREADS=${TRIWILD_THREADS:-8}              # 16 x 8 = 128 = kirby's core count
export TRIWILD_JOB_TIMEOUT=${TRIWILD_JOB_TIMEOUT:-3600}   # 1 hour
# Per model, not a sweep budget. 16 x 128G is far past kirby's 440G, so this is a
# runaway-killer rather than admission control. It binds harder here than in 3D: the
# dataset spans 2.7 KB to 1.6 GB per file, and the big ones are read whole up front.
export TRIWILD_MEM_GB=${TRIWILD_MEM_GB:-128}              # per model

out_for() { case "$1" in quick) echo "$ROOT/runs/quick300";; *) echo "$ROOT/runs/full";; esac; }

start() {
    local mode="$1" out; out="$(out_for "$mode")"
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "a 2D sweep session is already running -- './sweep2d.sh attach' or './sweep2d.sh stop' first."
        exit 1
    fi
    mkdir -p "$out"
    local env_extra=""
    if [ "$mode" = quick ]; then
        # 300 rather than 100: the 2D dataset is twice the size of Thingi10K and the
        # known failure mode (arrangement orientation) hits roughly a third of models,
        # so a 100-model trial would put a wide error bar on the number that matters.
        env_extra="TRIWILD_LIMIT=300 TRIWILD_SAMPLE=spread"
    else
        # Filename order, not size order: size is uncorrelated with the name, so the
        # expensive models are spread through the run rather than all landing at the end.
        env_extra="TRIWILD_SAMPLE=name"
    fi
    # keep the pane alive after the run so the tail of the report stays readable.
    # Every knob spelled out in the tmux command rather than relying on the tmux server
    # inheriting our environment: a resume days later must run at the same settings as
    # the run it is resuming, whatever the shell that launches it.
    tmux new-session -d -s "$SESSION" -n "$mode" \
        "TRIWILD_OUT='$out' TRIWILD_PARALLEL=$TRIWILD_PARALLEL TRIWILD_THREADS=$TRIWILD_THREADS \
         TRIWILD_JOB_TIMEOUT=$TRIWILD_JOB_TIMEOUT TRIWILD_MEM_GB=$TRIWILD_MEM_GB \
         $env_extra python3 '$RUNNER' 2>&1 | tee -a '$out/sweep.console.log'; \
         echo; echo '[sweep finished -- pane kept open; ctrl-b d to detach]'; exec bash"
    echo "started '$mode' 2D sweep in tmux session '$SESSION'"
    echo "  output:  $out"
    echo "  console: $out/sweep.console.log"
    echo "  attach:  tmux attach -t $SESSION"
}

case "${1:-}" in
    quick|full) start "$1" ;;
    stop)
        for m in quick full; do
            out="$(out_for "$m")"
            [ -f "$out/sweep.pid" ] && TRIWILD_OUT="$out" python3 "$RUNNER" stop
        done
        ;;
    attach) tmux attach -t "$SESSION" ;;
    report)
        out="$(out_for "${2:-full}")"
        TRIWILD_OUT="$out" TRIWILD_REPORT_ONLY=1 python3 "$RUNNER"
        ;;
    status)
        for m in quick full; do
            out="$(out_for "$m")"
            [ -d "$out" ] || continue
            s=$(find "$out/success" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
            f=$(find "$out/failure" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
            w=$(find "$out/.work"   -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
            running=$([ -f "$out/sweep.pid" ] && echo "running" || echo "idle")
            printf '%-6s %-8s success %-6s failure %-6s in-flight %s\n' "$m" "$running" "$s" "$f" "$w"
        done
        tmux has-session -t "$SESSION" 2>/dev/null && echo "tmux session '$SESSION' is up" || echo "no tmux session"
        ;;
    *)
        sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'
        exit 1
        ;;
esac
