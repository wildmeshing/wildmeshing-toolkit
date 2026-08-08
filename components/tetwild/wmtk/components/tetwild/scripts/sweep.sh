#!/usr/bin/env bash
# Launch / control the tetwild Thingi10K sweep on kirby, inside tmux so it survives
# a dropped connection. (Lingering is enabled for this user, so the tmux server and
# the per-model systemd scopes outlive the ssh session.)
#
#   ./sweep.sh quick    100 models spread across the size range -> runs/quick100
#   ./sweep.sh full     all 10,000 models, smallest first       -> runs/full
#   ./sweep.sh stop     clean stop: drain, report, exit (resumable)
#   ./sweep.sh attach   attach to the running sweep's tmux session
#   ./sweep.sh status   one-line progress for both runs
#   ./sweep.sh report [quick|full]   regenerate the report without running anything
#
# Resume is just re-running the same subcommand: models already in success/ or
# failure/ are skipped.
set -euo pipefail

ROOT=${TETWILD_ROOT:-/u/3/daniele/thingi10k-sweep}
export TETWILD_ROOT="$ROOT"
RUNNER="$ROOT/scripts/run_tetwild_sweep.py"
SESSION=tetwild-sweep

export TETWILD_PARALLEL=${TETWILD_PARALLEL:-16}
export TETWILD_THREADS=${TETWILD_THREADS:-8}               # 16 x 8 = 128 = kirby's core count
export TETWILD_JOB_TIMEOUT=${TETWILD_JOB_TIMEOUT:-10800}   # 3 hours
# Per model, not a sweep budget. 16 x 128G is far past kirby's 440G, so this is a
# runaway-killer rather than admission control: it stops one pathological mesh from
# swapping the box, and does nothing in the normal case. It only binds when a single
# model is genuinely huge -- three models hit the previous 64G cap.
export TETWILD_MEM_GB=${TETWILD_MEM_GB:-128}               # per model

out_for() { case "$1" in quick) echo "$ROOT/runs/quick100";; *) echo "$ROOT/runs/full";; esac; }

start() {
    local mode="$1" out; out="$(out_for "$mode")"
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "a sweep session is already running -- './sweep.sh attach' or './sweep.sh stop' first."
        exit 1
    fi
    mkdir -p "$out"
    local env_extra=""
    if [ "$mode" = quick ]; then
        env_extra="TETWILD_LIMIT=100 TETWILD_SAMPLE=spread"
    else
        env_extra="TETWILD_SAMPLE=smallest"
    fi
    # keep the pane alive after the run so the tail of the report stays readable
    # Every knob spelled out in the tmux command rather than relying on the tmux
    # server inheriting our environment: a resume days later must run at the same
    # settings as the run it is resuming, whatever the shell that launches it.
    tmux new-session -d -s "$SESSION" -n "$mode" \
        "TETWILD_OUT='$out' TETWILD_PARALLEL=$TETWILD_PARALLEL TETWILD_THREADS=$TETWILD_THREADS \
         TETWILD_JOB_TIMEOUT=$TETWILD_JOB_TIMEOUT TETWILD_MEM_GB=$TETWILD_MEM_GB \
         $env_extra python3 '$RUNNER' 2>&1 | tee -a '$out/sweep.console.log'; \
         echo; echo '[sweep finished -- pane kept open; ctrl-b d to detach]'; exec bash"
    echo "started '$mode' sweep in tmux session '$SESSION'"
    echo "  output:  $out"
    echo "  console: $out/sweep.console.log"
    echo "  attach:  tmux attach -t $SESSION"
}

case "${1:-}" in
    quick|full) start "$1" ;;
    stop)
        for m in quick full; do
            out="$(out_for "$m")"
            [ -f "$out/sweep.pid" ] && TETWILD_OUT="$out" python3 "$RUNNER" stop
        done
        ;;
    attach) tmux attach -t "$SESSION" ;;
    report)
        out="$(out_for "${2:-full}")"
        TETWILD_OUT="$out" TETWILD_REPORT_ONLY=1 python3 "$RUNNER"
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
        sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
        exit 1
        ;;
esac
