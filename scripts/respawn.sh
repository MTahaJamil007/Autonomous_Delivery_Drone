#!/usr/bin/env bash
# respawn.sh - keep one sensor process alive: restart it if it crashes.
#
#   scripts/respawn.sh <name> <command...>
#
# run_system.sh starts each drone's avoider and vision bridge through this.
#
# WHY
# ---
# On 2026-10-13 the avoider raised an exception the moment the drone cleared its
# first obstacle and exited. Navigation did exactly what it should with a dead
# obstacle feed -- held position rather than fly blind -- and kept holding
# until the 600 s leg timeout, because nothing ever restarted the avoider. A
# restart within a second or two would have cost the mission a brief pause.
#
# Holding on a stale feed is still the safety net; this only shortens how long
# the drone has to sit in it. Each restart is logged, so a crash loop is loud.
#
# Exit codes that mean "do not retry":
#   0        clean exit
#   2        configuration error (bad drone id, scan geometry mismatch):
#            restarting cannot fix it, and a tight loop would bury the reason
#   130/143  stopped by Ctrl-C or stop_system.sh

set -uo pipefail

NAME="$1"; shift

child=0
stop() {
    [ "$child" -ne 0 ] && kill "$child" 2>/dev/null
    wait "$child" 2>/dev/null
    exit 0
}
trap stop TERM INT

delay=1
restarts=0
while true; do
    started=$(date +%s)
    "$@" &
    child=$!
    wait "$child"
    rc=$?
    child=0

    case "$rc" in
        0|2|130|143)
            echo "[respawn] $NAME exited with $rc - not restarting" >&2
            exit "$rc"
            ;;
    esac

    # A process that ran for a while before dying gets a fast restart; one that
    # keeps dying on startup backs off, up to 10 s.
    if [ $(( $(date +%s) - started )) -gt 30 ]; then delay=1; else delay=$(( delay * 2 > 10 ? 10 : delay * 2 )); fi
    restarts=$((restarts + 1))
    echo "[respawn] $(date '+%F %T') $NAME exited with $rc - restart #$restarts in ${delay}s" >&2
    sleep "$delay" &
    child=$!
    wait "$child"
    child=0
done
