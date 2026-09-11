#!/bin/bash
#
# Runs the threading pass once per case, capturing a halsampler log for each.
# checkresult does the analysis afterwards.

set -o pipefail

PITCH=1.5
export PITCH

# case name : program : sweep : planner type
#
# The two "sanity" cases are controls: they run the same sweeps against an
# ordinary G1 move, where the override MUST take effect.  They are what makes
# the threading cases' "no effect" result mean something.
CASES="
sanity-feed:feed-sanity.ngc:feed:0
sanity-adaptive:feed-sanity.ngc:adaptive:0
g33-spindle:thread-g33.ngc:spindle:0
g33-feed:thread-g33.ngc:feed:0
g33-adaptive:thread-g33.ngc:adaptive:0
g76-feed:thread-g76.ngc:feed:0
g76-adaptive:thread-g76.ngc:adaptive:0
scurve-g33-spindle:thread-g33.ngc:spindle:1
scurve-g33-feed:thread-g33.ngc:feed:1
scurve-g33-adaptive:thread-g33.ngc:adaptive:1
scurve-g76-feed:thread-g76.ngc:feed:1
"

rm -f ./*.halsamples ./sim.var ./sampler.ready

run_case() {
    local name="$1" program="$2" sweep="$3" planner="$4"

    echo "I: === case $name (program=$program sweep=$sweep planner=$planner) ==="

    # PLANNER_TYPE lives in the INI, so stamp out a per-case copy rather than
    # mutating the checked-in file.
    sed "s/^PLANNER_TYPE = .*/PLANNER_TYPE = $planner/" \
        threading-sync.ini >| "run-$name.ini"

    local ready="sampler.ready"
    rm -f "$ready"

    PROGRAM="$program" SWEEP="$sweep" SAMPLER_READY="$ready" \
        linuxcnc -r "run-$name.ini" &
    local lcncpid=$!

    # Wait for the sampler component to exist before attaching.  halcmd always
    # exits 0, so test the output rather than the exit status.
    local togo=300
    while [ $togo -gt 0 ] \
      && ! halcmd show pin sampler.0.enable 2>/dev/null | grep -q sampler; do
        sleep 0.1
        togo=$((togo - 1))
    done
    if [ $togo -eq 0 ]; then
        echo "E: sampler component never appeared"
        kill "$lcncpid" 2>/dev/null
        return 1
    fi

    halsampler -t >| "$name.halsamples" &
    local samplerpid=$!

    # The sampler is loaded disabled; tell test-ui.py it may start logging now.
    touch "$ready"

    wait "$lcncpid"
    local rc=$?

    kill "$samplerpid" 2>/dev/null
    wait "$samplerpid" 2>/dev/null

    if [ $rc -ne 0 ]; then
        echo "E: linuxcnc exited $rc for case $name"
        return 1
    fi

    echo "I: captured $(wc -l < "$name.halsamples") samples for $name"
    return 0
}

rc=0
for spec in $CASES; do
    IFS=: read -r name program sweep planner <<< "$spec"
    if ! run_case "$name" "$program" "$sweep" "$planner"; then
        rc=1
        break
    fi
done

exit $rc
