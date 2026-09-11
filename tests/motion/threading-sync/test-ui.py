#!/usr/bin/env python3
#
# Drives a threading pass and disturbs it mid-cut, while threading-sync.hal's
# sampler logs Z against spindle revolutions.  The verification happens in
# checkresult; this script only has to set the machine up, inject the
# disturbance at the right moment, and fail loudly if any of that goes wrong.
#
# Environment:
#   SWEEP    spindle | feed | adaptive | none   (which override to disturb)
#   PROGRAM  the .ngc file to run

import linuxcnc
import linuxcnc_util
import hal
import os
import sys
import time


SWEEP = os.getenv("SWEEP", "none")
PROGRAM = os.getenv("PROGRAM", "thread-g33.ngc")
SAMPLER_READY = os.getenv("SAMPLER_READY", "sampler.ready")

# Applied one at a time, SETTLE seconds apart, starting SETTLE seconds after
# the synchronized segment begins (so the initial sync ramp-up is well past).
SWEEP_STEPS = [0.8, 0.5, 1.0]
SETTLE = 1.0

# Anything slower than this counts as stopped (mm/s); matches checkresult.
MOVING_VEL = 0.1

# How long the disturb marker stays high, in seconds.  Only needs to be wide
# enough that checkresult cannot miss it at the 1 ms sample rate.
MARK_WIDTH = 0.050


def fail(msg):
    print("threading-sync: %s" % msg, file=sys.stderr)
    sys.exit(1)


h = hal.component("test-ui")
h.newpin("disturb", hal.HAL_BIT, hal.HAL_OUT)
h.newpin("sync-active", hal.HAL_BIT, hal.HAL_IN)
h.newpin("path-vel", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("xpos", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("adaptive-feed", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("sample-enable", hal.HAL_BIT, hal.HAL_OUT)
h["adaptive-feed"] = 1.0
h.ready()

if os.system("halcmd source ./postgui.hal") != 0:
    fail("could not source postgui.hal")

c = linuxcnc.command()
s = linuxcnc.stat()
e = linuxcnc.error_channel()
l = linuxcnc_util.LinuxCNC(command=c, status=s, error=e)

l.wait_for_linuxcnc_startup()

c.state(linuxcnc.STATE_ESTOP_RESET)
c.state(linuxcnc.STATE_ON)
c.home(-1)
c.wait_complete()
l.wait_for_home([1, 1, 0, 0, 0, 0, 0, 0, 0])

c.mode(linuxcnc.MODE_AUTO)
c.wait_complete()
c.program_open(os.path.abspath(PROGRAM))

# Handshake with test.sh: start logging only once halsampler is attached, so
# the captured stream contains this run and nothing before it.
deadline = time.time() + 30.0
while not os.path.exists(SAMPLER_READY):
    if time.time() > deadline:
        fail("halsampler never signalled that it was ready")
    time.sleep(0.05)
h["sample-enable"] = 1

c.auto(linuxcnc.AUTO_RUN, 0)


def wait_for_cutting(timeout=60.0):
    """Block until a threading pass is actually under way.

    The M64 marker goes high at the start of the whole cycle, which for G76 is
    several traverses before the first pass.  A pass is identified the same way
    checkresult identifies it - moving, with X held still - so the driver and
    the verifier agree on what they are looking at.
    """
    deadline = time.time() + timeout
    steady_since = None
    x_ref = None
    while time.time() < deadline:
        if h["sync-active"] and abs(h["path-vel"]) > MOVING_VEL:
            if x_ref is None or abs(h["xpos"] - x_ref) > 1e-6:
                x_ref = h["xpos"]
                steady_since = time.time()
            elif time.time() - steady_since >= 0.1:
                return
        else:
            x_ref = None
            steady_since = None
        time.sleep(0.005)
    fail("timed out waiting for a threading pass to start")


def apply_step(scale):
    """Change one override, bracketed by the disturb marker.

    The marker must go high *before* the change is commanded: an override step
    takes effect within a few milliseconds, so marking afterwards would leave
    the new velocity already present in what checkresult treats as the "before"
    window, and the comparison either side of the step would be meaningless.
    """
    if SWEEP == "none":
        return

    h["disturb"] = 1
    time.sleep(0.010)

    if SWEEP == "spindle":
        c.spindleoverride(scale)
    elif SWEEP == "feed":
        c.feedrate(scale)
    elif SWEEP == "adaptive":
        h["adaptive-feed"] = scale

    time.sleep(MARK_WIDTH)
    h["disturb"] = 0


wait_for_cutting()

for scale in SWEEP_STEPS:
    time.sleep(SETTLE)
    if not h["sync-active"]:
        fail("the threading cycle ended before the sweep finished - "
             "the cut is too short or the machine is running too fast")
    apply_step(scale)

# Let the program run to the end so the log covers the whole pass.
l.wait_for_interp_state(linuxcnc.INTERP_IDLE, timeout=60.0)
time.sleep(0.2)
h["sample-enable"] = 0

# Restore, so a leftover override cannot leak into a following test case.
c.feedrate(1.0)
c.spindleoverride(1.0)
h["adaptive-feed"] = 1.0

sys.exit(0)
