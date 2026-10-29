#!/usr/bin/env python3
"""What gcode.parse does at an M6 for a program simulator, as task and
iocontrol do it on a machine.

With no LinuxCNC running and a private tool table (tools.tbl: T1 in P1 with
X/Z offsets, T2 in P2 with X/Y/Z), every program is parsed with a canon that
records each callback as ``(name, args...)``. Lengths arrive in inches and
are compared here in millimetres: the canon's external length units are 1.0,
so machine units, the tool table and TOOL_CHANGE_POSITION are millimetres,
and every program is G21.
"""
import os
import sys
import tempfile

import gcode

HERE = os.path.dirname(os.path.abspath(__file__))
MM = 25.4
EPS = 1e-6

# What a canon written before any of this defines, and the events added
# since: the simulator callbacks and the M6 ones.
OLD_EVENTS = ("straight_feed", "straight_traverse", "straight_probe",
              "arc_feed", "rigid_tap", "dwell", "user_defined_function",
              "change_tool", "tool_offset", "set_g5x_offset",
              "set_g92_offset", "set_xy_rotation", "set_g68_frame",
              "set_plane", "set_feed_rate", "set_traverse_rate", "comment",
              "message")
NEW_EVENTS = ("program_stop", "optional_program_stop", "program_end",
              "spindle_on", "spindle_off", "set_spindle_speed",
              "set_spindle_mode", "set_feed_mode", "set_spindle_sync",
              "tool_change_move", "set_position")
# Events whose nine arguments are a point or offsets, compared in mm.
POINTS = {"straight_feed", "straight_traverse", "tool_change_move",
          "set_position", "tool_offset"}


def _recorder(name):
    def method(self, *args):
        if name in POINTS:
            args = tuple(a * MM for a in args)
        self.calls.append((name,) + args)
    method.__name__ = name
    return method


class Canon:
    """The getters every canon needs, minus get_tool: the private table
    answers for a canon without it."""

    def __init__(self):
        self.calls = []

    def next_line(self, linecode):
        self.calls.append(("next_line", linecode.sequence_number))

    def get_external_length_units(self): return 1.0
    def get_external_angular_units(self): return 1.0
    def get_axis_mask(self): return 0x1ff
    def get_block_delete(self): return False
    def check_abort(self): return False


for _name in OLD_EVENTS:
    setattr(Canon, _name, _recorder(_name))


class LegacyCanon(Canon):
    """A canon from before: the old events and its own get_tool."""

    def get_tool(self, pocket):
        return -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0


class SimCanon(Canon):
    """Every new event, source_line, and no get_tool."""

    def source_line(self, filename, line, call_level):
        self.calls.append(("source_line", filename, line, call_level))


for _name in NEW_EVENTS:
    setattr(SimCanon, _name, _recorder(_name))


class PositionCanon(SimCanon):
    """A simulator answering the iocontrol tool-change-pos-* pins."""

    position = (30.0, 0.0, 40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def get_tool_change_position(self):
        return self.position


def parse(program, canon, ini=None):
    """Parse `program` (lines) with `canon`; (result, program file)."""
    if ini:
        os.environ["INI_FILE_NAME"] = os.path.join(HERE, ini)
    else:
        os.environ.pop("INI_FILE_NAME", None)
    with tempfile.NamedTemporaryFile("w", suffix=".ngc", delete=False) as f:
        f.write("\n".join(program) + "\n")
        name = f.name
    # a fresh parameter file each time: the interpreter saves the work
    # offsets into it, and one program's must not be the next one's start
    parameter = tempfile.NamedTemporaryFile(suffix=".var")
    canon.parameter_file = parameter.name
    cwd = os.getcwd()
    try:
        os.chdir(HERE)            # the INIs' SUBROUTINE_PATH is relative
        result, _ = gcode.parse(name, canon, "", "", "")
    finally:
        os.chdir(cwd)
        os.unlink(name)
        parameter.close()
    return result, os.path.realpath(name)


FAILURES = []


def check(what, cond, detail=""):
    print(("ok   " if cond else "FAIL ") + what
          + (": %s" % (detail,) if detail != "" and not cond else ""))
    if not cond:
        FAILURES.append(what)


def parsed(what, result):
    check(what + " parses", result <= gcode.MIN_ERROR,
          gcode.strerror(result) if result > gcode.MIN_ERROR else "")


def near(args, want):
    want = tuple(want) + (0.0,) * (9 - len(want))
    return len(args) == 9 and all(abs(a - w) < EPS for a, w in zip(args, want))


def with_lines(calls):
    """(line, call) for every call but next_line/source_line."""
    line, out = None, []
    for c in calls:
        if c[0] == "next_line":
            line = c[1]
        elif c[0] != "source_line":
            out.append((line, c))
    return out


def events(calls, name):
    return [(line, c[1:]) for line, c in with_lines(calls) if c[0] == name]


MOVES = ("straight_feed", "straight_traverse", "arc_feed", "tool_change_move")


# --- a canon without get_tool and no table --------------------------------

def test_missing_get_tool():
    canon = Canon()
    try:
        parse(["G0 X1", "M2"], canon)
    except AttributeError as e:
        check("no get_tool and no table: AttributeError naming get_tool",
              "get_tool" in str(e), e)
    except Exception as e:
        check("no get_tool and no table: AttributeError naming get_tool",
              False, repr(e))
    else:
        check("no get_tool and no table: AttributeError naming get_tool",
              False, "no exception")


# --- two tool changes to TOOL_CHANGE_POSITION -----------------------------

# Each program starts with G49 and its first move names every axis: a tool
# offset stays in effect from one program to the next, as on a machine, and
# G49 leaves the tip where that offset had it.
TWO_CHANGES = [
    "G21 G90 G49",                  # 1
    "G0 X5 Y0 Z5",                  # 2
    "T1 M6",                        # 3
    "(DEBUG,#5420 #5421 #5422)",    # 4
    "G43",                          # 5
    "G0 X1 Z1",                     # 6
    "T2 M6",                        # 7
    "G91 G0 X1",                    # 8
    "G90 G43",                      # 9
    "G0 X2",                        # 10
    "M2",                           # 11
]


def check_two_changes(calls, first, second, label):
    """The events of TWO_CHANGES with the change position `first` (machine,
    mm) for T1 and `second` for T2, both before the new tool's offset."""
    moves = events(calls, "tool_change_move")
    check(label + ": a tool_change_move at each M6, before change_tool",
          [l for l, _ in moves] == [3, 7]
          and near(moves[0][1], first) and near(moves[1][1], second), moves)
    order = [c[0] for _, c in with_lines(calls) if c[0] in
             ("tool_change_move", "change_tool")]
    check(label + ": ... each before its change_tool",
          order == ["tool_change_move", "change_tool"] * 2, order)


def test_two_changes():
    canon = SimCanon()
    result, _ = parse(TWO_CHANGES, canon, "tcp.ini")
    parsed("two tool changes", result)
    c = canon.calls
    # T1's change from no tool: the INI's point as it is. T2's with T1's
    # offset still in effect: the same machine point is (10-0.5, 0, 20-1).
    check_two_changes(c, (10, 0, 20), (9.5, 0, 19), "INI position")

    msg = events(c, "message")
    check("#5420-#5422 right after the M6 are the tool change position",
          msg and msg[0][0] == 4
          and [float(v) for v in msg[0][1][0].split()] == [10, 0, 20], msg)

    offsets = [o for o in events(c, "tool_offset") if o[0] != 1]   # G49
    check("tool_offset with T1's then T2's offsets from the table",
          [l for l, _ in offsets] == [5, 9]
          and near(offsets[0][1], (0.5, 0, 1))
          and near(offsets[1][1], (-0.25, 0.75, 2)), offsets)

    sets = events(c, "set_position")
    # After line 8 the machine is at (10.5+0.5, 0, 19+1); T2's tip is there
    # less T2's offset.
    check("set_position after each G43 at the new tool's tip",
          [l for l, _ in sets] == [5, 9]
          and near(sets[0][1], (9.5, 0, 19))
          and near(sets[1][1], (11.25, -0.75, 18)), sets)

    seq = [(l, x[0]) for l, x in with_lines(c)
           if x[0] in MOVES or x[0] == "set_position"]
    i = seq.index((5, "set_position"))
    check("the first move after the G43 follows its set_position directly",
          seq[i + 1] == (6, "straight_traverse"), seq)

    trav = events(c, "straight_traverse")
    g91 = [p for l, p in trav if l == 8]
    check("G91 G0 X1 after the M6 moves 1 from the tool change position",
          g91 and near(g91[0], (10.5, 0, 19)), trav)
    g0 = [p for l, p in trav if l == 10]
    check("G0 X2 after the second G43 starts from its set_position",
          g0 and near(g0[0], (2, -0.75, 18)), trav)


def test_canon_position():
    canon = PositionCanon()
    result, _ = parse(TWO_CHANGES, canon, "tcp.ini")
    parsed("get_tool_change_position", result)
    check_two_changes(canon.calls, (30, 0, 40), (29.5, 0, 39),
                      "get_tool_change_position wins over the INI")

    canon = PositionCanon()
    canon.position = None
    result, _ = parse(TWO_CHANGES, canon, "tcp.ini")
    parsed("get_tool_change_position None", result)
    check_two_changes(canon.calls, (10, 0, 20), (9.5, 0, 19),
                      "get_tool_change_position None falls back to the INI")

    canon = PositionCanon()
    result, _ = parse(TWO_CHANGES, canon, "notcp.ini")
    parsed("get_tool_change_position, no INI position", result)
    check_two_changes(canon.calls, (30, 0, 40), (29.5, 0, 39),
                      "get_tool_change_position without an INI position")


def test_no_position():
    canon = SimCanon()
    result, _ = parse(TWO_CHANGES, canon, "notcp.ini")
    parsed("no tool change position", result)
    c = canon.calls
    check("no TOOL_CHANGE_POSITION, no get_tool_change_position: no move",
          not events(c, "tool_change_move"), events(c, "tool_change_move"))
    trav = events(c, "straight_traverse")
    # line 6 left the tip at (1, 0, 1) under T1, the machine at (1.5, 0, 2);
    # G91 X1 from there
    g91 = [p for l, p in trav if l == 8]
    check("... and G91 G0 X1 continues from the last move",
          g91 and near(g91[0], (2, 0, 1)), trav)


# --- a random toolchanger: the spindle and the pocket swap ----------------

def test_random():
    gcode.load_tool_table(os.path.join(HERE, "tools.tbl"), True)
    try:
        canon = SimCanon()
        result, _ = parse(["G21 G49", "T1 M6", "T2",
                           "(DEBUG,#<_selected_pocket> #<_current_pocket> #5400)",
                           "M6", "T1",
                           "(DEBUG,#<_selected_pocket> #<_current_pocket> #5400)",
                           "M6 G43", "(DEBUG,#5400)", "M2"], canon, "random.ini")
        parsed("random toolchanger", result)
        msg = [[float(v) for v in m[0].split()]
               for _, m in events(canon.calls, "message")]
        check("random: T2 waits in P2 while T1 is in the spindle from P1",
              msg and msg[0] == [2, 1, 1], msg)
        check("random: after T2's change T1 is in P2, where T2 was",
              len(msg) > 1 and msg[1] == [2, 2, 2], msg)
        check("random: T1 back in the spindle, its offsets with it",
              len(msg) > 2 and msg[2] == [1]
              and near(events(canon.calls, "tool_offset")[-1][1], (0.5, 0, 1)),
              (msg, events(canon.calls, "tool_offset")))
    finally:
        gcode.load_tool_table(os.path.join(HERE, "tools.tbl"))


# --- an M6 remap that calls the built-in M6 and then G43 ------------------

def test_remap():
    canon = SimCanon()
    program = ["G21 G90 G49", "G0 X5 Y0 Z5", "T1 M6", "(DEBUG,#5420 #5421 #5422)",
               "G0 X1 Z1", "T2 M6", "G91 G0 X1", "M2"]
    result, main = parse(program, canon, "remap.ini")
    parsed("M6 remap", result)
    c = canon.calls
    sub = os.path.join(HERE, "subs", "m6_remap.ngc")
    moves = events(c, "tool_change_move")
    check("remap: a tool_change_move at each built-in M6",
          len(moves) == 2 and near(moves[0][1], (10, 0, 20))
          and near(moves[1][1], (9.5, 0, 19)), moves)
    sets = events(c, "set_position")
    check("remap: set_position after each G43 in the sub",
          len(sets) == 2 and near(sets[0][1], (9.5, 0, 19))
          and near(sets[1][1], (10.25, -0.75, 18)), sets)
    offsets = events(c, "tool_offset")[1:]                   # after the G49
    check("remap: tool_offset with T1's then T2's offsets",
          len(offsets) == 2 and near(offsets[0][1], (0.5, 0, 1))
          and near(offsets[1][1], (-0.25, 0.75, 2)), offsets)

    # every next_line is followed by its source_line
    pairs = [(a[1], b) for a, b in zip(c, c[1:]) if a[0] == "next_line"]
    check("source_line right after every next_line",
          all(b[0] == "source_line" and b[2] == line for line, b in pairs),
          pairs)
    where = {}
    for name, (line, source) in ((x[0], (x[2], x)) for x in c
                                 if x[0] == "source_line"):
        where.setdefault((source[1], source[3]), set()).add(line)
    check("remap: its lines come from the remap file at call level 1",
          where.get((sub, 1)) == {2, 3}, where)
    check("remap: the main program's lines at call level 0",
          {2, 4, 5, 7, 8} <= where.get((main, 0), set()), where)
    # the events of the sub are on the sub's lines (the G49 is the main
    # program's line 1)
    on_sub = []
    file = None
    for x in c:
        if x[0] == "source_line":
            file = x[1]
        elif x[0] in ("tool_change_move", "change_tool", "tool_offset") \
                and file != main:
            on_sub.append(file == sub)
    check("remap: the M6 and G43 events arrive on the sub's lines",
          on_sub and all(on_sub), on_sub)


# --- work offsets ---------------------------------------------------------

def test_work_offsets():
    canon = SimCanon()
    result, _ = parse([
        "G21 G90 G49",                  # 1
        "G10 L2 P2 X10 Y0 Z5",          # 2  G55, not active: nothing moves
        "G0 X1 Y1 Z1",                  # 3
        "G55",                          # 4  program frame shifts
        "G0 X2",                        # 5
        "G10 L2 P2 X20",                # 6  the active system changes
        "G0 Y3",                        # 7
        "G10 L2 P1 X7",                 # 8  G54, inactive: nothing moves
        "G55",                          # 9  already active: same point
        "G92 X0",                       # 10
        "G0 Z0",                        # 11
        "M2"], canon, "notcp.ini")
    parsed("work offsets", result)
    sets = events(canon.calls, "set_position")
    # line 6: the machine is at X12, G55 now X20; line 12: M2 goes back to
    # G54 (X7) and keeps the G92, so X 12 - 7 + 8
    want = [(4, (-9, 1, -4)), (6, (-8, 1, -4)), (10, (0, 3, -4)),
            (12, (13, 3, 5))]
    check("set_position after G55, G10 L2 on the active system, G92 and the "
          "M2 reset, and not otherwise",
          [l for l, _ in sets] == [l for l, _ in want]
          and all(near(p, w) for (_, p), (_, w) in zip(sets, want)), sets)
    trav = events(canon.calls, "straight_traverse")
    check("the moves after them start where set_position said",
          [(l, [round(v, 6) for v in p[:3]]) for l, p in trav]
          == [(3, [1, 1, 1]), (5, [2, 1, -4]), (7, [-8, 3, -4]),
              (11, [0, 3, 0])], trav)


# --- canons without the new methods ---------------------------------------

def test_legacy_canon():
    """A canon from before gets the same sequence whether or not there is a
    tool change position: no move, no set_position, no source_line."""
    traces = []
    for ini in ("tcp.ini", "notcp.ini"):
        canon = LegacyCanon()
        result, _ = parse(TWO_CHANGES, canon, ini)
        parsed("legacy canon (%s)" % ini, result)
        traces.append(canon.calls)
    check("legacy canon: the same sequence with and without a position",
          traces[0] == traces[1])
    names = {x[0] for x in traces[0]}
    check("legacy canon: none of the new events",
          not names & (set(NEW_EVENTS) | {"source_line"}), names)


# --- the renderer draws the move ------------------------------------------

def test_renderer():
    try:
        sys.path.insert(0, os.path.join(HERE, "..", "gcode-renderer"))
        import canon as harness
        import rs274.glcanon        # noqa: F401 - PyOpenGL, as the harness
        from rs274 import glcanon_bake as bake
    except ImportError as e:
        print("skip renderer check: %s" % e)
        return
    program = "G21 G90\nG1 X1 F100\nG0 X5 Z5\nM6\nG91 G0 X1\nG1 X1\nM2\n"
    got = {}
    for ini in ("tcp.ini", "notcp.ini"):
        os.environ["INI_FILE_NAME"] = os.path.join(HERE, ini)
        pg = harness.parse(program).program_geometry
        got[ini] = ([int(k) for k in pg.kinds],
                    [[round(v * MM, 6) for v in p] for p in pg.positions()])
    os.environ.pop("INI_FILE_NAME", None)
    kinds, pos = got["tcp.ini"]
    i = kinds.index(bake.KIND_TOOLCHANGE)
    check("renderer: the move to the tool change position is a traverse",
          kinds[i - 1] == bake.KIND_TRAVERSE and pos[i - 1] == [10, 0, 20],
          got["tcp.ini"])
    check("renderer: and the moves after the M6 start from there",
          pos[i + 1] == [11, 0, 20], got["tcp.ini"])
    kinds, pos = got["notcp.ini"]
    check("renderer: no position, no move",
          kinds.count(bake.KIND_TRAVERSE) == 1 and pos[-1] == [7, 0, 5],
          got["notcp.ini"])


def main():
    # before any table: this process has none to fall back on
    test_missing_get_tool()

    gcode.load_tool_table(os.path.join(HERE, "tools.tbl"))
    test_two_changes()
    test_canon_position()
    test_no_position()
    test_random()
    test_remap()
    test_work_offsets()
    test_legacy_canon()
    test_renderer()

    if FAILURES:
        print("%d check(s) failed" % len(FAILURES))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
