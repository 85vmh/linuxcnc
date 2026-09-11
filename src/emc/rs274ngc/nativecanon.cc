/********************************************************************
* Description: nativecanon.cc
*
*   Implementation of NativePreview - see nativecanon.hh.
*
*   The arithmetic here is a port of, and is verified against,
*   lib/python/rs274/interpret.py (Translated) and
*   lib/python/rs274/glcanon.py (GLCanon), plus rs274_arc_to_segments()
*   in gcodemodule.cc.  Where a historical quirk affects the result it is
*   reproduced deliberately and called out in a comment.
*
* License: GPL Version 2
********************************************************************/

#include <algorithm>
#include <cmath>
#include <cstring>

#include "nativecanon.hh"

#ifndef CART_FUZZ
#define CART_FUZZ (0.000001)
#endif

namespace {

/* Direction-only XY rotation: translations do not affect vectors. */
void rotate_dir(double v[3], double c, double s) {
    double tx = v[0] * c - v[1] * s;
    v[1] = v[0] * s + v[1] * c;
    v[0] = tx;
}

void rotate_xy(double &x, double &y, double c, double s) {
    double tx = x * c - y * s;
    y = x * s + y * c;
    x = tx;
}

void unrotate_xy(double &x, double &y, double c, double s) {
    double tx = x * c + y * s;
    y = -x * s + y * c;
    x = tx;
}

} // namespace

NativePreview::NativePreview(const NativeConfig &cfg_in) : config(cfg_in) {
    sources.push_back("");     /* source_id 0: the main program */
    tags.push_back("");        /* tag 0: not inside a subroutine */

    for(int i = 0; i < 3; i++) {
        min_extents[i] = max_extents[i] = 0;
        min_extents_notool[i] = max_extents_notool[i] = 0;
        min_extents_zero_rxy[i] = max_extents_zero_rxy[i] = 0;
        min_extents_notool_zero_rxy[i] = max_extents_notool_zero_rxy[i] = 0;
        acc_min[i] = acc_min_t[i] = 9e99;
        acc_max[i] = acc_max_t[i] = -9e99;
    }
}

int NativePreview::intern(std::vector<std::string> &table, const std::string &s) {
    for(size_t i = 0; i < table.size(); i++)
        if(table[i] == s) return (int)i;
    table.push_back(s);
    return (int)table.size() - 1;
}

/* rs274.interpret.Translated.rotate_and_translate */
void NativePreview::rotate_and_translate(double x, double y, double z,
                                         double a, double b, double c,
                                         double u, double v, double w,
                                         double out[9]) const {
    out[0] = x + g92_offset[0];
    out[1] = y + g92_offset[1];
    out[2] = z + g92_offset[2];
    out[3] = a + g92_offset[3];
    out[4] = b + g92_offset[4];
    out[5] = c + g92_offset[5];
    out[6] = u + g92_offset[6];
    out[7] = v + g92_offset[7];
    out[8] = w + g92_offset[8];

    /* Python guards this with `if self.rotation_xy:`; with a zero rotation the
       multiply is an exact no-op anyway, but keep the guard so the floating
       point path is identical. */
    if(rotation_xy)
        rotate_xy(out[0], out[1], rotation_cos, rotation_sin);

    for(int i = 0; i < 9; i++) out[i] += g5x_offset[i];
}

void NativePreview::note_point(const double p[9], const double to[3]) {
    for(int i = 0; i < 3; i++) {
        acc_max[i] = std::max(acc_max[i], p[i]);
        acc_min[i] = std::min(acc_min[i], p[i]);
        acc_max_t[i] = std::max(acc_max_t[i], p[i] + to[i]);
        acc_min_t[i] = std::min(acc_min_t[i], p[i] + to[i]);
    }
    any_motion = true;
}

void NativePreview::add_line(int kind, const double start[9], const double end[9]) {
    LcncLine l;
    l.seq = seq++;
    l.lineno = lineno;
    l.source_id = source_id;
    l.kind = kind;
    l.tag = 0;
    memcpy(l.start, start, sizeof(l.start));
    memcpy(l.end, end, sizeof(l.end));
    l.feedrate = (kind == LCNC_TRAVERSE) ? 0.0 : feedrate;
    l.tooloffset[0] = tool_offset_now[0];
    l.tooloffset[1] = tool_offset_now[1];
    l.tooloffset[2] = tool_offset_now[2];
    lines.push_back(l);
}

void NativePreview::next_line(const CanonLineState &st) {
    modal = st;
    have_modal = true;
    lineno = st.sequence_number();
    lines_done.store(lineno);
}

void NativePreview::set_g5x_offset(int index, double x, double y, double z,
                                   double a, double b, double c,
                                   double u, double v, double w) {
    g5x_index = index;
    g5x_offset[0] = x; g5x_offset[1] = y; g5x_offset[2] = z;
    g5x_offset[3] = a; g5x_offset[4] = b; g5x_offset[5] = c;
    g5x_offset[6] = u; g5x_offset[7] = v; g5x_offset[8] = w;
}

void NativePreview::set_g92_offset(double x, double y, double z,
                                   double a, double b, double c,
                                   double u, double v, double w) {
    g92_offset[0] = x; g92_offset[1] = y; g92_offset[2] = z;
    g92_offset[3] = a; g92_offset[4] = b; g92_offset[5] = c;
    g92_offset[6] = u; g92_offset[7] = v; g92_offset[8] = w;
}

void NativePreview::set_xy_rotation(double t) {
    rotation_xy = t;
    double r = t * M_PI / 180.0;
    rotation_sin = sin(r);
    rotation_cos = cos(r);
}

void NativePreview::set_plane(int pl) { plane = pl; }

void NativePreview::set_traverse_rate(double /*rate*/) {
    /* No canon class has ever implemented this, and no interpreter call site
       reaches it.  Kept as a no-op so the sink stays complete. */
}

void NativePreview::set_feed_rate(double rate) { feedrate = rate / 60.0; }

/* rs274.interpret.StatMixin.change_tool / get_tool.

   StatMixin keeps `tool_in_spindle` in a module-level global, so it leaks
   between canon instances; here it is per-instance, which is the intended
   meaning. */
void NativePreview::change_tool(int tool) {
    first_move = true;
    tool_list.push_back(tool);

    if(config.tools.empty()) return;
    int idx = 0;
    for(size_t i = 1; i < config.tools.size(); i++) {
        if(config.tools[i].toolno == tool) { idx = (int)i; break; }
    }
    if(config.random_toolchanger) {
        std::swap(config.tools[0], config.tools[idx]);
        tool_in_spindle = idx;
    } else if(idx == 0) {
        config.tools[0] = CANON_TOOL_TABLE{-1, -1, {{0,0,0},0,0,0,0,0,0}, 0,0,0,0, {}};
    } else {
        config.tools[0] = config.tools[idx];
    }
}

CANON_TOOL_TABLE NativePreview::get_tool(int pocket) {
    if(pocket >= 0 && pocket < (int)config.tools.size()) {
        if(pocket == tool_in_spindle) return config.tools[0];
        return config.tools[pocket];
    }
    return CANON_TOOL_TABLE{-1, -1, {{0,0,0},0,0,0,0,0,0}, 0,0,0,0, {}};
}

void NativePreview::get_parameter_file_name(char *name, int max_size) {
    memset(name, 0, max_size);
    strncpy(name, config.parameter_file.c_str(), max_size - 1);
}

/* rs274.glcanon.GLCanon.tool_offset.

   Shifts the current point so the tool tip stays put across a TLO change.
   NOTE: glcanon.py computes the C component as `c - bo + self.bo`, applying
   the B offset twice and never the C offset.  That is a typo; this port does
   the intended `c - co + self.co`, which is the one deliberate behavioural
   difference from the Python path. */
void NativePreview::tool_offset_set(double x, double y, double z,
                                    double a, double b, double c,
                                    double u, double v, double w) {
    first_move = true;
    const double n[9] = {x, y, z, a, b, c, u, v, w};
    for(int i = 0; i < 9; i++)
        lo[i] = lo[i] - n[i] + tool_offset_now[i];
    memcpy(tool_offset_now, n, sizeof(tool_offset_now));
}

void NativePreview::straight_traverse(double x, double y, double z,
                                      double a, double b, double c,
                                      double u, double v, double w) {
    if(suppress > 0) return;
    double l[9];
    rotate_and_translate(x, y, z, a, b, c, u, v, w, l);
    if(!first_move) add_line(LCNC_TRAVERSE, lo, l);
    memcpy(lo, l, sizeof(lo));
}

void NativePreview::straight_feed(double x, double y, double z,
                                  double a, double b, double c,
                                  double u, double v, double w) {
    if(suppress > 0) return;
    first_move = false;
    double l[9];
    rotate_and_translate(x, y, z, a, b, c, u, v, w, l);
    add_line(LCNC_FEED, lo, l);
    memcpy(lo, l, sizeof(lo));
}

void NativePreview::straight_probe(double x, double y, double z,
                                   double a, double b, double c,
                                   double u, double v, double w) {
    if(suppress > 0) return;
    first_move = false;
    double l[9];
    rotate_and_translate(x, y, z, a, b, c, u, v, w, l);
    add_line(LCNC_PROBE, lo, l);
    memcpy(lo, l, sizeof(lo));
}

/* GLCanon.rigid_tap emits the plunge and the retract as two feed segments and
   leaves the current point where it started. */
void NativePreview::rigid_tap(double x, double y, double z) {
    if(suppress > 0) return;
    first_move = false;
    double l[9];
    rotate_and_translate(x, y, z, 0, 0, 0, 0, 0, 0, l);
    for(int i = 3; i < 9; i++) l[i] = lo[i];
    add_line(LCNC_RIGID_TAP, lo, l);
    add_line(LCNC_RIGID_TAP, l, lo);
}

void NativePreview::arc_feed(double first_end, double second_end,
                             double first_axis, double second_axis,
                             int rotation, double axis_end_point,
                             double a, double b, double c,
                             double u, double v, double w) {
    if(suppress > 0) return;
    first_move = false;

    /* Port of rs274_arc_to_segments(), with the canon state read from members
       instead of pulled back out of a Python object. */
    int X, Y, Z;
    if(plane == 1)      { X = 0; Y = 1; Z = 2; }
    else if(plane == 3) { X = 2; Y = 0; Z = 1; }
    else                { X = 1; Y = 2; Z = 0; }

    double cx = first_axis, cy = second_axis;
    double o[9], n[9];
    memcpy(o, lo, sizeof(o));
    n[X] = first_end; n[Y] = second_end; n[Z] = axis_end_point;
    n[3] = a; n[4] = b; n[5] = c; n[6] = u; n[7] = v; n[8] = w;

    for(int ax = 0; ax < 9; ax++) o[ax] -= g5x_offset[ax];
    unrotate_xy(o[0], o[1], rotation_cos, rotation_sin);
    for(int ax = 0; ax < 9; ax++) o[ax] -= g92_offset[ax];

    double theta1 = atan2(o[Y] - cy, o[X] - cx);
    double theta2 = atan2(n[Y] - cy, n[X] - cx);

    /* Issue #1528: a chord shorter than CART_FUZZ means a full turn. */
    double len = hypot(o[X] - n[X], o[Y] - n[Y]) * (25.4 * config.linear_units);
    if(rotation < 0) {
        if(theta1 < theta2) theta2 -= 2 * M_PI;
        if(len < CART_FUZZ) theta2 -= 2 * M_PI;
    } else {
        if(theta1 > theta2) theta2 += 2 * M_PI;
        if(len < CART_FUZZ) theta2 += 2 * M_PI;
    }
    if(rotation < -1) theta2 += 2 * M_PI * (rotation + 1);
    if(rotation >  1) theta2 += 2 * M_PI * (rotation - 1);

    int max_segments = config.arcdivision;
    int steps = std::max(3, int(max_segments * fabs(theta1 - theta2) / M_PI));
    double rsteps = 1.0 / steps;
    double dtheta = theta2 - theta1;

    double d[9] = {0, 0, 0, n[3]-o[3], n[4]-o[4], n[5]-o[5],
                            n[6]-o[6], n[7]-o[7], n[8]-o[8]};
    d[Z] = n[Z] - o[Z];

    /* ---- the primitive, in the same space as start[]/end[] ---- */
    LcncArc arc;
    arc.seq = seq;                 /* shares the counter with its segments */
    arc.lineno = lineno;
    arc.source_id = source_id;
    arc.plane = plane;
    arc.tag = 0;
    arc.radius = hypot(o[X] - cx, o[Y] - cy);
    arc.start_angle = theta1;
    arc.end_angle = theta2;
    arc.helix_delta = d[Z];
    arc.feedrate = feedrate;
    arc.tooloffset[0] = tool_offset_now[0];
    arc.tooloffset[1] = tool_offset_now[1];
    arc.tooloffset[2] = tool_offset_now[2];
    memcpy(arc.start, lo, sizeof(arc.start));

    /* centre: a point, so it takes the full affine transform */
    {
        double cpt[9] = {0};
        cpt[X] = cx; cpt[Y] = cy; cpt[Z] = o[Z];
        for(int ax = 0; ax < 9; ax++) cpt[ax] += g92_offset[ax];
        rotate_xy(cpt[0], cpt[1], rotation_cos, rotation_sin);
        for(int ax = 0; ax < 9; ax++) cpt[ax] += g5x_offset[ax];
        arc.center[0] = cpt[0]; arc.center[1] = cpt[1]; arc.center[2] = cpt[2];
    }
    /* basis: directions, so only the rotation applies */
    {
        double ua[3] = {0,0,0}, va[3] = {0,0,0}, na[3] = {0,0,0};
        if(X < 3) ua[X] = 1.0;
        if(Y < 3) va[Y] = 1.0;
        if(Z < 3) na[Z] = 1.0;
        rotate_dir(ua, rotation_cos, rotation_sin);
        rotate_dir(va, rotation_cos, rotation_sin);
        rotate_dir(na, rotation_cos, rotation_sin);
        memcpy(arc.u_axis, ua, sizeof(ua));
        memcpy(arc.v_axis, va, sizeof(va));
        memcpy(arc.n_axis, na, sizeof(na));
    }

    /* ---- tessellation, bit for bit as rs274_arc_to_segments did it ----
       The stepper rotates an offset vector incrementally rather than calling
       cos/sin per step; that accumulates a little drift, which the original
       hides by writing the final point exactly.  Reproduced so the segment
       stream, and therefore the extents, match the Python path exactly. */
    int arc_index = (int)arcs.size();
    double cur[9];
    memcpy(cur, lo, sizeof(cur));

    double tx = o[X] - cx, ty = o[Y] - cy;
    double dc = cos(dtheta * rsteps), ds = sin(dtheta * rsteps);
    for(int i = 0; i < steps - 1; i++) {
        double f = (i + 1) * rsteps;
        double p[9];
        rotate_xy(tx, ty, dc, ds);
        p[X] = tx + cx;
        p[Y] = ty + cy;
        p[Z] = o[Z] + d[Z] * f;
        for(int ax = 3; ax < 9; ax++) p[ax] = o[ax] + d[ax] * f;
        for(int ax = 0; ax < 9; ax++) p[ax] += g92_offset[ax];
        rotate_xy(p[0], p[1], rotation_cos, rotation_sin);
        for(int ax = 0; ax < 9; ax++) p[ax] += g5x_offset[ax];
        add_line(LCNC_FEED, cur, p);
        lines.back().arc_index = arc_index;
        memcpy(cur, p, sizeof(cur));
    }
    for(int ax = 0; ax < 9; ax++) n[ax] += g92_offset[ax];
    rotate_xy(n[0], n[1], rotation_cos, rotation_sin);
    for(int ax = 0; ax < 9; ax++) n[ax] += g5x_offset[ax];
    add_line(LCNC_FEED, cur, n);
    lines.back().arc_index = arc_index;

    memcpy(arc.end, n, sizeof(arc.end));
    arcs.push_back(arc);

    memcpy(lo, n, sizeof(lo));
}

void NativePreview::add_dwell(int kind) {
    LcncDwell d;
    d.seq = seq++;
    d.lineno = lineno;
    d.source_id = source_id;
    d.kind = kind;
    d.tag = 0;
    d.pos[0] = lo[0]; d.pos[1] = lo[1]; d.pos[2] = lo[2];
    /* GLCanon derives the axis from the active plane: G17/18/19 report as
       170/180/190, so plane/10 - 17 gives 0/1/2. */
    d.plane_axis = have_modal ? (int)(modal.gcodes[3] / 10 - 17) : 0;
    dwells.push_back(d);
}

void NativePreview::dwell(double time) {
    if(suppress > 0) return;
    dwell_time += time;
    add_dwell(LCNC_DWELL_G4);
}

void NativePreview::user_defined_function(int /*num*/, double /*p*/, double /*q*/) {
    if(suppress > 0) return;
    add_dwell(LCNC_DWELL_M1XX);
}

void NativePreview::message(const char * /*s*/) {
    /* GLCanon.message is a no-op. */
}

/* rs274.glcanon.GLCanon.comment - the (AXIS,...) / (PREVIEW,...) vocabulary. */
void NativePreview::comment(const char *s) {
    std::string arg(s ? s : "");
    if(arg.rfind("AXIS,", 0) != 0 && arg.rfind("PREVIEW,", 0) != 0) return;

    std::vector<std::string> parts;
    size_t pos = 0;
    while(true) {
        size_t comma = arg.find(',', pos);
        if(comma == std::string::npos) { parts.push_back(arg.substr(pos)); break; }
        parts.push_back(arg.substr(pos, comma - pos));
        pos = comma + 1;
    }
    if(parts.size() < 2) return;
    const std::string &cmd = parts[1];

    if(cmd == "stop") {
        abort_requested.store(true);
        return;
    }
    if(cmd == "hide") { suppress += 1; return; }
    if(cmd == "show") { suppress -= 1; return; }

    /* Foam positions arrive in program units, so convert when the program is
       in G21 - matching the `if 210 in self.state.gcodes` test in glcanon.py.
       gcodes[5] carries the units code. */
    bool metric_program = have_modal && modal.gcodes[5] == 210;
    if(cmd == "XY_Z_POS") {
        if(parts.size() > 2) {
            try {
                double val = std::stod(parts[2]);
                config.foam_z = metric_program ? val / 25.4 : val;
            } catch(...) {
                config.foam_z = 5.0 / 25.4;
            }
        }
        return;
    }
    if(cmd == "UV_Z_POS") {
        if(parts.size() > 2) {
            try {
                double val = std::stod(parts[2]);
                config.foam_w = metric_program ? val / 25.4 : val;
            } catch(...) {
                config.foam_w = 30.0;
            }
        }
        return;
    }
    if(cmd == "notify") {
        notify_count += 1;
        notify_message = "(AXIS,notify):" + std::to_string(notify_count);
        if(parts.size() > 2 && !parts[2].empty()) notify_message = parts[2];
        return;
    }
}

/* gcode.calc_extents folds in the start point of every item, plus the end
   point of the last item of each sequence it is handed.  GLCanon.calc_extents
   passes (arcfeed, feed, traverse), so exactly three end points participate.
   Reproduce that rather than folding in every end point, which would be a
   superset wherever the path is discontinuous. */
void NativePreview::finish() {
    if(lines.empty() && dwells.empty() && arcs.empty()) return;

    bool have_last[3] = {false, false, false};
    double last_end[3][3] = {{0}}, last_to[3][3] = {{0}};

    for(size_t i = 0; i < lines.size(); i++) {
        const LcncLine &l = lines[i];
        note_point(l.start, l.tooloffset);
        int cat = (l.kind == LCNC_TRAVERSE) ? 2 : (l.arc_index >= 0 ? 0 : 1);
        have_last[cat] = true;
        for(int k = 0; k < 3; k++) {
            last_end[cat][k] = l.end[k];
            last_to[cat][k] = l.tooloffset[k];
        }
    }
    for(int cat = 0; cat < 3; cat++) {
        if(!have_last[cat]) continue;
        for(int k = 0; k < 3; k++) {
            acc_max[k] = std::max(acc_max[k], last_end[cat][k]);
            acc_min[k] = std::min(acc_min[k], last_end[cat][k]);
            acc_max_t[k] = std::max(acc_max_t[k], last_end[cat][k] + last_to[cat][k]);
            acc_min_t[k] = std::min(acc_min_t[k], last_end[cat][k] + last_to[cat][k]);
        }
    }

    if(!any_motion) return;   /* dwells only: keep the zeroed extents */

    for(int k = 0; k < 3; k++) {
        min_extents[k] = acc_min[k];
        max_extents[k] = acc_max[k];
        min_extents_notool[k] = acc_min_t[k];
        max_extents_notool[k] = acc_max_t[k];
    }

    compute_zero_rxy_extents();

    if(config.is_foam) {
        double zmin = std::min(config.foam_z, config.foam_w);
        double zmax = std::max(config.foam_z, config.foam_w);
        min_extents[2] = zmin;
        max_extents[2] = zmax;
        min_extents_notool[2] = zmin;
        max_extents_notool[2] = zmax;
    }
}

/* GLCanon.unrotate_preview rotates the whole preview by -rotation_xy about the
   G5x origin, using the FINAL rotation and offset values rather than the ones
   in effect when each move was emitted, then measures that.  It appends feed
   and arcfeed first and traverse last into one list, so only the very last
   item contributes an end point. */
void NativePreview::compute_zero_rxy_extents() {
    double angle = -rotation_xy * M_PI / 180.0;
    double cs = cos(angle), sn = sin(angle);
    double ox = g5x_offset[0], oy = g5x_offset[1];

    double mn[3] = {9e99, 9e99, 9e99}, mx[3] = {-9e99, -9e99, -9e99};
    double mnt[3] = {9e99, 9e99, 9e99}, mxt[3] = {-9e99, -9e99, -9e99};

    bool have_last = false;
    double last_end[3] = {0}, last_to[3] = {0};

    auto unrotate_point = [&](const double p[3], double out[3]) {
        double tx = p[0] - ox, ty = p[1] - oy;
        out[0] = (tx * cs) - (ty * sn) + ox;
        out[1] = (tx * sn) + (ty * cs) + oy;
        out[2] = p[2];
    };

    /* pass 1: feed and arcfeed, in emission order; pass 2: traverse */
    for(int pass = 0; pass < 2; pass++) {
        for(size_t i = 0; i < lines.size(); i++) {
            const LcncLine &l = lines[i];
            bool is_traverse = (l.kind == LCNC_TRAVERSE);
            if((pass == 0) == is_traverse) continue;
            double s[3];
            unrotate_point(l.start, s);
            for(int k = 0; k < 3; k++) {
                mx[k] = std::max(mx[k], s[k]);
                mn[k] = std::min(mn[k], s[k]);
                mxt[k] = std::max(mxt[k], s[k] + l.tooloffset[k]);
                mnt[k] = std::min(mnt[k], s[k] + l.tooloffset[k]);
            }
            unrotate_point(l.end, last_end);
            for(int k = 0; k < 3; k++) last_to[k] = l.tooloffset[k];
            have_last = true;
        }
    }
    if(have_last) {
        for(int k = 0; k < 3; k++) {
            mx[k] = std::max(mx[k], last_end[k]);
            mn[k] = std::min(mn[k], last_end[k]);
            mxt[k] = std::max(mxt[k], last_end[k] + last_to[k]);
            mnt[k] = std::min(mnt[k], last_end[k] + last_to[k]);
        }
    }

    for(int k = 0; k < 3; k++) {
        min_extents_zero_rxy[k] = mn[k];
        max_extents_zero_rxy[k] = mx[k];
        min_extents_notool_zero_rxy[k] = mnt[k];
        max_extents_notool_zero_rxy[k] = mxt[k];
    }
}
