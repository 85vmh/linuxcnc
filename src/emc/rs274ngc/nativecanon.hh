/********************************************************************
* Description: nativecanon.hh
*
*   A canon sink that accumulates the preview toolpath into plain C++
*   vectors instead of calling back into Python.
*
*   The geometry that rs274.interpret.Translated and rs274.glcanon.GLCanon
*   compute today - G5x/G92 offsets, XY rotation, segment accumulation,
*   extents, the (AXIS,...) comment vocabulary and the tool table lookup -
*   lives here instead, so the preview engine no longer needs a live
*   CPython interpreter to produce a toolpath.
*
*   All lengths are inches, matching the historical convention: the canon
*   entry points in gcodemodule.cc already divide by 25.4 when the program
*   is in G21.
*
* License: GPL Version 2
********************************************************************/
#ifndef NATIVECANON_HH
#define NATIVECANON_HH

#include <array>
#include <atomic>
#include <map>
#include <string>
#include <vector>
#include <cstdint>

#include "canonsink.hh"

/* Bumped whenever the layout of the structs below changes.  Consumers that
   reach the vectors through a raw pointer must check it. */
#define LCNC_PREVIEW_ABI 1

enum LcncLineKind {
    LCNC_TRAVERSE  = 0,
    LCNC_FEED      = 1,
    LCNC_PROBE     = 2,
    LCNC_RIGID_TAP = 3,
};

enum LcncSync {
    LCNC_SYNC_NONE     = 0,
    LCNC_SYNC_POSITION = 1,   /* G33 / G33.1 / G76 - locked to spindle position */
};

enum LcncDwellKind {
    LCNC_DWELL_G4   = 0,
    LCNC_DWELL_M1XX = 1,
};

/* Spindle synchronisation state carried by a motion. */
struct LcncSyncInfo {
    int32_t mode = LCNC_SYNC_NONE;
    int32_t spindle = 0;
    double  pitch = 0;         /* per revolution */
    double  angle = 0;         /* angular offset, degrees */
};

struct LcncLine {
    uint32_t seq = 0;          /* monotonic, shared across lines/arcs/dwells */
    int32_t  lineno = -1;      /* line within source_id, not a global number */
    int32_t  source_id = 0;    /* index into NativePreview::sources */
    int32_t  kind = LCNC_FEED;
    int32_t  tag = 0;          /* index into NativePreview::tags */
    /* >= 0 when this segment is one step of arcs[arc_index].  A consumer that
       renders arcs from the primitive skips these; one that wants plain
       polylines uses them and ignores arcs[].  The tessellation is kept
       because it is what the historical extents and the Python compatibility
       layer are defined in terms of, down to the floating point. */
    int32_t  arc_index = -1;
    double   start[9] = {0};   /* X Y Z A B C U V W */
    double   end[9] = {0};
    double   feedrate = 0;     /* 0 for traverse */
    double   tooloffset[3] = {0};
    LcncSyncInfo sync;
};

/* An arc is kept as a primitive so a consumer can tessellate it at whatever
   tolerance its zoom level needs.

       P(t) = center + r*cos(theta)*u_axis + r*sin(theta)*v_axis
                     + helix_delta*t*n_axis
       theta = start_angle + t*(end_angle - start_angle),  t in [0,1]

   and axes 3..8 interpolate linearly from start[] to end[].

   start_angle/end_angle are already resolved: direction, multi-turn and the
   full-circle case are folded in, so end_angle may differ from start_angle by
   more than 2*pi and the consumer only has to interpolate.

   u/v/n_axis are unit vectors in the same space as start[]/end[], so the
   formula stays correct for arcs in any plane, including when a non-zero
   XY rotation tilts the arc plane out of the machine planes. */
struct LcncArc {
    uint32_t seq = 0;
    int32_t  lineno = -1;
    int32_t  source_id = 0;
    int32_t  plane = 1;        /* CANON_PLANE, informational */
    int32_t  tag = 0;
    double   center[3] = {0};
    double   radius = 0;
    double   start_angle = 0;
    double   end_angle = 0;
    double   u_axis[3] = {0};
    double   v_axis[3] = {0};
    double   n_axis[3] = {0};
    double   helix_delta = 0;
    double   start[9] = {0};
    double   end[9] = {0};
    double   feedrate = 0;
    double   tooloffset[3] = {0};
    LcncSyncInfo sync;
};

struct LcncDwell {
    uint32_t seq = 0;
    int32_t  lineno = -1;
    int32_t  source_id = 0;
    int32_t  kind = LCNC_DWELL_G4;   /* not a colour; the UI picks the palette */
    int32_t  tag = 0;
    double   pos[3] = {0};
    int32_t  plane_axis = 0;         /* 0/1/2, from the active plane */
};

/* Everything the interpreter used to ask Python for during a parse.  Supplied
   once up front, so nothing is queried while parsing. */
struct NativeConfig {
    std::vector<CANON_TOOL_TABLE> tools;
    int    axis_mask = 511;              /* XYZABCUVW */
    double angular_units = 1.0;
    double linear_units = 0.03937007874016;
    bool   block_delete = false;
    bool   random_toolchanger = false;
    std::string parameter_file;

    int    arcdivision = 64;
    bool   is_foam = false;
    double foam_z = 0.0;
    double foam_w = 1.5;
};

class NativePreview : public CanonSink {
public:
    explicit NativePreview(const NativeConfig &cfg);

    /* ---- accumulated output ---- */
    std::vector<LcncLine>  lines;
    std::vector<LcncArc>   arcs;
    std::vector<LcncDwell> dwells;
    std::vector<int>       tool_list;
    std::vector<std::string> sources;   /* sources[0] is the main program */
    std::vector<std::string> tags;      /* tags[0] is "no subroutine" */

    /* extents, in the order the Python side exposes them */
    double min_extents[3], max_extents[3];
    double min_extents_notool[3], max_extents_notool[3];
    double min_extents_zero_rxy[3], max_extents_zero_rxy[3];
    double min_extents_notool_zero_rxy[3], max_extents_notool_zero_rxy[3];

    double dwell_time = 0;
    int    notify_count = 0;
    std::string notify_message;

    /* ---- progress / cancellation, readable from another thread ---- */
    std::atomic<bool> abort_requested{false};
    std::atomic<int>  lines_done{0};

    /* Call once the parse has finished. */
    void finish();

    /* ---- CanonSink ---- */
    void next_line(const CanonLineState &st) override;
    void arc_feed(double first_end, double second_end,
                  double first_axis, double second_axis,
                  int rotation, double axis_end_point,
                  double a, double b, double c,
                  double u, double v, double w) override;
    void straight_feed(double x, double y, double z, double a, double b,
                       double c, double u, double v, double w) override;
    void straight_traverse(double x, double y, double z, double a, double b,
                           double c, double u, double v, double w) override;
    void straight_probe(double x, double y, double z, double a, double b,
                        double c, double u, double v, double w) override;
    void rigid_tap(double x, double y, double z) override;
    void set_g5x_offset(int index, double x, double y, double z,
                        double a, double b, double c,
                        double u, double v, double w) override;
    void set_g92_offset(double x, double y, double z, double a, double b,
                        double c, double u, double v, double w) override;
    void set_xy_rotation(double t) override;
    void set_plane(int plane) override;
    void set_traverse_rate(double rate) override;
    void change_tool(int tool) override;
    void set_feed_rate(double rate) override;
    void dwell(double time) override;
    void message(const char *s) override;
    void comment(const char *s) override;
    void tool_offset_set(double x, double y, double z, double a, double b,
                         double c, double u, double v, double w) override;
    void user_defined_function(int num, double p, double q) override;

    bool get_block_delete() override { return config.block_delete; }
    CANON_TOOL_TABLE get_tool(int pocket) override;
    int get_axis_mask() override { return config.axis_mask; }
    double get_external_angular_units() override { return config.angular_units; }
    double get_external_length_units() override { return config.linear_units; }
    bool check_abort() override { return abort_requested.load(); }
    void get_parameter_file_name(char *name, int max_size) override;

    const NativeConfig &cfg() const { return config; }

private:
    NativeConfig config;

    /* ---- state that used to live on the Python canon object ---- */
    double g5x_offset[9] = {0};
    double g92_offset[9] = {0};
    double rotation_xy = 0, rotation_sin = 0, rotation_cos = 1;
    double lo[9] = {0};
    double tool_offset_now[9] = {0};
    int    lineno = -1;
    int    source_id = 0;
    int    tool_in_spindle = -1;
    double feedrate = 1;
    int    plane = 1;
    int    g5x_index = 1;
    bool   first_move = true;
    int    suppress = 0;
    uint32_t seq = 0;
    CanonLineState modal{};
    bool   have_modal = false;

    /* extents accumulated as motions arrive */
    bool   any_motion = false;
    double acc_min[3], acc_max[3];
    double acc_min_t[3], acc_max_t[3];

    void rotate_and_translate(double x, double y, double z,
                              double a, double b, double c,
                              double u, double v, double w,
                              double out[9]) const;
    void note_point(const double p[9], const double to[3]);
    void add_line(int kind, const double start[9], const double end[9]);
    void add_dwell(int kind);
    void compute_zero_rxy_extents();
    int  intern(std::vector<std::string> &table, const std::string &s);
};

#endif /* NATIVECANON_HH */
