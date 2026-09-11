/********************************************************************
* Description: canonsink.hh
*
*   Abstract sink for the canonical machining commands produced by the
*   preview interpreter (gcodemodule.cc).
*
*   Historically gcodemodule.cc implemented every canon function by
*   calling straight into a Python object via PyObject_CallMethod.  That
*   made the preview engine unusable without a live CPython interpreter,
*   and cost ~20 upcalls per motion.
*
*   CanonSink separates *what* the canon reports from *where* it goes:
*
*     PyCallbackSink  - the historical behaviour, bit for bit
*     NativeSink      - accumulates into plain C++ vectors (nativecanon.hh)
*
*   The per-parse state that used to live in file-scope globals of
*   gcodemodule.cc lives here instead, so the two implementations do not
*   share a common pool of mutable globals.
*
* License: GPL Version 2
********************************************************************/
#ifndef CANONSINK_HH
#define CANONSINK_HH

#include "nml_intf/canon.hh"
#include "interp_base.hh"

/* Modal snapshot handed to next_line().  Mirrors the layout that the
   Python-facing gcode.linecode type exposes; sequence_number is gcodes[0],
   exactly as before. */
struct CanonLineState {
    double settings[ACTIVE_SETTINGS];
    int    gcodes[ACTIVE_G_CODES];
    int    mcodes[ACTIVE_M_CODES];

    int sequence_number() const { return gcodes[0]; }
};

class CanonSink {
public:
    virtual ~CanonSink() {}

    /* ---- per-parse state (was file-scope globals in gcodemodule.cc) ----
       Exactly the set that parse_file() used to reset on entry.

       NOT here on purpose: selected_tool and tool_offset.  Those persist
       across parses in the historical code, and the interpreter reads the
       tool offset back through GET_EXTERNAL_TOOL_LENGTH_*OFFSET() when it
       handles G43 - which is what decides whether active_g_codes[9] reports
       G43 or G49 (interp_write.cc:110).  Zeroing them per parse changes the
       reported modal state of any file loaded after one that set a TLO, so
       they stay process-global in gcodemodule.cc. */
    int     interp_error = 0;
    int     last_sequence_number = -1;
    bool    metric = false;
    double  pos_x = 0, pos_y = 0, pos_z = 0;
    double  pos_a = 0, pos_b = 0, pos_c = 0;
    double  pos_u = 0, pos_v = 0, pos_w = 0;

    void reset_state() {
        interp_error = 0;
        last_sequence_number = -1;
        metric = false;
        pos_x = pos_y = pos_z = 0;
        pos_a = pos_b = pos_c = 0;
        pos_u = pos_v = pos_w = 0;
    }

    void set_position(double x, double y, double z,
                      double a, double b, double c,
                      double u, double v, double w) {
        pos_x = x; pos_y = y; pos_z = z;
        pos_a = a; pos_b = b; pos_c = c;
        pos_u = u; pos_v = v; pos_w = w;
    }

    /* ---- outputs ---------------------------------------------------- */
    virtual void next_line(const CanonLineState &st) = 0;

    virtual void arc_feed(double first_end, double second_end,
                          double first_axis, double second_axis,
                          int rotation, double axis_end_point,
                          double a, double b, double c,
                          double u, double v, double w) = 0;

    virtual void straight_feed(double x, double y, double z,
                               double a, double b, double c,
                               double u, double v, double w) = 0;

    virtual void straight_traverse(double x, double y, double z,
                                   double a, double b, double c,
                                   double u, double v, double w) = 0;

    virtual void straight_probe(double x, double y, double z,
                                double a, double b, double c,
                                double u, double v, double w) = 0;

    virtual void rigid_tap(double x, double y, double z) = 0;

    virtual void set_g5x_offset(int index,
                                double x, double y, double z,
                                double a, double b, double c,
                                double u, double v, double w) = 0;

    virtual void set_g92_offset(double x, double y, double z,
                                double a, double b, double c,
                                double u, double v, double w) = 0;

    virtual void set_xy_rotation(double t) = 0;
    virtual void set_plane(int plane) = 0;
    virtual void set_traverse_rate(double rate) = 0;
    virtual void change_tool(int tool) = 0;
    virtual void set_feed_rate(double rate) = 0;
    virtual void dwell(double time) = 0;
    virtual void message(const char *s) = 0;
    virtual void comment(const char *s) = 0;

    /* offset already converted to inches by the caller */
    virtual void tool_offset_set(double x, double y, double z,
                                 double a, double b, double c,
                                 double u, double v, double w) = 0;

    virtual void user_defined_function(int num, double p, double q) = 0;

    /* ---- inputs ----------------------------------------------------- */
    virtual bool get_block_delete() = 0;
    virtual CANON_TOOL_TABLE get_tool(int pocket) = 0;
    virtual int get_axis_mask() = 0;
    virtual double get_external_angular_units() = 0;
    virtual double get_external_length_units() = 0;
    virtual bool check_abort() = 0;
    virtual void get_parameter_file_name(char *name, int max_size) = 0;
};

/* The sink in use for the parse currently in flight.  Never null while
   parse_file() is running; the canon functions dereference it directly. */
extern CanonSink *g_sink;

#endif /* CANONSINK_HH */
