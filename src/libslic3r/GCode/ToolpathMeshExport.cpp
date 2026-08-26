#include "ToolpathMeshExport.hpp"

#include "libslic3r/Exception.hpp"
#include "libslic3r/Utils.hpp"

#include <boost/nowide/cstdio.hpp>
#include <boost/filesystem.hpp>

#include <array>
#include <cmath>
#include <cstdio>
#include <unordered_map>
#include <vector>

namespace Slic3r {

namespace {

// Number of sides of the (elliptical) tube cross-section ring. 32 matches the
// round-tube exporter this was ported from; lower it if file size becomes a concern.
constexpr int CROSS_SECTION_SIDES = 32;
constexpr float PI_F = 3.14159265358979323846f;
// Same rounding factor used by the original box/rounded-cap exporters: pulls the cap
// vertex back along -forward by this fraction of the tube width, giving a slightly
// domed end instead of a flat one.
constexpr float CAP_ROUNDING_FACTOR = 0.25f;

using ColorRGBu8 = std::array<unsigned char, 3>;

const char *move_type_to_string(EMoveType type) {
    switch (type) {
    case EMoveType::Retract: return "Retract";
    case EMoveType::Unretract: return "Unretract";
    case EMoveType::Seam: return "Seam";
    case EMoveType::Tool_change: return "ToolChange";
    case EMoveType::Color_change: return "ColorChange";
    case EMoveType::Pause_Print: return "PausePrint";
    case EMoveType::Custom_GCode: return "CustomGCode";
    case EMoveType::Travel: return "Travel";
    case EMoveType::Wipe: return "Wipe";
    case EMoveType::Extrude: return "Extrude";
    case EMoveType::Flush: return "Flush";
    default: return "Unknown";
    }
}

const char *time_mode_name(size_t idx) {
    switch (idx) {
    case 0: return "normal";
    case 1: return "stealth";
    default: return "mode";
    }
}

// Fixed, GUI-independent palette. Deliberately not tied to any libvgcode/view-type
// coloring, so the exported materials are stable regardless of what the Preview tab
// happens to be showing at export time.
ColorRGBu8 role_to_color(GCodeExtrusionRole role) {
    switch (role) {
    case GCodeExtrusionRole::Perimeter: return {0, 95, 115}; //P2
    case GCodeExtrusionRole::ExternalPerimeter: return {10, 147, 150}; //P3
    case GCodeExtrusionRole::OverhangPerimeter: return {255, 128, 0};
    case GCodeExtrusionRole::InternalInfill: return {202, 103, 2}; //P7
    case GCodeExtrusionRole::SolidInfill: return {187, 62, 3}; //P8
    case GCodeExtrusionRole::TopSolidInfill: return {238, 155, 0}; //P6
    case GCodeExtrusionRole::Ironing: return {233, 216, 166}; //P5
    case GCodeExtrusionRole::BridgeInfill: return {174, 32, 18}; //P9
    case GCodeExtrusionRole::GapFill: return {155, 34, 28}; //P10
    case GCodeExtrusionRole::Skirt: return {0, 18, 25}; //P1
    case GCodeExtrusionRole::SupportMaterial: return {148, 210, 189}; //P4
    case GCodeExtrusionRole::SupportMaterialInterface: return {0, 120, 0};
    case GCodeExtrusionRole::WipeTower: return {180, 180, 180}; //Unchanged
    case GCodeExtrusionRole::Custom: return {160, 160, 160};    // Unchanged
    default: return {128, 128, 128}; // None / unknown //Unchanged
    }
}

// Material key/color for a given move: Wipe and Flush get their own dedicated
// materials (regardless of role) so they read distinctly in the mesh viewer, and can
// be toggled via the "move_type" JSON column without also filtering by role.
std::string material_key_for(const GCodeProcessorResult::MoveVertex &mv) {
    if (mv.type == EMoveType::Wipe)
        return "wipe";
    if (mv.type == EMoveType::Flush)
        return "flush";
    return gcode_extrusion_role_to_string(mv.extrusion_role);
}

ColorRGBu8 material_color_for(const GCodeProcessorResult::MoveVertex &mv) {
    if (mv.type == EMoveType::Wipe)
        return {0, 200, 255};
    if (mv.type == EMoveType::Flush)
        return {255, 140, 0};
    return role_to_color(mv.extrusion_role);
}

std::string escape_json_string(const std::string &in) {
    std::string out;
    out.reserve(in.size());
    for (char c : in) {
        if (c == '\\' || c == '"')
            out += '\\';
        out += c;
    }
    return out;
}

// ---- geometry helpers -------------------------------------------------------

struct Vertex
{
    Vec3f position;
    Vec3f normal;
};

struct SegmentLocalAxes
{
    Vec3f forward;
    Vec3f right;
    Vec3f up;
};

SegmentLocalAxes segment_local_axes(const Vec3f &v1, const Vec3f &v2) {
    SegmentLocalAxes ret;
    ret.forward = (v2 - v1).normalized();
    // NOTE: degenerates if forward is (near) parallel to UnitZ, same limitation as
    // the reference implementation this was ported from. In practice this doesn't
    // happen for Extrude/Wipe/Flush moves, since extruding while moving purely
    // vertically isn't a thing slicers generate.
    ret.right = ret.forward.cross(Vec3f::UnitZ()).normalized();
    ret.up = ret.right.cross(ret.forward);
    return ret;
}

struct CrossSection
{
    std::vector<Vertex> ring; // size == CROSS_SECTION_SIDES
};

CrossSection cross_section(
    const Vec3f &v, const Vec3f &right, const Vec3f &up, float width, float height
) {
    CrossSection ret;
    ret.ring.reserve(CROSS_SECTION_SIDES);

    // Stadium / discorectangle shape: two straight sides (along `right`) connected by
    // semicircular caps of radius b = height/2 (the caps sit on the `up` axis, i.e.
    // the shape is "flattened" the same way a real extrusion bead is: wider than it
    // is tall). If height >= width, the straight part vanishes and this degenerates
    // to a plain circle of radius b.
    const float a = 0.5f * width;
    const float b = 0.5f * height;
    const float s = std::max(a - b, 0.0f); // half-length of each straight side

    const float arc_len = PI_F * b;      // length of one semicircular cap
    const float straight_len = 2.0f * s; // length of one straight side
    const float perimeter = 2.0f * arc_len + 2.0f * straight_len;

    for (int i = 0; i < CROSS_SECTION_SIDES; ++i) {
        const float dist = perimeter * (float(i) / float(CROSS_SECTION_SIDES));

        float x, y; // local 2D coords in the (right, up) plane
        Vec3f n;    // local outward normal

        if (dist < arc_len) {
            // right-hand cap: sweeps CCW from the bottom (-90 deg) to the top (+90 deg)
            const float theta = -0.5f * PI_F + dist / b;
            x = s + b * std::cos(theta);
            y = b * std::sin(theta);
            n = std::cos(theta) * right + std::sin(theta) * up;
        } else if (dist < arc_len + straight_len) {
            // top straight side: from (s, b) to (-s, b)
            const float t = (dist - arc_len) / straight_len;
            x = s - t * straight_len;
            y = b;
            n = up;
        } else if (dist < 2.0f * arc_len + straight_len) {
            // left-hand cap: sweeps CCW from the top (+90 deg) to the bottom (+270 deg)
            const float theta = 0.5f * PI_F + (dist - arc_len - straight_len) / b;
            x = -s + b * std::cos(theta);
            y = b * std::sin(theta);
            n = std::cos(theta) * right + std::sin(theta) * up;
        } else {
            // bottom straight side: from (-s, -b) to (s, -b)
            const float t = (dist - 2.0f * arc_len - straight_len) / straight_len;
            x = -s + t * straight_len;
            y = -b;
            n = -up;
        }

        Vertex vx;
        vx.position = v + x * right + y * up;
        const float nl = n.norm();
        vx.normal = (nl > 1e-12f) ? (n / nl) : right;

        ret.ring.push_back(vx);
    }

    return ret;
}

CrossSection normal_cross_section(
    const Vec3f &v, const SegmentLocalAxes &axes, float width, float height
) {
    return cross_section(v, axes.right, axes.up, width, height);
}

enum class CornerType : unsigned char { RightTurn, LeftTurn, Straight };

// corner_type is computed for parity with the reference implementation (and possible
// future use, e.g. widening the ring on sharp turns) but is currently unused, exactly
// as in the code this was ported from.
CrossSection corner_cross_section(
    const Vec3f &v,
    const SegmentLocalAxes &axes1,
    const SegmentLocalAxes &axes2,
    float width,
    float height,
    CornerType &corner_type
) {
    if (std::abs(std::abs(axes1.forward.dot(axes2.forward)) - 1.0f) < EPSILON)
        corner_type = CornerType::Straight;
    else if (axes1.up.dot(axes1.forward.cross(axes2.forward)) < 0.0f)
        corner_type = CornerType::RightTurn;
    else
        corner_type = CornerType::LeftTurn;

    const Vec3f right = (0.5f * (axes1.right + axes2.right)).normalized();
    return cross_section(v, right, axes1.up, width, height);
}

// ---- low-level OBJ writing, absolute 1-based indices -------------------------

struct RingRef
{
    size_t start_vertex_id{0}; // absolute 1-based OBJ index of the ring's first vertex
};

void write_vertex(FILE *f, const Vertex &v) {
    fprintf(f, "v %g %g %g\n", double(v.position.x()), double(v.position.y()), double(v.position.z()));
    fprintf(f, "vn %g %g %g\n", double(v.normal.x()), double(v.normal.y()), double(v.normal.z()));
}

void write_triangle(FILE *f, size_t v1, size_t v2, size_t v3) {
    fprintf(f, "f %zu//%zu %zu//%zu %zu//%zu\n", v1, v1, v2, v2, v3, v3);
}

void write_material_ref(FILE *f, size_t material_index_1based) {
    fprintf(f, "\nusemtl material_%zu\n", material_index_1based);
}

RingRef write_ring(FILE *f, const CrossSection &cs, size_t &next_vertex_id) {
    RingRef ref;
    ref.start_vertex_id = next_vertex_id;
    for (const Vertex &vx : cs.ring)
        write_vertex(f, vx);
    next_vertex_id += cs.ring.size();
    return ref;
}

void write_stitch_rings(FILE *f, const RingRef &prev_ring, const RingRef &curr_ring) {
    for (int i = 0; i < CROSS_SECTION_SIDES; ++i) {
        const int in = (i + 1) % CROSS_SECTION_SIDES;
        const size_t prev_i = prev_ring.start_vertex_id + i;
        const size_t prev_in = prev_ring.start_vertex_id + in;
        const size_t cur_i = curr_ring.start_vertex_id + i;
        const size_t cur_in = curr_ring.start_vertex_id + in;
        write_triangle(f, prev_i, cur_i, cur_in);
        write_triangle(f, prev_i, cur_in, prev_in);
    }
}

void write_cap_fan(FILE *f, size_t center_vertex_id, const RingRef &ring) {
    for (int i = 0; i < CROSS_SECTION_SIDES; ++i) {
        const int in = (i + 1) % CROSS_SECTION_SIDES;
        write_triangle(f, center_vertex_id, ring.start_vertex_id + i, ring.start_vertex_id + in);
    }
}

// True if the move from from_pos to `to` should produce tube geometry: it must be an
// Extrude/Wipe/Flush move with a valid (non-stale) width/height and non-zero length.
// Used both for the current row and, identically, for one-row lookahead -- keeping
// this in one place guarantees the "is there an open ring to stitch to" bookkeeping
// stays internally consistent (see the loop below).
bool segment_has_geometry(const Vec3f &from_pos, const GCodeProcessorResult::MoveVertex &to) {
    const bool printable_type = to.type == EMoveType::Extrude || to.type == EMoveType::Wipe ||
        to.type == EMoveType::Flush;
    if (!printable_type)
        return false;
    if (to.width <= 0.0f || to.height <= 0.0f)
        return false;
    return (to.position - from_pos).norm() > 1e-6f;
}

// ---- columnar JSON writer (unchanged from phase 1, plus a "material" column) --

template<typename T>
void append_uint_field(
    std::vector<std::string> &fields, const char *name, const std::vector<T> &values
) {
    std::string s;
    s.reserve(values.size() * 8 + 32);
    s += "  \"";
    s += name;
    s += "\": [";
    char buf[32];
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0)
            s += ',';
        snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(values[i]));
        s += buf;
    }
    s += ']';
    fields.push_back(std::move(s));
}

void append_float_field(
    std::vector<std::string> &fields, const std::string &name, const std::vector<float> &values
) {
    std::string s;
    s.reserve(values.size() * 10 + 32);
    s += "  \"";
    s += name;
    s += "\": [";
    char buf[32];
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0)
            s += ',';
        snprintf(buf, sizeof(buf), "%.5f", double(values[i]));
        s += buf;
    }
    s += ']';
    fields.push_back(std::move(s));
}

void append_string_field(
    std::vector<std::string> &fields, const char *name, const std::vector<std::string> &values
) {
    std::string s;
    s.reserve(values.size() * 16 + 32);
    s += "  \"";
    s += name;
    s += "\": [";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0)
            s += ',';
        s += '"';
        s += values[i];
        s += '"'; // fixed lookup-table strings, no escaping needed
    }
    s += ']';
    fields.push_back(std::move(s));
}

void append_vec3_field(
    std::vector<std::string> &fields, const char *name, const std::vector<std::array<float, 3>> &values
) {
    std::string s;
    s.reserve(values.size() * 30 + 32);
    s += "  \"";
    s += name;
    s += "\": [";
    char buf[64];
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0)
            s += ',';
        snprintf(
            buf, sizeof(buf), "[%.4f,%.4f,%.4f]", double(values[i][0]), double(values[i][1]),
            double(values[i][2])
        );
        s += buf;
    }
    s += ']';
    fields.push_back(std::move(s));
}

} // anonymous namespace

ToolpathMeshExportStats export_toolpath_mesh_and_mapping(
    const GCodeProcessorResult &result, const ToolpathMeshExportParams &params
) {
    ToolpathMeshExportStats stats;

    FilePtr obj_file{boost::nowide::fopen(params.obj_path.c_str(), "w")};
    if (obj_file.f == nullptr)
        throw Slic3r::RuntimeError(
            "export_toolpath_mesh_and_mapping: cannot open '" + params.obj_path + "' for writing"
        );

    FilePtr mtl_file{boost::nowide::fopen(params.mtl_path.c_str(), "w")};
    if (mtl_file.f == nullptr)
        throw Slic3r::RuntimeError(
            "export_toolpath_mesh_and_mapping: cannot open '" + params.mtl_path + "' for writing"
        );

    FilePtr json_file{boost::nowide::fopen(params.json_path.c_str(), "w")};
    if (json_file.f == nullptr)
        throw Slic3r::RuntimeError(
            "export_toolpath_mesh_and_mapping: cannot open '" + params.json_path + "' for writing"
        );

    fprintf(obj_file.f, "# Toolpath tube mesh exported by PrusaSlicer (phase 1 prototype)\n");
    fprintf(
        obj_file.f,
        "# One stitched, rounded tube per contiguous run of same-type Extrude/Wipe/Flush moves.\n"
    );
    fprintf(obj_file.f, "# See the companion .json file for the gcode_id <-> geometry mapping.\n");
    fprintf(
        obj_file.f, "mtllib ./%s\n",
        boost::filesystem::path(params.mtl_path).filename().string().c_str()
    );

    // materials registry: key (role name / "wipe" / "flush") -> 1-based material index
    std::unordered_map<std::string, size_t> material_index;
    std::vector<std::pair<std::string, ColorRGBu8>>
        materials; // in insertion order, for the .mtl file

    auto material_for = [&](const GCodeProcessorResult::MoveVertex &mv) -> size_t {
        const std::string key = material_key_for(mv);
        auto it = material_index.find(key);
        if (it != material_index.end())
            return it->second;
        materials.emplace_back(key, material_color_for(mv));
        const size_t idx = materials.size(); // 1-based
        material_index.emplace(key, idx);
        return idx;
    };

    const size_t time_modes = result.moves.empty() ? 0 : result.moves.front().time.size();
    std::vector<double> cumulative_time(time_modes, 0.0);

    const size_t n_rows = result.moves.empty() ? 0 : result.moves.size() - 1;

    std::vector<uint32_t> col_gcode_id;
    col_gcode_id.reserve(n_rows);
    std::vector<uint32_t> col_layer_id;
    col_layer_id.reserve(n_rows);
    std::vector<uint32_t> col_extruder_id;
    col_extruder_id.reserve(n_rows);
    std::vector<std::string> col_role;
    col_role.reserve(n_rows);
    std::vector<std::string> col_move_type;
    col_move_type.reserve(n_rows);
    std::vector<std::string> col_material;
    col_material.reserve(n_rows);
    std::vector<std::array<float, 3>> col_p_start;
    col_p_start.reserve(n_rows);
    std::vector<std::array<float, 3>> col_p_end;
    col_p_end.reserve(n_rows);
    std::vector<uint64_t> col_vbegin;
    col_vbegin.reserve(n_rows);
    std::vector<uint64_t> col_vcount;
    col_vcount.reserve(n_rows);
    std::vector<uint64_t> col_fbegin;
    col_fbegin.reserve(n_rows);
    std::vector<uint64_t> col_fcount;
    col_fcount.reserve(n_rows);
    std::vector<std::vector<float>> col_t_start(time_modes);
    std::vector<std::vector<float>> col_t_end(time_modes);
    for (size_t m = 0; m < time_modes; ++m) {
        col_t_start[m].reserve(n_rows);
        col_t_end[m].reserve(n_rows);
    }

    size_t next_vertex_id = 1; // OBJ indices are 1-based
    size_t total_faces = 0;

    // run state carried between rows: is there an "open" ring (from the end of the
    // previous row) that this row can stitch onto?
    bool open_valid = false;
    RingRef open_ring;
    EMoveType open_type = EMoveType::Noop;

    for (size_t i = 1; i < result.moves.size(); ++i) {
        const GCodeProcessorResult::MoveVertex &prev = result.moves[i - 1];
        const GCodeProcessorResult::MoveVertex &curr = result.moves[i];

        const bool valid = segment_has_geometry(prev.position, curr);
        const bool continues = valid && open_valid && open_type == curr.type;
        const bool starts = valid && !continues;

        const size_t v_before = next_vertex_id - 1;
        const size_t f_before = total_faces;

        if (!valid) {
            // Defensive: with the lookahead below, any row whose successor breaks the
            // run already gets end-capped there, so reaching an invalid row with
            // open_valid still true should be structurally unreachable. Guarded anyway.
            open_valid = false;
        } else {
            const SegmentLocalAxes axes_cur = segment_local_axes(prev.position, curr.position);
            const size_t mat = material_for(curr);

            RingRef ring_for_stitch;
            if (starts) {
                // Start-of-run cap + ring, positioned at the true run-start point (prev),
                // but sized using curr's width/height -- see note in the class docs
                // about why prev's width/height can't be trusted here.
                const Vertex cap0{
                    prev.position - CAP_ROUNDING_FACTOR * curr.width * axes_cur.forward,
                    -axes_cur.forward
                };
                const size_t cap0_id = next_vertex_id;
                write_vertex(obj_file.f, cap0);
                ++next_vertex_id;

                const CrossSection cs0 =
                    normal_cross_section(prev.position, axes_cur, curr.width, curr.height);
                const RingRef ring0 = write_ring(obj_file.f, cs0, next_vertex_id);

                write_material_ref(obj_file.f, mat);
                write_cap_fan(obj_file.f, cap0_id, ring0);
                total_faces += CROSS_SECTION_SIDES;

                ring_for_stitch = ring0;
            } else {
                ring_for_stitch = open_ring;
            }

            // Lookahead: does the *next* row continue this same run? Determines whether
            // this row's ring is a plain (end) ring or a mitred corner ring, and whether
            // we close the run with an end cap here.
            bool next_continues = false;
            if (i + 1 < result.moves.size())
                next_continues = segment_has_geometry(curr.position, result.moves[i + 1]) &&
                    result.moves[i + 1].type == curr.type;
            const bool ends = !next_continues;

            CrossSection cs_main;
            if (ends) {
                cs_main = normal_cross_section(curr.position, axes_cur, curr.width, curr.height);
            } else {
                const SegmentLocalAxes axes_next =
                    segment_local_axes(curr.position, result.moves[i + 1].position);
                CornerType
                    corner_type; // computed, currently unused -- see corner_cross_section() comment
                cs_main = corner_cross_section(
                    curr.position, axes_cur, axes_next, curr.width, curr.height, corner_type
                );
            }
            const RingRef ring_main = write_ring(obj_file.f, cs_main, next_vertex_id);

            write_material_ref(obj_file.f, mat);
            write_stitch_rings(obj_file.f, ring_for_stitch, ring_main);
            total_faces += 2 * CROSS_SECTION_SIDES;

            if (ends) {
                const Vertex cap1{
                    curr.position + CAP_ROUNDING_FACTOR * curr.width * axes_cur.forward,
                    axes_cur.forward
                };
                const size_t cap1_id = next_vertex_id;
                write_vertex(obj_file.f, cap1);
                ++next_vertex_id;

                write_material_ref(obj_file.f, mat);
                write_cap_fan(obj_file.f, cap1_id, ring_main);
                total_faces += CROSS_SECTION_SIDES;

                open_valid = false;
            } else {
                open_ring = ring_main;
                open_type = curr.type;
                open_valid = true;
            }
        }

        col_gcode_id.push_back(curr.gcode_id);
        col_layer_id.push_back(curr.layer_id);
        col_extruder_id.push_back(static_cast<uint32_t>(curr.extruder_id));
        col_role.push_back(gcode_extrusion_role_to_string(curr.extrusion_role));
        col_move_type.push_back(move_type_to_string(curr.type));
        col_material.push_back(valid ? material_key_for(curr) : std::string());
        col_p_start.push_back({prev.position.x(), prev.position.y(), prev.position.z()});
        col_p_end.push_back({curr.position.x(), curr.position.y(), curr.position.z()});

        const uint64_t vcount = (next_vertex_id - 1) - v_before;
        const uint64_t fcount = total_faces - f_before;
        col_vbegin.push_back(vcount > 0 ? v_before + 1 : 0);
        col_vcount.push_back(vcount);
        col_fbegin.push_back(fcount > 0 ? f_before + 1 : 0);
        col_fcount.push_back(fcount);

        for (size_t m = 0; m < time_modes; ++m) {
            const double t0 = cumulative_time[m];
            cumulative_time[m] += double(curr.time[m]);
            col_t_start[m].push_back(static_cast<float>(t0));
            col_t_end[m].push_back(static_cast<float>(cumulative_time[m]));
        }

        ++stats.rows_written;
        if (vcount > 0) {
            ++stats.geometry_rows;
            stats.obj_vertices += vcount;
            stats.obj_faces += fcount;
        }
    }
    stats.obj_materials = materials.size();

    // ---- write the .mtl file ----
    fprintf(mtl_file.f, "# Toolpath materials exported by PrusaSlicer (phase 1 prototype)\n");
    fprintf(
        mtl_file.f,
        "# One material per GCodeExtrusionRole encountered, plus dedicated wipe/flush materials.\n"
    );
    static const float inv_255 = 1.0f / 255.0f;
    for (size_t i = 0; i < materials.size(); ++i) {
        const auto &[key, color] = materials[i];
        fprintf(mtl_file.f, "\nnewmtl material_%zu\n", i + 1);
        fprintf(mtl_file.f, "# %s\n", key.c_str());
        fprintf(mtl_file.f, "Ka 1 1 1\n");
        fprintf(
            mtl_file.f, "Kd %g %g %g\n", double(color[0]) * inv_255, double(color[1]) * inv_255,
            double(color[2]) * inv_255
        );
        fprintf(mtl_file.f, "Ks 0 0 0\n");
    }

    // ---- assemble and write the JSON sidecar ----
    std::vector<std::string> fields;

    {
        char buf[512];
        snprintf(
            buf, sizeof(buf), "  \"obj_file\": \"%s\"",
            escape_json_string(boost::filesystem::path(params.obj_path).filename().string()).c_str()
        );
        fields.push_back(buf);

        snprintf(
            buf, sizeof(buf), "  \"mtl_file\": \"%s\"",
            escape_json_string(boost::filesystem::path(params.mtl_path).filename().string()).c_str()
        );
        fields.push_back(buf);

        snprintf(
            buf, sizeof(buf),
            "  \"coordinate_frame\": { \"z_offset\": %g, \"bed_translation\": [%g, %g, %g] }",
            double(params.z_offset), double(params.bed_translation.x()),
            double(params.bed_translation.y()), double(params.bed_translation.z())
        );
        fields.push_back(buf);
    }

    {
        std::string s = "  \"time_modes\": [";
        for (size_t m = 0; m < time_modes; ++m) {
            if (m > 0)
                s += ", ";
            s += '"';
            s += time_mode_name(m);
            s += '"';
        }
        s += ']';
        fields.push_back(std::move(s));
    }

    fields.push_back("  \"count\": " + std::to_string(col_gcode_id.size()));

    append_uint_field(fields, "gcode_id", col_gcode_id);
    append_uint_field(fields, "layer_id", col_layer_id);
    append_uint_field(fields, "extruder_id", col_extruder_id);
    append_string_field(fields, "role", col_role);
    append_string_field(fields, "move_type", col_move_type);
    append_string_field(fields, "material", col_material);
    append_vec3_field(fields, "p_start", col_p_start);
    append_vec3_field(fields, "p_end", col_p_end);
    append_uint_field(fields, "obj_vertex_begin", col_vbegin);
    append_uint_field(fields, "obj_vertex_count", col_vcount);
    append_uint_field(fields, "obj_face_begin", col_fbegin);
    append_uint_field(fields, "obj_face_count", col_fcount);

    for (size_t m = 0; m < time_modes; ++m) {
        append_float_field(fields, std::string("t_start_") + time_mode_name(m), col_t_start[m]);
        append_float_field(fields, std::string("t_end_") + time_mode_name(m), col_t_end[m]);
    }

    fprintf(json_file.f, "{\n");
    for (size_t i = 0; i < fields.size(); ++i)
        fprintf(json_file.f, "%s%s\n", fields[i].c_str(), (i + 1 < fields.size()) ? "," : "");
    fprintf(json_file.f, "}\n");

    return stats;
}

} // namespace Slic3r