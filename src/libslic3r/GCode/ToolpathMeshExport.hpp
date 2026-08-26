#ifndef slic3r_ToolpathMeshExport_hpp_
#define slic3r_ToolpathMeshExport_hpp_

#include <string>
#include "libslic3r/GCode/GCodeProcessor.hpp"

namespace Slic3r {

// Parameters controlling how the toolpath mesh, its material file, and the mapping
// sidecar are written.
struct ToolpathMeshExportParams
{
    // Path of the Wavefront OBJ file containing the tube geometry.
    std::string obj_path;
    // Path of the Wavefront MTL file containing the per-role/move-type materials.
    // Referenced from obj_path via "mtllib <basename>".
    std::string mtl_path;
    // Path of the columnar JSON sidecar file with the gcode_id <-> geometry mapping.
    std::string json_path;

    // Recorded into the JSON only, so downstream tools can reconcile this export with
    // whatever frame the model OBJ / CT data uses. MoveVertex::position is already
    // expressed in the frame GCodeProcessor works in -- this is metadata, not a
    // transform applied here.
    Vec3f bed_translation{Vec3f::Zero()};
    float z_offset{0.0f};
};

// Result of the export, useful for logging / assertions in tests.
struct ToolpathMeshExportStats
{
    size_t rows_written{0}; // one row per MoveVertex except the first (which has no predecessor)
    size_t geometry_rows{0
    }; // rows with obj_vertex_count > 0 (Extrude / Wipe / Flush, non-degenerate)
    size_t obj_vertices{0};
    size_t obj_faces{0};
    size_t obj_materials{0};
};

// Exports the toolpaths contained in a GCodeProcessorResult as:
//  - a Wavefront OBJ file (params.obj_path): a stitched, rounded tube (elliptical
//    cross-section, CROSS_SECTION_SIDES-gon rings) per contiguous run of same-type
//    Extrude/Wipe/Flush moves, with rounded end caps at the start/end of each run.
//  - a Wavefront MTL file (params.mtl_path): one material per GCodeExtrusionRole
//    encountered, plus dedicated "wipe"/"flush" materials, colored from a fixed
//    palette (independent of any GUI view-type state).
//  - a columnar JSON sidecar (params.json_path): one row per move (including Travel/
//    Retract/etc, recorded with zero geometry so the gcode_id sequence has no
//    unexplained gaps), giving gcode line id, layer, extruder, role, move type,
//    material key, cumulative start/end time per time mode, endpoint positions, and
//    the OBJ vertex/face ranges the row produced.
//
// A single physical gcode line that got discretized into several moves (e.g. G2/G3
// arcs) will appear as several consecutive rows sharing the same gcode_id -- this is
// intentional, see GCodeProcessor::process_G2_G3()/process_G1() call chain.
//
// Throws Slic3r::RuntimeError if any output file cannot be opened for writing.
ToolpathMeshExportStats export_toolpath_mesh_and_mapping(
    const GCodeProcessorResult &result, const ToolpathMeshExportParams &params
);

} // namespace Slic3r

#endif // slic3r_ToolpathMeshExport_hpp_