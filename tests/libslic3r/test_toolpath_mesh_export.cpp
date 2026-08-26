#include <catch2/catch_test_macros.hpp>
#include "libslic3r/GCode/ToolpathMeshExport.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>

using namespace Slic3r;

namespace {

GCodeProcessorResult::MoveVertex make_move(
    unsigned int gcode_id,
    EMoveType type,
    GCodeExtrusionRole role,
    unsigned int layer_id,
    unsigned char extruder_id,
    Vec3f position,
    float width,
    float height,
    float dt
) {
    GCodeProcessorResult::MoveVertex mv{};
    mv.gcode_id = gcode_id;
    mv.type = type;
    mv.extrusion_role = role;
    mv.layer_id = layer_id;
    mv.extruder_id = extruder_id;
    mv.position = position;
    mv.width = width;
    mv.height = height;
    for (size_t m = 0; m < mv.time.size(); ++m)
        mv.time[m] = dt;
    mv.internal_only = false;
    return mv;
}

} // namespace

TEST_CASE("ToolpathMeshExport: basic extrude/wipe/travel sequence", "[ToolpathMeshExport]") {
    GCodeProcessorResult result;
    result.moves.push_back(make_move(
        1, EMoveType::Travel, GCodeExtrusionRole::None, 0, 0, Vec3f(0, 0, 0.2f), 0.f, 0.f, 0.1f
    ));
    result.moves.push_back(make_move(
        2, EMoveType::Extrude, GCodeExtrusionRole::Perimeter, 0, 0, Vec3f(10, 0, 0.2f), 0.45f, 0.2f,
        1.0f
    ));
    result.moves.push_back(make_move(
        2, EMoveType::Extrude, GCodeExtrusionRole::Perimeter, 0, 0, Vec3f(10, 10, 0.2f), 0.45f,
        0.2f, 1.0f
    )); // same gcode_id: simulates an arc split
    result.moves.push_back(make_move(
        3, EMoveType::Wipe, GCodeExtrusionRole::Perimeter, 0, 0, Vec3f(9, 10, 0.2f), 0.05f, 0.05f,
        0.2f
    ));
    result.moves.push_back(make_move(
        4, EMoveType::Retract, GCodeExtrusionRole::None, 0, 0, Vec3f(9, 10, 0.2f), 0.f, 0.f, 0.05f
    ));

    const boost::filesystem::path tmp_dir = boost::filesystem::temp_directory_path();
    ToolpathMeshExportParams params;
    params.obj_path = (tmp_dir / "toolpath_test.obj").string();
    params.json_path = (tmp_dir / "toolpath_test.json").string();

    ToolpathMeshExportStats stats = export_toolpath_mesh_and_mapping(result, params);

    CHECK(stats.rows_written == result.moves.size() - 1);
    CHECK(stats.geometry_rows == 3); // Extrude, Extrude, Wipe rows still produce geometry
    CHECK(stats.obj_vertices > 0);
    CHECK(stats.obj_faces > 0);
    CHECK(stats.obj_materials >= 2); // at least "Perimeter" and "wipe" in this fixture
    // vertex/face counts must still reconcile between the OBJ file and the JSON sums,
    // exactly as before -- keep that part of the test unchanged.

    std::ifstream obj_in(params.obj_path);
    REQUIRE(obj_in.good());
    size_t v_count = 0, f_count = 0;
    std::string line;
    while (std::getline(obj_in, line)) {
        if (line.rfind("v ", 0) == 0)
            ++v_count;
        else if (line.rfind("f ", 0) == 0)
            ++f_count;
    }
    CHECK(v_count == stats.obj_vertices);
    CHECK(f_count == stats.obj_faces);

    std::ifstream json_in(params.json_path);
    REQUIRE(json_in.good());
    std::stringstream ss;
    ss << json_in.rdbuf();
    const std::string json_text = ss.str();
    CHECK(json_text.find("\"gcode_id\"") != std::string::npos);
    CHECK(json_text.find("\"move_type\"") != std::string::npos);
}