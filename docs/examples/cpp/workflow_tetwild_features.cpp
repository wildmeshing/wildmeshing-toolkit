// The workflow TetWild with features (docs/tutorials/workflows/tetwild-features.md): a cube with a
// feature curve and feature points inside, forced into the tet mesh. The same run as
// json/workflow_tetwild_features.json and python/workflow_tetwild_features.py.

#include <wmtk/components/tetwild/tetwild.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>

int main()
{
    // The example meshes: $WMTK_DATA, or data/models from the repository root.
    const char* env = std::getenv("WMTK_DATA");
    const std::filesystem::path data = env ? env : "data/models";

    // --8<-- [start:run]
    wmtk::MeshMap inputs{
        {"cube", wmtk::io::read_mesh(data / "tetwild_features_cube.obj")},
        {"curves", wmtk::io::read_mesh(data / "tetwild_features_edges.obj")},
        {"points", wmtk::io::read_mesh(data / "tetwild_features_points.obj")}};
    auto [outputs, report] = wmtk::components::tetwild::tetwild(
        {{"input", nlohmann::json::array({"cube"})},
         {"input_edges", nlohmann::json::array({"curves"})},
         {"input_points", nlohmann::json::array({"points"})},
         {"filter", "tracked"},
         {"stop_energy", 10},
         {"num_threads", 0}},
        inputs);
    const wmtk::MeshData& mesh = outputs.at("mesh");
    const wmtk::MeshData& features = outputs.at("features");
    std::cout << mesh.V.rows() << " vertices, "
              << mesh.cell(wmtk::CellType::Tetrahedron).connectivity.rows() << " tets, "
              << features.cell(wmtk::CellType::Edge).connectivity.rows()
              << " tet edges on the curve\n";
    // Every input point is a vertex of the tet mesh: the nearest vertex is at distance 0.
    const auto& points = inputs.at("points").V;
    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        const double d = (mesh.V.rowwise() - points.row(i)).rowwise().norm().minCoeff();
        std::cout << "point " << points.row(i) << ": nearest vertex at " << d << "\n";
    }
    wmtk::io::write_mesh("cube.msh", mesh);
    // --8<-- [end:run]
}
