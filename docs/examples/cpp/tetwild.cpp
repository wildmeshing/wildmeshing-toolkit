// The TetWild tutorial (docs/tutorials/tetwild.md): tetrahedralize the inside of a Thingi10K
// snowman. The same run as json/tetwild.json and python/tetwild.py.

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
    wmtk::MeshMap meshes{{"snowman", wmtk::io::read_mesh(data / "thingi_366725.stl")}};
    auto [outputs, report] = wmtk::components::tetwild::tetwild(
        {{"input", nlohmann::json::array({"snowman"})},
         {"filter", "tracked"},
         {"stop_energy", 10},
         {"num_threads", 4}},
        meshes);
    const wmtk::MeshData& mesh = outputs.at("mesh");
    const wmtk::MeshData& surface = outputs.at("surface");
    std::cout << mesh.V.rows() << " vertices, "
              << mesh.cell(wmtk::CellType::Tetrahedron).connectivity.rows() << " tets, "
              << surface.cell(wmtk::CellType::Triangle).connectivity.rows()
              << " surface triangles, max energy " << report["max_energy"].get<double>() << "\n";
    wmtk::io::write_mesh("snowman.msh", mesh);
    // --8<-- [end:run]
}
