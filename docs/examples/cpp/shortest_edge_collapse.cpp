// The shortest edge collapse tutorial (docs/tutorials/shortest_edge_collapse.md): simplify the
// Octocat to a tenth of its vertices, inside an envelope. The same run as
// json/shortest_edge_collapse.json and python/shortest_edge_collapse.py.

#include <wmtk/components/shortest_edge_collapse/shortest_edge_collapse.hpp>
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
    wmtk::MeshMap meshes{{"octocat", wmtk::io::read_mesh(data / "Octocat.obj")}};
    auto [outputs, report] = wmtk::components::shortest_edge_collapse::shortest_edge_collapse(
        {{"input", "octocat"}, {"target_rel", 0.1}, {"eps_rel", 0.002}, {"num_threads", 4}},
        meshes);
    const wmtk::MeshData& simplified = outputs.at("mesh");
    std::cout << meshes.at("octocat").V.rows() << " -> " << simplified.V.rows() << " vertices\n";
    wmtk::io::write_mesh("octocat_sec.msh", simplified);
    // --8<-- [end:run]
}
