// The topological offset tutorial (docs/tutorials/topological_offset.md): a band at distance 0.4
// around a body, as constructed. The same run as json/topological_offset.json and
// python/topological_offset.py.

#include <wmtk/components/topological_offset/topological_offset.hpp>
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
    wmtk::MeshMap meshes{{"bar", wmtk::io::read_mesh(data / "127891.msh")}};
    auto [outputs, report] = wmtk::components::topological_offset::topological_offset(
        {{"input", "bar"},
         {"offset_selection", "127891"},
         {"offset_output_tags", nlohmann::json::array({"offset"})},
         {"target_distance", 0.4},
         {"optimize_offset", false}},
        meshes);
    std::cout << report["before #t"] << " -> " << report["after #t"] << " tets\n";
    wmtk::io::write_mesh("127891_offset.msh", outputs.at("mesh"));
    // --8<-- [end:run]
}
