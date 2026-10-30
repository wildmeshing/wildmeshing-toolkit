// The repair-before-meshing workflow (docs/tutorials/workflows/repair-before-meshing.md): make the
// boundary of two touching spheres manifold, then remesh for quality, in memory. The same chain as
// json/workflow_repair/ and python/workflow_repair.py.

#include <wmtk/components/manifold_extraction/manifold_extraction.hpp>
#include <wmtk/components/simwild/simwild.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>

int main()
{
    // The example meshes: $WMTK_DATA, or data/models from the repository root.
    const char* env = std::getenv("WMTK_DATA");
    const std::filesystem::path data = env ? env : "data/models";

    // --8<-- [start:chain]
    namespace components = wmtk::components;
    wmtk::MeshMap meshes{{"spheres", wmtk::io::read_mesh(data / "double_sphere_hole.msh")}};

    // 1. Carve a thin tube of background around the circle where the spheres touch.
    auto [repaired, repair_report] = components::manifold_extraction::manifold_extraction(
        {{"input", "spheres"},
         {"tag_selection", "tag_0 | tag_1"},
         {"fill_tags", nlohmann::json::array()}},
        meshes);

    // 2. Remesh for quality; the regions keep their topology.
    wmtk::MeshMap next{{"repaired", repaired.at("mesh")}};
    auto [remeshed, report] = components::simwild::simwild(
        {{"input", nlohmann::json::array({"repaired"})},
         {"operation", "remeshing"},
         {"num_threads", 4}},
        next);
    std::cout << repair_report["after #t"] << " tets after the repair, " << report["#t"]
              << " after remeshing, max energy " << report["max_energy"] << "\n";
    wmtk::io::write_mesh("spheres_remeshed.msh", remeshed.at("mesh"));
    // --8<-- [end:chain]
}
