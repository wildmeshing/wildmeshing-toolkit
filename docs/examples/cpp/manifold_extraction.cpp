// The manifold extraction tutorial (docs/tutorials/manifold_extraction.md): two spheres that
// touch along a circle, made into a selection whose boundary is manifold. The same run as
// json/manifold_extraction.json and python/manifold_extraction.py.

#include <wmtk/components/manifold_extraction/manifold_extraction.hpp>
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
    wmtk::MeshMap meshes{{"spheres", wmtk::io::read_mesh(data / "double_sphere_hole.msh")}};
    auto [outputs, report] = wmtk::components::manifold_extraction::manifold_extraction(
        {{"input", "spheres"},
         {"tag_selection", "tag_0 | tag_1"},
         {"fill_tags", nlohmann::json::array()}},
        meshes);
    std::cout << report["before #t"] << " -> " << report["after #t"] << " tets\n";
    wmtk::io::write_mesh("double_sphere_manifold.msh", outputs.at("mesh"));
    // The boundary of the selection after the repair, a triangle mesh of its own.
    wmtk::io::write_mesh("double_sphere_manifold_surface.msh", outputs.at("surface"));
    // --8<-- [end:run]
}
