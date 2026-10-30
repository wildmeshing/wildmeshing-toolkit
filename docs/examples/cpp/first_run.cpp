// Getting started, first run (docs/getting-started/first-run.md): isotropic remeshing of the
// bunny from C++. The same run as json/first_run.json and python/first_run.py.

#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
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
    wmtk::MeshMap meshes{{"bunny", wmtk::io::read_mesh(data / "bunny.obj")}};
    auto [outputs, report] = wmtk::components::isotropic_remeshing::isotropic_remeshing(
        {{"input", "bunny"}, {"length_rel", 0.02}},
        meshes);
    wmtk::io::write_mesh("bunny_remeshed.msh", outputs.at("mesh"));
    // --8<-- [end:run]

    std::cout << "faces: "
              << outputs.at("mesh").cells.at(wmtk::CellType::Triangle).connectivity.rows() << "\n";
}
