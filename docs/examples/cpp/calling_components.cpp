// Getting started, calling components (docs/getting-started/calling-components.md): MeshData,
// the memory route, reports and files, in C++.

#include <wmtk/components/qslim/qslim.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>

int main()
{
    // The example meshes: $WMTK_DATA, or data/models from the repository root.
    const char* env = std::getenv("WMTK_DATA");
    const std::filesystem::path data = env ? env : "data/models";

    // --8<-- [start:meshdata]
    // Two triangles of the unit square: row-major positions with three columns, and cell blocks
    // by type.
    wmtk::MeshData square;
    square.V.resize(4, 3);
    square.V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0;
    auto& triangles = square.cells[wmtk::CellType::Triangle].connectivity;
    triangles.resize(2, 3);
    triangles << 0, 1, 2, 0, 2, 3;
    square.validate();
    // --8<-- [end:meshdata]

    // --8<-- [start:routes]
    const wmtk::MeshMap meshes{{"sphere", wmtk::io::read_mesh(data / "sphere.obj")}};
    auto [outputs, report] =
        wmtk::components::qslim::qslim({{"input", "sphere"}, {"target_rel", 0.5}}, meshes);
    std::cout << report["vertices"] << " vertices\n";
    wmtk::io::write_mesh("sphere_half.msh", outputs.at("mesh"));
    // --8<-- [end:routes]
}
