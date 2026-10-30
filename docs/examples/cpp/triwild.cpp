// The TriWild tutorial (docs/tutorials/triwild.md): triangulate five overlapping closed curves, each
// an input with a name. The same run as json/triwild.json and python/triwild.py.

#include <wmtk/components/triwild/triwild.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

int main()
{
    // The example meshes: $WMTK_DATA, or data/models from the repository root.
    const char* env = std::getenv("WMTK_DATA");
    const std::filesystem::path data = env ? env : "data/models";

    // --8<-- [start:run]
    const std::vector<std::string> names = {"blob1", "blob2", "circle", "ellipse", "rectangle"};
    wmtk::MeshMap curves;
    for (const std::string& name : names) {
        curves[name] = wmtk::io::read_mesh(data / ("triwild_" + name + ".obj"));
    }
    auto [outputs, report] = wmtk::components::triwild::triwild(
        {{"input", names},
         {"input_names", names},
         {"filter", "input"},
         {"stop_energy", 10},
         {"num_threads", 4}},
        curves);
    const wmtk::MeshData& mesh = outputs.at("mesh");
    const wmtk::CellData& triangles = mesh.cell(wmtk::CellType::Triangle);
    std::cout << mesh.V.rows() << " vertices, " << triangles.connectivity.rows()
              << " triangles, max energy " << report["max_energy"].get<double>() << "\n";
    // One column per input: 1 where the triangle lies inside that curve.
    const wmtk::MeshAttribute& tags = triangles.attributes.at("tags");
    const wmtk::DataMatrix<int64_t> inside = tags.integers();
    for (size_t j = 0; j < tags.labels.size(); ++j) {
        std::cout << "  " << tags.labels[j] << ": " << inside.col(j).sum() << " triangles\n";
    }
    wmtk::io::write_mesh("shapes.msh", mesh);
    // --8<-- [end:run]
}
