// The sizing mesh workflow (docs/tutorials/workflows/sizing-mesh.md): TriWild on a circle, with a
// target edge length that grows from left to right. The same run as python/workflow_sizing_mesh.py.

#include <wmtk/components/triwild/triwild.hpp>
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
    wmtk::MeshData circle = wmtk::io::read_mesh(data / "triwild_circle.obj");
    const double size = circle.V.leftCols(2).maxCoeff() - circle.V.leftCols(2).minCoeff();
    const double lo = circle.V.leftCols(2).minCoeff() - 0.2 * size;
    const double hi = circle.V.leftCols(2).maxCoeff() + 0.2 * size;

    // A square around the circle, past the background mesh TriWild builds around it, two
    // triangles, whose vertex field target_length grows from 1% of the circle's size on its left
    // side to 6% on its right, in the input's units.
    wmtk::MeshData sizing;
    sizing.V.resize(4, 3);
    sizing.V << lo, lo, 0, hi, lo, 0, hi, hi, 0, lo, hi, 0;
    sizing.cells[wmtk::CellType::Triangle].connectivity.resize(2, 3);
    sizing.cells[wmtk::CellType::Triangle].connectivity << 0, 1, 2, 0, 2, 3;
    wmtk::DataMatrix<double> length(4, 1);
    length << 0.01 * size, 0.06 * size, 0.06 * size, 0.01 * size;
    sizing.vertex_attributes["target_length"].values = length;

    wmtk::MeshMap meshes{{"circle", circle}, {"sizing", sizing}};
    auto [outputs, report] = wmtk::components::triwild::triwild(
        {{"input", nlohmann::json::array({"circle"})},
         {"sizing_mesh", "sizing"},
         {"filter", "input"},
         {"stop_energy", 10},
         {"num_threads", 0}},
        meshes);

    // Every edge against the field at its midpoint: the share longer than 4/3 of it, those a
    // split would cut. The field is a largest length; shorter is fine.
    const wmtk::MeshData& mesh = outputs.at("mesh");
    const auto& F = mesh.cell(wmtk::CellType::Triangle).connectivity;
    size_t total = 0, too_long = 0;
    for (Eigen::Index f = 0; f < F.rows(); ++f) {
        for (int k = 0; k < 3; ++k) {
            const Eigen::RowVector3d a = mesh.V.row(F(f, k)), b = mesh.V.row(F(f, (k + 1) % 3));
            const double t = (0.5 * (a.x() + b.x()) - lo) / (hi - lo);
            const double field = (0.01 + 0.05 * t) * size;
            ++total; // each interior edge twice, as both its triangles see it
            if ((a - b).norm() > 4. / 3. * field) ++too_long;
        }
    }
    std::cout << F.rows() << " triangles, " << 100. * too_long / total
              << "% of the edges past 4/3 of the field\n";
    wmtk::io::write_mesh("circle.msh", mesh);
    // --8<-- [end:run]
}
