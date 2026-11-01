// The isotropic remeshing tutorial (docs/tutorials/isotropic_remeshing.md): remesh the bunny to
// edges of 1% of its bounding box diagonal, the same run as json/isotropic_remeshing.json and
// python/isotropic_remeshing.py; then remesh a box keeping its sharp edges, as the Python example
// does in memory.

#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <utility>
#include <vector>

int main()
{
    // The example meshes: $WMTK_DATA, or data/models from the repository root.
    const char* env = std::getenv("WMTK_DATA");
    const std::filesystem::path data = env ? env : "data/models";
    using wmtk::components::isotropic_remeshing::isotropic_remeshing;

    // --8<-- [start:run]
    wmtk::MeshMap meshes{{"bunny", wmtk::io::read_mesh(data / "bunny.obj")}};
    auto [outputs, report] =
        isotropic_remeshing({{"input", "bunny"}, {"length_rel", 0.01}}, meshes);
    const wmtk::MeshData& remeshed = outputs.at("mesh");
    std::cout << meshes.at("bunny").V.rows() << " -> " << remeshed.V.rows() << " vertices\n";
    wmtk::io::write_mesh("bunny_remeshed.msh", remeshed);
    // --8<-- [end:run]

    // --8<-- [start:features]
    wmtk::MeshData box = wmtk::io::read_mesh(data / "piece_0.obj");
    // The sharp edges: those whose two triangles' normals are more than 30 degrees apart.
    const auto& F = box.cell(wmtk::CellType::Triangle).connectivity;
    std::map<std::pair<int64_t, int64_t>, std::vector<Eigen::Vector3d>> normals;
    for (Eigen::Index f = 0; f < F.rows(); ++f) {
        const Eigen::Vector3d a = box.V.row(F(f, 0)), b = box.V.row(F(f, 1)),
                              c = box.V.row(F(f, 2));
        for (int k = 0; k < 3; ++k) {
            const int64_t u = F(f, k), v = F(f, (k + 1) % 3);
            normals[{std::min(u, v), std::max(u, v)}].push_back((b - a).cross(c - a).normalized());
        }
    }
    std::vector<std::pair<int64_t, int64_t>> sharp;
    std::vector<int> degree(box.V.rows(), 0);
    for (const auto& [e, n] : normals) {
        if (n.size() == 2 && n[0].dot(n[1]) < std::cos(30.0 / 180.0 * 3.14159265358979)) {
            sharp.push_back(e);
            ++degree[e.first];
            ++degree[e.second];
        }
    }
    // The curves as a mesh on the vertex rows of the box: its edges, each with an id, and its
    // points, the corners (on three or more sharp edges), each with an id too.
    wmtk::MeshData curves;
    curves.V = box.V;
    wmtk::CellData& edges = curves.cells[wmtk::CellType::Edge];
    edges.connectivity.resize(sharp.size(), 2);
    wmtk::DataMatrix<int64_t> edge_id(sharp.size(), 1);
    for (size_t i = 0; i < sharp.size(); ++i) {
        edges.connectivity.row(i) << sharp[i].first, sharp[i].second;
        edge_id(i, 0) = int64_t(i);
    }
    edges.attributes["edge_id"].values = edge_id;
    std::vector<int64_t> corners;
    for (size_t v = 0; v < degree.size(); ++v) {
        if (degree[v] >= 3) corners.push_back(int64_t(v));
    }
    wmtk::CellData& points = curves.cells[wmtk::CellType::Point];
    points.connectivity.resize(corners.size(), 1);
    for (size_t i = 0; i < corners.size(); ++i) points.connectivity(i, 0) = corners[i];
    points.attributes["vertex_id"].values = points.connectivity; // a corner's id: its index

    const auto [box_outputs, box_report] = isotropic_remeshing(
        {{"input", "box"}, {"length_rel", 0.05}, {"features", "curves"}},
        {{"box", box}, {"curves", curves}});
    std::cout << sharp.size() << " sharp edges -> "
              << box_outputs.at("edges").cell(wmtk::CellType::Edge).connectivity.rows()
              << " feature edges\n";
    // --8<-- [end:features]
}
