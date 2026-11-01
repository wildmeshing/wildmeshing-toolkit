// The SimWild tutorial (docs/tutorials/simwild.md): two overlapping spheres meshed into one tet
// mesh with a region each, five overlapping closed curves into one triangle mesh, and a region
// edit followed by a remeshing. The same runs as json/simwild.json, json/simwild_curves.json,
// json/simwild_resolve_overlaps.json and python/simwild.py.

#include <wmtk/components/simwild/simwild.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <variant>

namespace {

// --8<-- [start:regions]
// How many cells each combination of regions has: "a & b" counts the cells in a and b and in no
// other region, "ambient" the cells in none.
std::map<std::string, int> regions(const wmtk::MeshData& mesh)
{
    const auto top = mesh.cells.count(wmtk::CellType::Tetrahedron) ? wmtk::CellType::Tetrahedron
                                                                   : wmtk::CellType::Triangle;
    const wmtk::MeshAttribute& tags = mesh.cell(top).attributes.at("tags");
    const auto& member = std::get<wmtk::DataMatrix<uint8_t>>(tags.values);
    std::map<std::string, int> counts;
    for (Eigen::Index i = 0; i < member.rows(); ++i) {
        std::string key;
        for (Eigen::Index j = 0; j < member.cols(); ++j) {
            if (member(i, j)) key += (key.empty() ? "" : " & ") + tags.labels[j];
        }
        ++counts[key];
    }
    return counts;
}
// --8<-- [end:regions]

void print(const std::map<std::string, int>& counts)
{
    for (const auto& [key, n] : counts) std::cout << "  " << key << ": " << n << "\n";
}

} // namespace

int main()
{
    // The example meshes: $WMTK_DATA, or data/models from the repository root.
    const char* env = std::getenv("WMTK_DATA");
    const std::filesystem::path data = env ? env : "data/models";

    // --8<-- [start:run]
    // Two copies of one sphere, the second moved 0.3 along x by a 4x4 matrix ([] is the identity).
    const wmtk::MeshMap inputs{{"sphere", wmtk::io::read_mesh(data / "sphere.obj")}};
    auto [spheres, report] = wmtk::components::simwild::simwild(
        nlohmann::json::parse(R"({
            "input": ["sphere", "sphere"],
            "input_names": ["a", "b"],
            "input_transform": [[], [[1, 0, 0, 0.3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]],
            "eps_rel": 0.01,
            "length_rel": 0.05,
            "num_threads": 4
        })"),
        inputs);
    std::cout << report["#t"] << " tets, max energy " << report["max_energy"] << "\n";
    print(regions(spheres.at("mesh")));
    wmtk::io::write_mesh("two_spheres.msh", spheres.at("mesh"));
    // --8<-- [end:run]

    // --8<-- [start:curves]
    wmtk::MeshMap curves;
    const std::vector<std::string> names = {"blob1", "blob2", "circle", "ellipse", "rectangle"};
    for (const std::string& name : names) {
        curves[name] = wmtk::io::read_mesh(data / ("triwild_" + name + ".obj"));
    }
    auto [shapes, shapes_report] = wmtk::components::simwild::simwild(
        {{"input", names}, {"input_names", names}, {"length_rel", 0.02}, {"num_threads", 4}},
        curves);
    std::cout << shapes_report["#t"] << " triangles\n";
    print(regions(shapes.at("mesh")));
    wmtk::io::write_mesh("five_shapes.msh", shapes.at("mesh"));
    // --8<-- [end:curves]

    // --8<-- [start:edit]
    // Split the overlap between a and b, then remesh: the split leaves slivers.
    auto [resolved, resolved_report] = wmtk::components::simwild::simwild(
        nlohmann::json::parse(R"({
            "operation": "resolve_overlaps",
            "input": ["m"],
            "resolve_overlaps_tags": [["a", "b"]]
        })"),
        {{"m", spheres.at("mesh")}});
    auto [remeshed, remeshed_report] = wmtk::components::simwild::simwild(
        {{"input", {"m"}}, {"eps_rel", 0.01}, {"num_threads", 4}},
        {{"m", resolved.at("mesh")}});
    std::cout << "max energy " << resolved_report["max_energy"] << " after the split, "
              << remeshed_report["max_energy"] << " after the remeshing\n";
    print(regions(remeshed.at("mesh")));
    // --8<-- [end:edit]
}
