// The multi-region workflow (docs/tutorials/workflows/regions.md): two overlapping spheres meshed
// into one tet mesh with a region each, the overlap given to one of them, a thin gap carved around
// the other, and a final remeshing, in memory. The same pipeline as python/workflow_regions.py.

#include <wmtk/components/simwild/simwild.hpp>
#include <wmtk/components/topological_offset/topological_offset.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <variant>

namespace {

// How many cells each combination of regions has (see cpp/simwild.cpp).
std::map<std::string, int> regions(const wmtk::MeshData& mesh)
{
    const wmtk::MeshAttribute& tags = mesh.cell(wmtk::CellType::Tetrahedron).attributes.at("tags");
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

void print(const std::string& step, const std::map<std::string, int>& counts)
{
    std::cout << step << "\n";
    for (const auto& [key, n] : counts) std::cout << "  " << key << ": " << n << "\n";
}

} // namespace

int main()
{
    // The example meshes: $WMTK_DATA, or data/models from the repository root.
    const char* env = std::getenv("WMTK_DATA");
    const std::filesystem::path data = env ? env : "data/models";
    namespace simwild = wmtk::components::simwild;
    namespace topological_offset = wmtk::components::topological_offset;
    using nlohmann::json;

    // --8<-- [start:run]
    // 1. Two copies of one sphere, the second moved 0.3 along x: regions a, b and their overlap.
    auto [m1, r1] = simwild::simwild(
        json::parse(R"({
            "input": ["sphere", "sphere"],
            "input_names": ["a", "b"],
            "input_transform": [[], [[1, 0, 0, 0.3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]],
            "eps_rel": 0.01,
            "length_rel": 0.05,
            "num_threads": 4
        })"),
        {{"sphere", wmtk::io::read_mesh(data / "sphere.obj")}});
    print("1. simwild", regions(m1.at("mesh")));

    // 2. The overlap goes to b: its cells leave a and stay in b.
    auto [m2, r2] = simwild::simwild(
        json::parse(R"({
            "operation": "replace_tags",
            "input": ["m"],
            "replace_tags_in": ["a & b"],
            "replace_tags_out": ["b"]
        })"),
        {{"m", m1.at("mesh")}});
    print("2. replace_tags", regions(m2.at("mesh")));

    // 3. A gap of thickness 0.01 around a, carved out of b and the background.
    auto [m3, r3] = topological_offset::topological_offset(
        json::parse(R"({
            "input": "m",
            "offset_selection": "a",
            "offset_output_tags": ["gap"],
            "target_distance": 0.01,
            "optimize_offset": false,
            "num_threads": 4
        })"),
        {{"m", m2.at("mesh")}});
    print("3. topological_offset", regions(m3.at("mesh")));
    wmtk::io::write_mesh("regions_offset.msh", m3.at("mesh"));

    // 4. Improve the elements the offset cut; a thin envelope keeps the gap's walls in place.
    auto [m4, r4] = simwild::simwild(
        {{"input", {"m"}}, {"eps_rel", 0.001}, {"num_threads", 4}},
        {{"m", m3.at("mesh")}});
    print("4. simwild remeshing", regions(m4.at("mesh")));
    std::cout << r4["#t"] << " tets, max energy " << r4["max_energy"] << "\n";
    wmtk::io::write_mesh("regions_final.msh", m4.at("mesh"));
    // --8<-- [end:run]
}
