#include <cmath>
#include <set>
#include <unordered_map>
#include <wmtk/io/ComponentIO.hpp>
#include "prismatic_mesh.hpp"

namespace wmtk::components::prismatic_mesh {
namespace {
void require(bool ok, const std::string& message)
{
    if (!ok) throw std::runtime_error("prismatic_mesh: " + message);
}
int64_t integer(long double v, int64_t lo, int64_t hi, const char* name)
{
    require(
        std::isfinite(v) && v >= lo && v <= hi && std::floor(v) == v,
        std::string("invalid ") + name);
    return static_cast<int64_t>(v);
}
} // namespace
PrismaticMeshInput load_prismatic_data(const MeshData& data)
{
    data.validate();
    const auto& block = data.cell(CellType::Tetrahedron);
    require(block.connectivity.rows() > 0, "expected tetrahedra");
    io::ComponentIO::attributes(
        data.vertex_attributes,
        {"labels",
         "vid",
         "corr_input_vid",
         "component_id",
         "singular_component",
         "optimal_normal",
         "target_position",
         "sizing_scalar",
         "sizing_field",
         "input_distance",
         "distance_over_target",
         "front_error_bars",
         "band_force_n",
         "amips_force_n",
         "offset_foldover",
         "front_last_change"},
        {"corr_input_vid"});
    for (const auto& [type, b] : data.cells) {
        require(
            type == CellType::Tetrahedron || type == CellType::Triangle || type == CellType::Edge,
            "expected a tetrahedral construction mesh");
        io::ComponentIO::attributes(
            b.attributes,
            {"tags",
             "tag_0",
             "offset_tag",
             "offset",
             "tau22",
             "amips",
             "tet_energy",
             "amips_term",
             "band_term",
             "band_gap_bars",
             "plastic"});
    }
    for (const auto& [name, a] : data.vertex_attributes) {
        const int width = (name == "optimal_normal" || name == "target_position") ? 3 : 1;
        require(a.cols() == width, "invalid attribute width for " + name);
    }
    const int n = int(data.V.rows()), m = int(block.connectivity.rows());
    auto column =
        [&](const MeshAttributes& fields, const std::string& key, int count, int64_t fallback) {
            auto it = fields.find(key);
            if (it == fields.end()) return DataMatrix<int64_t>::Constant(count, 1, fallback).eval();
            require(it->second.cols() == 1, "expected scalar " + key);
            return it->second.integers();
        };
    auto input = column(block.attributes, "tag_0", m, 0),
         offset = column(block.attributes, "offset_tag", m, -1);
    const auto tags_it = block.attributes.find("tags");
    if (tags_it != block.attributes.end()) {
        const auto& a = tags_it->second;
        const auto tags = a.integers();
        require(a.labels.size() == size_t(tags.cols()), "tags need labels");
        require((tags.array() >= 0).all() && (tags.array() <= 1).all(), "tags must be binary");
        bool has_input = block.attributes.count("tag_0"),
             has_offset = block.attributes.count("offset_tag");
        for (int j = 0; j < tags.cols(); ++j) {
            has_input = has_input || a.labels[j] == "tag_0";
            has_offset = has_offset || a.labels[j] == "offset";
            if (a.labels[j] == "tag_0") input = tags.col(j);
            if (a.labels[j] == "offset" && !block.attributes.count("offset_tag"))
                offset = tags.col(j);
        }
        require(has_input && has_offset, "missing input or offset region tag");
    } else
        require(
            block.attributes.count("tag_0") && block.attributes.count("offset_tag"),
            "missing cell tags");
    auto ids = column(data.vertex_attributes, "vid", n, -1),
         corr = column(data.vertex_attributes, "corr_input_vid", n, -1),
         labels = column(data.vertex_attributes, "labels", n, -1);
    if (!data.vertex_attributes.count("vid"))
        for (int i = 0; i < n; ++i) ids(i, 0) = i;
    if (!data.vertex_attributes.count("labels")) {
        for (int i = 0; i < m; ++i)
            if (input(i, 0) == 1)
                for (int j = 0; j < 4; ++j) labels(block.connectivity(i, j), 0) = 1;
        for (int i = 0; i < n; ++i)
            if (corr(i, 0) >= 0) {
                require(labels(i, 0) != 1, "correspondence on input-volume vertex");
                labels(i, 0) = 2;
            }
    }
    PrismaticMeshInput result;
    result.vertices = data.V;
    result.tetrahedra = io::ComponentIO::indices(block.connectivity);
    std::vector<std::array<size_t, 4>> tets(m);
    std::set<std::array<size_t, 4>> seen;
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < 4; ++j) tets[i][j] = result.tetrahedra(i, j);
        auto key = tets[i];
        std::sort(key.begin(), key.end());
        require(seen.insert(key).second, "duplicate tetrahedron");
    }
    auto vectorize = [](const DataMatrix<int64_t>& a) {
        return std::vector<int64_t>(a.data(), a.data() + a.size());
    };
    auto input_v = vectorize(input), offset_v = vectorize(offset), ids_v = vectorize(ids),
         corr_v = vectorize(corr), labels_v = vectorize(labels);
    std::unordered_map<int64_t, size_t> id_to_row;
    result.input_to_offset_vertices.resize(n);
    result.corr_input_vertex.resize(n, -1);
    for (int i = 0; i < n; ++i) {
        const int label = static_cast<int>(integer(labels_v[i], -1, 2, "labels"));
        const auto id = ids_v[i], target = corr_v[i];
        require(id >= 0 && target >= -1, "invalid source vertex ID or correspondence");
        require(id_to_row.emplace(id, i).second, "duplicate vid");
        result.vertex_tags.push_back(label == 0 ? -1 : label);
        result.source_vertex_ids.push_back(id);
        result.corr_input_vid.push_back(target);
        if (label == 1) result.input_vertices.push_back(i);
        if (label == 2) result.offset_vertices.push_back(i);
        require(label == 2 || target == -1, "correspondence on a non-offset vertex");
    }
    for (size_t i : result.offset_vertices) {
        const auto it = id_to_row.find(result.corr_input_vid[i]);
        require(it != id_to_row.end(), "offset vertex has missing or unknown input correspondence");
        require(
            result.vertex_tags[it->second] == 1,
            "correspondence target is not an input vertex");
        result.corr_input_vertex[i] = static_cast<int64_t>(it->second);
        result.input_to_offset_vertices[it->second].push_back(i);
    }

    for (int i = 0; i < m; ++i) {
        result.input_cells.push_back(static_cast<int>(integer(input_v[i], 0, 1, "tag_0")));
        const auto band = integer(offset_v[i], -1, 1, "offset_tag");
        result.offset_tet_tags.push_back(band == 1 ? 1 : -1);
    }
    result.mesh = std::make_unique<TetMesh>();
    result.mesh->init_with_isolated_vertices(n, tets);
    return result;
}
} // namespace wmtk::components::prismatic_mesh
