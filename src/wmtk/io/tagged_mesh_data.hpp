#pragma once
#include <algorithm>
#include <array>
#include <set>
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/io/DataWriter.hpp>

namespace wmtk::io {
inline MeshData geometry_data(const MatrixXd& V, const MatrixXi& C, CellType type)
{
    MeshData data;
    data.V = DataMatrix<double>::Zero(V.rows(), 3);
    data.V.leftCols(V.cols()) = V;
    data.cells[type].connectivity = C.cast<int64_t>();
    data.validate();
    return data;
}
template <typename Vertex, typename Cell>
MeshData
geometry_data(const std::vector<Vertex>& vertices, const std::vector<Cell>& cells, CellType type)
{
    MeshData data;
    data.V = DataMatrix<double>::Zero(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); ++i)
        for (int j = 0; j < vertices[i].size(); ++j) data.V(i, j) = vertices[i][j];
    auto& C = data.cells[type].connectivity;
    C.resize(cells.size(), cell_size(type));
    for (size_t i = 0; i < cells.size(); ++i)
        for (int j = 0; j < C.cols(); ++j) C(i, j) = cells[i][j];
    data.validate();
    return data;
}
template <typename Point>
MeshData feature_data(
    const std::vector<std::array<Point, 2>>& segments,
    const std::vector<Point>& anchors)
{
    std::map<std::array<double, 3>, int64_t> rows;
    MeshData data;
    std::vector<std::array<double, 3>> vertices;
    auto vertex = [&](const Point& p) {
        std::array<double, 3> key{{0, 0, 0}};
        for (int i = 0; i < p.size(); ++i) key[i] = p[i];
        auto [it, added] = rows.emplace(key, rows.size());
        if (added) vertices.push_back(key);
        return it->second;
    };
    auto& E = data.cells[CellType::Edge].connectivity;
    E.resize(segments.size(), 2);
    for (size_t i = 0; i < segments.size(); ++i) {
        E(i, 0) = vertex(segments[i][0]);
        E(i, 1) = vertex(segments[i][1]);
    }
    auto& P = data.cells[CellType::Point].connectivity;
    P.resize(anchors.size(), 1);
    for (size_t i = 0; i < anchors.size(); ++i) P(i, 0) = vertex(anchors[i]);
    data.V.resize(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); ++i)
        for (int j = 0; j < 3; ++j) data.V(i, j) = vertices[i][j];
    data.validate();
    return data;
}
template <typename Mesh, typename TupleRange, typename Tags>
void collect_tags(Mesh& mesh, MeshMap& outputs, CellType type, const TupleRange& cells, Tags tags)
{
    // A result without cells has no block yet: the exporter only creates the ones it fills.
    auto& block = outputs.at("mesh").cells[type];
    if (block.connectivity.cols() != cell_size(type)) block.connectivity.resize(0, cell_size(type));
    // An unnamed tag is called tag_<id>, unless that name is taken, in which case '_' is
    // appended as SimWildMesh::write_msh does; labels must be unique.
    std::set<std::string> named, labelled;
    for (const auto& [id, name] : mesh.m_tag_id_to_name) named.insert(name);
    std::vector<std::string> labels;
    for (size_t j = 0; j < mesh.m_tags_count; ++j) {
        const auto it = mesh.m_tag_id_to_name.find(j);
        const bool unnamed = it == mesh.m_tag_id_to_name.end();
        std::string label = unnamed ? "tag_" + std::to_string(j) : it->second;
        while (labelled.count(label) || (unnamed && named.count(label))) label += "_";
        labelled.insert(label);
        labels.push_back(label);
    }
    DataMatrix<uint8_t> values = DataMatrix<uint8_t>::Zero(cells.size(), labels.size());
    for (size_t i = 0; i < cells.size(); ++i)
        for (size_t j = 0; j < labels.size(); ++j) values(i, j) = tags(cells[i]).count(j) ? 1 : 0;
    // The fixed tags field replaces per-group named scalar arrays.
    for (size_t j = 0; j < labels.size(); ++j) {
        block.attributes.erase(labels[j]);
        block.attributes.erase("tag_" + std::to_string(j));
    }
    // Cells without a tag are the background the component built. They are labelled
    // explicitly: the IO infers nothing from an empty row.
    if (std::find(labels.begin(), labels.end(), "ambient") == labels.end()) {
        const auto empty = (values.template cast<int>().rowwise().sum().array() == 0).eval();
        if (labels.empty() || empty.any()) {
            values.conservativeResize(Eigen::NoChange, values.cols() + 1);
            values.col(values.cols() - 1) = empty.template cast<uint8_t>();
            labels.push_back("ambient");
        }
    }
    block.attributes["tags"] = {std::move(values), std::move(labels)};
}
// The faces whose input set is not empty, as the triangle block of outputs["mesh"]: rows of its
// V (the vertices in get_vertices() order, as compact_slot_outputs leaves them), one tags
// column per name (bit j of the input set is names[j]), each row oriented as the face's input
// was. Nothing is written without names.
template <typename Mesh>
void collect_face_tags(const Mesh& mesh, MeshMap& outputs, const std::vector<std::string>& names)
{
    if (names.empty()) return;
    std::vector<int64_t> row(mesh.vert_capacity(), -1);
    int64_t n = 0;
    for (const auto& v : mesh.get_vertices()) row[v.vid(mesh)] = n++;
    std::vector<std::array<int64_t, 3>> faces;
    std::vector<uint16_t> sets;
    for (const auto& f : mesh.get_faces()) {
        const auto& attr = mesh.m_face_attribute[f.fid(mesh)];
        if (attr.input_set() == 0) continue;
        auto vids = mesh.get_face_vids(f);
        std::sort(vids.begin(), vids.end());
        if (attr.orientation_along(vids) < 0) std::swap(vids[1], vids[2]);
        faces.push_back({{row[vids[0]], row[vids[1]], row[vids[2]]}});
        sets.push_back(attr.input_set());
    }
    auto& block = outputs.at("mesh").cells[CellType::Triangle];
    block.connectivity.resize(faces.size(), 3);
    DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Zero(faces.size(), names.size());
    for (size_t i = 0; i < faces.size(); ++i) {
        for (int k = 0; k < 3; ++k) block.connectivity(i, k) = faces[i][k];
        for (size_t j = 0; j < names.size(); ++j) member(i, j) = (sets[i] >> j) & 1;
    }
    block.attributes["tags"] = {std::move(member), names};
}
} // namespace wmtk::io
