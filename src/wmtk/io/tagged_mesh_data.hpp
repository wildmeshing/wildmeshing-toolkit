#pragma once
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/io/DataWriter.hpp>

namespace wmtk::components::simwild {
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
    auto& block = outputs.at("mesh").cells.at(type);
    std::vector<std::string> labels;
    for (size_t j = 0; j < mesh.m_tags_count; ++j) {
        const auto it = mesh.m_tag_id_to_name.find(j);
        labels.push_back(
            it == mesh.m_tag_id_to_name.end() ? "tag_" + std::to_string(j) : it->second);
    }
    DataMatrix<uint8_t> values = DataMatrix<uint8_t>::Zero(cells.size(), labels.size());
    for (size_t i = 0; i < cells.size(); ++i)
        for (size_t j = 0; j < labels.size(); ++j) values(i, j) = tags(cells[i]).count(j) ? 1 : 0;
    // The fixed tags field replaces per-group named scalar arrays.
    for (size_t j = 0; j < labels.size(); ++j) {
        block.attributes.erase(labels[j]);
        block.attributes.erase("tag_" + std::to_string(j));
    }
    if (labels.empty()) {
        labels = {"ambient"};
        values = DataMatrix<uint8_t>::Ones(cells.size(), 1);
    }
    block.attributes["tags"] = {std::move(values), std::move(labels)};
}
} // namespace wmtk::components::simwild
