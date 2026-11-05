#pragma once
#include <wmtk/SurfaceTagAttributes.h>
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
// The envelope a component hands on -- outputs["envelope"] on the memory route,
// <output>_envelope.msh on the file route -- is the REFERENCE it held its tags to: the geometry its
// envelopes, tubes and anchors were built around (the input, or the simplified input, per
// envelope_reference), labelled as its output mesh labels what it held there. In 3D a triangle
// block (an input surface's face tag, a region's tet tag) and an edge block (the feature curves,
// feature_<i>), in 2D an edge block, each with one `tags` column per label; the input points as a
// point block without tags, since the mesh writes nothing for them. No eps: a receiver builds its
// own envelopes around the reference at its own eps, label by label, so that every stage of a
// chain holds its tags within one eps of the first component's input. Rebuilt from the mesh it
// receives, the bound would grow by one eps per stage.
//
// append_envelope_block() adds one kind of it: the `type` cells of `block` (geometry_data() of
// the reference's cells), on vertices of their own after the envelope's, row r labelled by the
// bits of sets[r] (bit j: names[j]); without names, unlabelled. A block without cells adds
// nothing. One block per type.
inline void append_envelope_block(
    MeshData& envelope,
    const MeshData& block,
    const CellType type,
    const std::vector<InputSet>& sets,
    const std::vector<std::string>& names)
{
    const auto& cells = block.cell(type).connectivity;
    if (cells.rows() == 0) return;
    if (envelope.cells.count(type))
        throw std::runtime_error("An envelope has one block of each cell type");
    if (!names.empty() && sets.size() != size_t(cells.rows()))
        throw std::runtime_error("An envelope needs one label set per cell");
    const Eigen::Index base = envelope.V.rows();
    envelope.V.conservativeResize(base + block.V.rows(), 3);
    envelope.V.bottomRows(block.V.rows()) = block.V;
    auto& out = envelope.cells[type];
    out.connectivity = (cells.array() + int64_t(base)).matrix();
    if (names.empty()) return;
    DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Zero(cells.rows(), names.size());
    for (Eigen::Index r = 0; r < cells.rows(); ++r)
        for (size_t j = 0; j < names.size() && j < size_t(SurfaceTagAttributes::max_inputs); ++j)
            member(r, j) = (sets[r] >> j) & 1;
    out.attributes["tags"] = {std::move(member), names};
}
// The reference's input points, as the envelope's point block, without tags.
template <typename Point>
void append_envelope_points(MeshData& envelope, const std::vector<Point>& points)
{
    std::vector<std::array<int64_t, 1>> cells(points.size());
    for (size_t i = 0; i < points.size(); ++i) cells[i] = {{int64_t(i)}};
    append_envelope_block(
        envelope,
        geometry_data(points, cells, CellType::Point),
        CellType::Point,
        {},
        {});
}
// Whether `envelope` is a labelled reference (append_envelope_block()), as producers write it,
// rather than bare geometry: its triangles (3D) or edges (2D) carry `tags`. Bare geometry, an
// `envelope` the caller built, is held to as a whole, as before the labels.
inline bool is_labelled_envelope(const MeshData& envelope, const bool is_3d)
{
    const auto it = envelope.cells.find(is_3d ? CellType::Triangle : CellType::Edge);
    return it != envelope.cells.end() && it->second.attributes.count("tags");
}
// The labels of tags 0 to count - 1, named by `names` (id -> name): a tag's name, or tag_<id>
// without one, unless that name is taken, in which case '_' is appended as SimWildMesh::write_msh
// does; labels must be unique.
inline std::vector<std::string> tag_labels(
    const std::map<int64_t, std::string>& names,
    const size_t count)
{
    std::set<std::string> named, labelled;
    for (const auto& [id, name] : names) named.insert(name);
    std::vector<std::string> labels;
    for (size_t j = 0; j < count; ++j) {
        const auto it = names.find(int64_t(j));
        const bool unnamed = it == names.end();
        std::string label = unnamed ? "tag_" + std::to_string(j) : it->second;
        while (labelled.count(label) || (unnamed && named.count(label))) label += "_";
        labelled.insert(label);
        labels.push_back(label);
    }
    return labels;
}
// The labels of an application's inputs, one per input: input_names, the rest tag_<i>, by the
// rule of tag_labels(). What tetwild's face tags and triwild's edge tags name the inputs by, so
// that an input's tags are named alike in every component.
inline std::vector<std::string> input_labels(
    const std::vector<std::string>& input_names,
    const size_t n_inputs)
{
    std::map<int64_t, std::string> names;
    for (size_t i = 0; i < input_names.size() && i < n_inputs; ++i)
        names[int64_t(i)] = input_names[i];
    return tag_labels(names, n_inputs);
}
template <typename Mesh, typename TupleRange, typename Tags>
void collect_tags(Mesh& mesh, MeshMap& outputs, CellType type, const TupleRange& cells, Tags tags)
{
    // A result without cells has no block yet: the exporter only creates the ones it fills.
    auto& block = outputs.at("mesh").cells[type];
    if (block.connectivity.cols() != cell_size(type)) block.connectivity.resize(0, cell_size(type));
    std::vector<std::string> labels = tag_labels(mesh.m_tag_id_to_name, mesh.m_tags_count);
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
// The tracked simplices -- `type` Triangle: a tet mesh's faces; Edge: a triangle mesh's edges --
// whose input set is not empty, as the `type` block of outputs["mesh"]: rows of its V (the
// vertices in get_vertices() order, as compact_slot_outputs leaves them), one tags column per
// name (bit j of the input set is names[j]), each row oriented as the simplex's input was.
// Nothing is written without names.
template <CellType type, typename Mesh>
void collect_input_set_tags(
    const Mesh& mesh,
    MeshMap& outputs,
    const std::vector<std::string>& names)
{
    static_assert(type == CellType::Triangle || type == CellType::Edge);
    constexpr size_t N = type == CellType::Triangle ? 3 : 2;
    if (names.empty()) return;
    std::vector<int64_t> row(mesh.vert_capacity(), -1);
    int64_t n = 0;
    for (const auto& v : mesh.get_vertices()) row[v.vid(mesh)] = n++;
    std::vector<std::array<int64_t, N>> simplices;
    std::vector<InputSet> sets;
    const InputSet mask = names.size() >= size_t(SurfaceTagAttributes::max_inputs)
                              ? ~InputSet(0)
                              : (InputSet(1) << names.size()) - 1;
    const auto add = [&](const SurfaceTagAttributes& attr, std::array<size_t, N> vids) {
        // Bits past the names are not these tags -- simwild's region labels -- and are not
        // written.
        const InputSet set = attr.input_set() & mask;
        if (set == 0) return;
        std::sort(vids.begin(), vids.end());
        if (attr.orientation_along(vids) < 0) std::swap(vids[N - 2], vids[N - 1]);
        std::array<int64_t, N> r;
        for (size_t k = 0; k < N; ++k) r[k] = row[vids[k]];
        simplices.push_back(r);
        sets.push_back(set);
    };
    if constexpr (type == CellType::Triangle) {
        for (const auto& f : mesh.get_faces())
            add(mesh.m_face_attribute[f.fid(mesh)], mesh.get_face_vids(f));
    } else {
        for (const auto& e : mesh.get_edges())
            add(mesh.m_edge_attribute[e.eid(mesh)], mesh.get_edge_vids(e));
    }
    auto& block = outputs.at("mesh").cells[type];
    block.connectivity.resize(simplices.size(), N);
    DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Zero(simplices.size(), names.size());
    for (size_t i = 0; i < simplices.size(); ++i) {
        for (size_t k = 0; k < N; ++k) block.connectivity(i, k) = simplices[i][k];
        for (size_t j = 0; j < names.size(); ++j) member(i, j) = (sets[i] >> j) & 1;
    }
    block.attributes["tags"] = {std::move(member), names};
}
// The tet edges tagged `feature` (TetOptimizerMesh::FeatureEdgeAttributes: the feature curves the
// application was given) whose feature input set is not empty, as the Edge block of
// outputs["mesh"], on the rows collect_input_set_tags() uses: one tags column per name (bit j of
// the set is names[j], the input_edges file j). A curve carries no orientation, so a row is its
// edge's vertices in ascending order. The order2 kind is not written: a reader derives it again
// from the faces (tag_order2_edges()). Nothing is written without names or edge tracking.
template <typename Mesh>
void collect_feature_edge_tags(
    const Mesh& mesh,
    MeshMap& outputs,
    const std::vector<std::string>& names)
{
    using EdgeTags = typename Mesh::FeatureEdgeAttributes;
    if (names.empty() || !mesh.m_track_feature_edges) return;
    std::vector<int64_t> row(mesh.vert_capacity(), -1);
    int64_t n = 0;
    for (const auto& v : mesh.get_vertices()) row[v.vid(mesh)] = n++;
    const InputSet mask = names.size() >= size_t(SurfaceTagAttributes::max_inputs)
                              ? ~InputSet(0)
                              : (InputSet(1) << names.size()) - 1;
    std::vector<std::array<int64_t, 2>> edges;
    std::vector<InputSet> sets;
    for (const auto& e : mesh.get_edges()) {
        const auto& attr = mesh.m_feature_edge_attribute[e.eid(mesh)];
        if (!attr.has(EdgeTags::feature)) continue;
        const InputSet set = attr.inputs(EdgeTags::feature) & mask;
        if (set == 0) continue;
        const size_t a = e.vid(mesh), b = e.switch_vertex(mesh).vid(mesh);
        edges.push_back({{row[std::min(a, b)], row[std::max(a, b)]}});
        sets.push_back(set);
    }
    auto& block = outputs.at("mesh").cells[CellType::Edge];
    block.connectivity.resize(edges.size(), 2);
    DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Zero(edges.size(), names.size());
    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t k = 0; k < 2; ++k) block.connectivity(i, k) = edges[i][k];
        for (size_t j = 0; j < names.size(); ++j) member(i, j) = (sets[i] >> j) & 1;
    }
    block.attributes["tags"] = {std::move(member), names};
}
} // namespace wmtk::io
