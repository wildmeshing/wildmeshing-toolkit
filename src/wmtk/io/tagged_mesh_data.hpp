#pragma once
#include <wmtk/SurfaceTagAttributes.h>
#include <algorithm>
#include <array>
#include <functional>
#include <map>
#include <set>
#include <variant>
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/io/DataWriter.hpp>
#include <wmtk/utils/FeatureInputs.hpp>

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
// The reference's input points, as the envelope's point block: point i labelled by the bit
// labels[i] of names, as the mesh's point block labels it (collect_point_tags()); without labels,
// untagged.
template <typename Point>
void append_envelope_points(
    MeshData& envelope,
    const std::vector<Point>& points,
    const std::vector<int>& labels = {},
    const std::vector<std::string>& names = {})
{
    std::vector<std::array<int64_t, 1>> cells(points.size());
    for (size_t i = 0; i < points.size(); ++i) cells[i] = {{int64_t(i)}};
    const bool labelled = !names.empty() && labels.size() == points.size();
    std::vector<InputSet> sets;
    if (labelled) {
        for (const int l : labels)
            sets.push_back(l >= 0 && l < SurfaceTagAttributes::max_inputs ? InputSet(1) << l : 0);
    }
    append_envelope_block(
        envelope,
        geometry_data(points, cells, CellType::Point),
        CellType::Point,
        sets,
        labelled ? names : std::vector<std::string>());
}
// Whether `envelope` is a labelled reference (append_envelope_block()), as producers write it,
// rather than bare geometry: its triangles (3D) or edges (2D) carry `tags`. Bare geometry, an
// `envelope` the caller built, is held to as a whole, as before the labels.
inline bool is_labelled_envelope(const MeshData& envelope, const bool is_3d)
{
    const auto it = envelope.cells.find(is_3d ? CellType::Triangle : CellType::Edge);
    return it != envelope.cells.end() && it->second.attributes.count("tags");
}
/// The fields a tagged mesh's reader reads on a cell block: `tags`, and in a build with
/// WMTK_PER_INPUT_ORIENTATION `orientation`, the per-input counts collect_input_set_tags() writes
/// beside them there (read_orientation()). A build without the option drops `orientation` as it
/// drops every annotation (ComponentIO::dropped_fields()).
inline const std::set<std::string>& tag_fields()
{
    static const std::set<std::string> fields = SurfaceTagAttributes::per_input_orientation
                                                    ? std::set<std::string>{"tags", "orientation"}
                                                    : std::set<std::string>{"tags"};
    return fields;
}
/// `mesh` with its tags only: every vertex field and every cell field but tag_fields() dropped.
inline MeshData tags_only(MeshData mesh)
{
    mesh.vertex_attributes.clear();
    for (auto& entry : mesh.cells) {
        auto& attributes = entry.second.attributes;
        for (auto it = attributes.begin(); it != attributes.end();)
            it = tag_fields().count(it->first) ? std::next(it) : attributes.erase(it);
    }
    return mesh;
}
/**
 * @brief A tagged mesh's lower block's `orientation` (collect_input_set_tags()), in a build with
 * WMTK_PER_INPUT_ORIENTATION: per row and `tags` column, that input's count against the row's
 * vertex order. 0 rows without the field -- each of a row's inputs then has +1 along the row,
 * the meaning of a row without it -- and always in a build without the option, which does not
 * read it. Throws on a field that does not match the block's `tags`: Int64, its column labels,
 * and 0 wherever the row lacks that tag.
 */
inline DataMatrix<int64_t> read_orientation(const CellData& cells, const std::string& what)
{
    if (!SurfaceTagAttributes::per_input_orientation) return {};
    const auto it = cells.attributes.find("orientation");
    if (it == cells.attributes.end()) return {};
    const auto tags = cells.attributes.find("tags");
    if (tags == cells.attributes.end() || it->second.labels != tags->second.labels)
        throw std::runtime_error(what + ": orientation needs the tags' column labels");
    if (!std::holds_alternative<DataMatrix<int64_t>>(it->second.values))
        throw std::runtime_error(what + ": orientation must be an Int64 matrix");
    DataMatrix<int64_t> counts = std::get<DataMatrix<int64_t>>(it->second.values);
    if (((tags->second.integers().array() == 0) && (counts.array() != 0)).any())
        throw std::runtime_error(what + ": an orientation count where its row lacks the tag");
    return counts;
}
/// Store a tagged row's orientation on its simplex's `attr`: `row` the row's vertices, `set` its
/// tags' bits. Each tag of the row +1 along it -- or, with `counts` (per row and tag column, its
/// count along the row, read_orientation()'s; 0 rows without), each its count r.
template <size_t N>
void orient_tagged_row(
    SurfaceTagAttributes& attr,
    const std::array<size_t, N>& row,
    const InputSet set,
    const MatrixSi& counts,
    const Eigen::Index r)
{
    if (counts.rows() == 0) {
        attr.set_orientation_along(set, row, 1);
        return;
    }
    for (Eigen::Index j = 0; j < counts.cols() && j < SurfaceTagAttributes::max_inputs; ++j)
        if ((set >> j) & 1) attr.set_orientation_along(InputSet(1) << j, row, counts.coeff(r, j));
}
/// A labelled reference (append_envelope_block()) as a receiving mesh reads it
/// (read_labelled_envelope()): its triangles (2D: segments) on vertices of their own (`dim`
/// columns), each with the bits of the receiver's labels it names, 0 for a row of labels the
/// receiver lacks; in 3D its feature curves of the receiver's feature labels, an input edge per
/// edge and label, edge_input that label's feature input.
struct LabelledReference
{
    MatrixXd V;
    MatrixXi F;
    std::vector<InputSet> F_labels;
    utils::FeatureInputs curves;
};
/**
 * @brief Read `envelope`, a labelled reference (is_labelled_envelope()), for a receiving mesh of
 * dimension `dim`: its triangles (2D: segments) and, in 3D, feature curves, each row with its
 * labels, which name the receiver's tags; its points, which no tag of a mesh marks, are only
 * handed on. Matched to the receiver by name: `bits_named(name)` is the receiver's bits a label
 * names (several where its tags share a name), `feature_names` its feature labels, bit i the
 * i-th. Throws on cells a labelled reference does not have, and drops every field but `tags`
 * (ComponentIO::dropped_fields()). simwild's and tetwild's
 * tagged routes read their `envelope` so; moved here from simwild's reader (2026-11), unchanged.
 */
inline LabelledReference read_labelled_envelope(
    const MeshData& envelope,
    const int dim,
    const std::function<InputSet(const std::string&)>& bits_named,
    const std::vector<std::string>& feature_names)
{
    envelope.validate();
    std::set<std::string> dropped = ComponentIO::dropped_fields(envelope.vertex_attributes, {});
    const auto t = dim == 3 ? CellType::Triangle : CellType::Edge;
    for (const auto& [ct, c] : envelope.cells) {
        if (ct != t && ct != CellType::Point && !(dim == 3 && ct == CellType::Edge))
            throw std::runtime_error("Incompatible envelope topology");
        dropped.merge(ComponentIO::dropped_fields(c.attributes, {"tags"}));
    }
    ComponentIO::log_dropped("envelope", dropped);
    const auto labelled = [](const CellData& cells) -> const MeshAttribute& {
        const auto it = cells.attributes.find("tags");
        if (it == cells.attributes.end())
            throw std::runtime_error("A labelled envelope needs tags on each of its blocks");
        const MeshAttribute& field = it->second;
        if (!std::holds_alternative<DataMatrix<uint8_t>>(field.values) ||
            field.labels.size() != size_t(field.cols()))
            throw std::runtime_error("tags must be a UInt8 membership matrix with column labels");
        if ((std::get<DataMatrix<uint8_t>>(field.values).array() > 1).any())
            throw std::runtime_error("tags must be binary");
        return field;
    };
    LabelledReference out;
    // The rows of `cells` on vertices of their own, renumbered in order.
    const auto own_vertices = [&](const DataMatrix<int64_t>& cells, MatrixXd& V, MatrixXi& C) {
        std::vector<int> row_of(envelope.V.rows(), -1);
        for (Eigen::Index i = 0; i < cells.size(); ++i) row_of[cells.data()[i]] = 0;
        int n = 0;
        for (auto& r : row_of)
            if (r == 0) r = n++;
        V.resize(n, dim);
        for (Eigen::Index v = 0; v < envelope.V.rows(); ++v)
            if (row_of[v] >= 0) V.row(row_of[v]) = envelope.V.row(v).head(dim);
        C = ComponentIO::indices(cells);
        for (Eigen::Index i = 0; i < C.size(); ++i) C.data()[i] = row_of[C.data()[i]];
    };
    const CellData& rows = envelope.cell(t);
    const MeshAttribute& row_tags = labelled(rows);
    const auto& member = std::get<DataMatrix<uint8_t>>(row_tags.values);
    own_vertices(rows.connectivity, out.V, out.F);
    out.F_labels.assign(out.F.rows(), 0);
    for (size_t j = 0; j < row_tags.labels.size(); ++j) {
        const InputSet bits = bits_named(row_tags.labels[j]);
        for (Eigen::Index r = 0; r < member.rows(); ++r)
            if (member(r, Eigen::Index(j))) out.F_labels[r] |= bits;
    }
    // The feature curves of the labels the receiver's feature curves have (feature_names), one
    // input edge per edge and label, as a tagged mesh's own are read.
    const auto curve_block = envelope.cells.find(CellType::Edge);
    if (dim == 3 && curve_block != envelope.cells.end()) {
        const MeshAttribute& curve_tags = labelled(curve_block->second);
        const auto& on = std::get<DataMatrix<uint8_t>>(curve_tags.values);
        const auto& E = curve_block->second.connectivity;
        std::map<int64_t, size_t> vertex_of; // envelope row -> curves.edge_vertices
        const auto vertex = [&](const int64_t v) {
            const auto [it, added] = vertex_of.emplace(v, out.curves.edge_vertices.size());
            if (added)
                out.curves.edge_vertices.emplace_back(
                    envelope.V(v, 0),
                    envelope.V(v, 1),
                    envelope.V(v, 2));
            return it->second;
        };
        for (size_t j = 0; j < curve_tags.labels.size(); ++j) {
            const auto k =
                std::find(feature_names.begin(), feature_names.end(), curve_tags.labels[j]);
            if (k == feature_names.end()) continue;
            for (Eigen::Index r = 0; r < E.rows(); ++r) {
                if (!on(r, Eigen::Index(j))) continue;
                out.curves.edges.push_back({{vertex(E(r, 0)), vertex(E(r, 1))}});
                out.curves.edge_input.push_back(int(k - feature_names.begin()));
            }
        }
    }
    return out;
}
/// The rows of a reference (V, F: triangles in 3D, segments in 2D, each with its `labels`, a
/// receiver's bits) that carry a label of the receiver, on vertices of their own, with their
/// labels: what a run given a reference holds to.
struct HeldRows
{
    MatrixXd V;
    MatrixXi F;
    std::vector<InputSet> labels;
};
/**
 * @brief What a run given a reference holds to: its rows of the labels of the mesh. A row of a
 * label the mesh lacks -- an input a filter upstream removed -- stays out of the union, which
 * holds every face with one label, or without per_input_envelopes, so that no face is held to
 * geometry of a label it does not carry. A reference none of whose rows does throws. Moved here
 * from simwild (2026-11), unchanged.
 */
inline HeldRows held_rows(const MatrixXd& V, const MatrixXi& F, const std::vector<InputSet>& labels)
{
    HeldRows out;
    std::vector<Eigen::Index> rows;
    for (Eigen::Index r = 0; r < F.rows(); ++r) {
        if (labels[r] != 0) rows.push_back(r);
    }
    if (rows.empty()) {
        log_and_throw_error(
            "envelope: none of its {} cells carries a label of this mesh",
            F.rows());
    }
    std::vector<int> remap(V.rows(), -1);
    for (const Eigen::Index r : rows)
        for (Eigen::Index k = 0; k < F.cols(); ++k) remap[F(r, k)] = 0;
    int n = 0;
    for (int& v : remap)
        if (v == 0) v = n++;
    out.V.resize(n, V.cols());
    for (Eigen::Index v = 0; v < V.rows(); ++v)
        if (remap[v] >= 0) out.V.row(remap[v]) = V.row(v);
    out.F.resize(rows.size(), F.cols());
    for (size_t i = 0; i < rows.size(); ++i) {
        for (Eigen::Index k = 0; k < F.cols(); ++k) out.F(i, k) = remap[F(rows[i], k)];
        out.labels.push_back(labels[rows[i]]);
    }
    logger().info(
        "envelope: a reference, {} of its {} cells of this mesh's labels held to",
        rows.size(),
        F.rows());
    return out;
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
// name (bit j of the input set is names[j]), each row oriented as the simplex's input was. In a
// build with WMTK_PER_INPUT_ORIENTATION, beside `tags` an `orientation` field with its columns:
// per row and name, that input's count against the row's vertex order, 0 off the input
// (read_orientation()). Nothing is written without names.
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
    std::vector<int64_t> counts; // per row, names.size() of them: with WMTK_PER_INPUT_ORIENTATION
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
#ifdef WMTK_PER_INPUT_ORIENTATION
        for (size_t j = 0; j < names.size(); ++j)
            counts.push_back(((set >> j) & 1) ? attr.input_orientation_along(int(j), vids) : 0);
#endif
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
    if (!SurfaceTagAttributes::per_input_orientation) return;
    DataMatrix<int64_t> orientation(simplices.size(), names.size());
    for (size_t i = 0; i < simplices.size(); ++i)
        for (size_t j = 0; j < names.size(); ++j) orientation(i, j) = counts[i * names.size() + j];
    block.attributes["orientation"] = {std::move(orientation), names};
}
// The tet edges tagged `feature` (TetOptimizerMesh::FeatureEdgeAttributes: the feature curves the
// application was given) whose feature input set is not empty, as the Edge block of
// outputs["mesh"], on the rows collect_input_set_tags() uses: one tags column per name (bit j of
// the set is names[j], the input curve j). A curve carries no orientation, so a row is its
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
// The input points (TetOptimizerMesh / TriOptimizerMesh::m_feature_point_inputs, by their
// anchors) as the Point block of outputs["mesh"], on the rows collect_input_set_tags() uses: one
// tags column per name (bit j: names[j]), the input points' labels. Each at the vertex standing
// for its anchor; one whose anchor a collapse merged into another's -- which the collapse allows
// where each lies in the other's ball -- at the nearest vertex in its ball; one with neither, its
// region filtered away, is not written. Several at one vertex are one row under all their labels.
// Nothing is written without names or point tracking. Until 2026-11 no output marked the input
// points: tetwild's and triwild's `features` listed them beside the mesh.
template <typename Mesh>
void collect_point_tags(const Mesh& mesh, MeshMap& outputs, const std::vector<std::string>& names)
{
    if (names.empty() || !mesh.m_track_feature_points) return;
    const auto& inputs = mesh.m_feature_point_inputs;
    std::vector<int64_t> row(mesh.vert_capacity(), -1);
    std::vector<size_t> carrier(inputs.size(), size_t(-1)); // anchor -> its vertex
    int64_t n = 0;
    for (const auto& v : mesh.get_vertices()) {
        const size_t vid = v.vid(mesh);
        row[vid] = n++;
        const size_t f = mesh.m_feature_point_attribute[vid].m_feature_point_id;
        if (f < inputs.size()) carrier[f] = vid;
    }
    const InputSet mask = names.size() >= size_t(SurfaceTagAttributes::max_inputs)
                              ? ~InputSet(0)
                              : (InputSet(1) << names.size()) - 1;
    std::map<int64_t, InputSet> sets; // output row -> labels
    for (size_t f = 0; f < inputs.size(); ++f) {
        if ((inputs[f] & mask) == 0) continue;
        size_t vid = carrier[f];
        if (vid == size_t(-1)) {
            const double r = mesh.m_feature_point_radius[f];
            double nearest = r * r;
            for (const auto& v : mesh.get_vertices()) {
                const double d =
                    (mesh.m_vertex_attribute[v.vid(mesh)].m_posf - mesh.m_feature_points[f])
                        .squaredNorm();
                if (d <= nearest) {
                    nearest = d;
                    vid = v.vid(mesh);
                }
            }
            if (vid == size_t(-1)) continue;
        }
        sets[row[vid]] |= inputs[f] & mask;
    }
    auto& block = outputs.at("mesh").cells[CellType::Point];
    block.connectivity.resize(Eigen::Index(sets.size()), 1);
    DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Zero(sets.size(), names.size());
    Eigen::Index i = 0;
    for (const auto& [r, set] : sets) {
        block.connectivity(i, 0) = r;
        for (size_t j = 0; j < names.size(); ++j) member(i, Eigen::Index(j)) = (set >> j) & 1;
        ++i;
    }
    block.attributes["tags"] = {std::move(member), names};
}
// A tagged mesh's point block, as collect_point_tags() writes it: its labels, and one point per
// row and label -- as an edge is read once per row and label -- with its row of mesh.V and its
// label's column. None without a point block. A point block needs tags, every row a label, as
// every other lower block of a tagged mesh.
struct PointTags
{
    std::vector<std::string> names;
    std::vector<int64_t> rows;
    std::vector<int> labels;
};
inline PointTags read_point_tags(const MeshData& mesh, const std::string& what)
{
    PointTags out;
    const auto it = mesh.cells.find(CellType::Point);
    if (it == mesh.cells.end() || it->second.connectivity.rows() == 0) return out;
    const auto tags = it->second.attributes.find("tags");
    if (tags == it->second.attributes.end())
        throw std::runtime_error(what + ": its points need tags, one column per label");
    const MeshAttribute& field = tags->second;
    if (!std::holds_alternative<DataMatrix<uint8_t>>(field.values) ||
        field.labels.size() != size_t(field.cols()))
        throw std::runtime_error("tags must be a UInt8 membership matrix with column labels");
    const auto& member = std::get<DataMatrix<uint8_t>>(field.values);
    if ((member.array() > 1).any()) throw std::runtime_error("tags must be binary");
    if ((member.cast<int>().rowwise().sum().array() == 0).any())
        throw std::runtime_error(what + ": every tagged point needs a tag");
    if (field.labels.size() > size_t(SurfaceTagAttributes::max_inputs))
        throw std::runtime_error(what + ": more point labels than an input set holds");
    out.names = field.labels;
    const auto& C = it->second.connectivity;
    for (Eigen::Index r = 0; r < C.rows(); ++r) {
        for (Eigen::Index j = 0; j < member.cols(); ++j) {
            if (!member(r, j)) continue;
            out.rows.push_back(C(r, 0));
            out.labels.push_back(int(j));
        }
    }
    return out;
}
} // namespace wmtk::io
