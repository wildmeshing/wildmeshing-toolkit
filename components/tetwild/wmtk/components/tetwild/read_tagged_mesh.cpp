#include "read_tagged_mesh.hpp"

#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/predicates.hpp>

#include <algorithm>
#include <map>
#include <set>
#include <variant>

namespace wmtk::components::tetwild {

namespace {

// A `tags` field: a binary UInt8 membership matrix, a label per column.
const DataMatrix<uint8_t>& membership(const MeshAttribute& a)
{
    if (!std::holds_alternative<DataMatrix<uint8_t>>(a.values) ||
        a.labels.size() != size_t(a.cols()))
        throw std::runtime_error("tags must be a UInt8 membership matrix with column labels");
    const auto& m = std::get<DataMatrix<uint8_t>>(a.values);
    if ((m.array() > 1).any()) throw std::runtime_error("tags must be binary");
    return m;
}

} // namespace

bool is_tet_mesh(const MeshData& mesh)
{
    return mesh.cells.count(CellType::Tetrahedron) > 0;
}

TaggedMesh read_tagged_mesh(const MeshData& mesh, const std::string& what)
{
    mesh.validate();
    const CellData& tets = mesh.cell(CellType::Tetrahedron);
    if (tets.connectivity.rows() == 0) log_and_throw_error("{}: a tagged mesh without tets", what);

    // No volumes, first: the reason a mesh with them is refused is the one worth reading. A tag
    // column of no tet, or ambient -- the background, as collect_tags() labels it -- is no volume.
    if (const auto it = tets.attributes.find("tags"); it != tets.attributes.end()) {
        const auto& member = membership(it->second);
        std::vector<std::string> volumes;
        for (Eigen::Index j = 0; j < member.cols(); ++j) {
            const std::string& label = it->second.labels[size_t(j)];
            if (label != "ambient" && (member.col(j).array() != 0).any()) volumes.push_back(label);
        }
        if (!volumes.empty()) {
            log_and_throw_error(
                "{}: tets tagged {}. tetwild holds no volumes, only tagged faces, feature curves "
                "and points; a mesh with volume tags is simwild's to remesh",
                what,
                volumes);
        }
    }
    // Fields: tags are read, every other is dropped.
    for (const auto& entry : mesh.cells) {
        const CellType type = entry.first;
        if (type != CellType::Tetrahedron && type != CellType::Triangle && type != CellType::Edge &&
            type != CellType::Point) {
            log_and_throw_error(
                "{}: a tagged mesh for tetwild has tets and, besides them, triangles (tagged "
                "faces), edges (feature curves) and points only",
                what);
        }
    }

    TaggedMesh out;
    // The vertices of the tets only, in order: the vertices of the other blocks must be theirs.
    std::vector<int64_t> row(mesh.V.rows(), -1);
    const auto& TC = tets.connectivity;
    for (Eigen::Index i = 0; i < TC.size(); ++i) row[TC.data()[i]] = 0;
    for (Eigen::Index v = 0; v < mesh.V.rows(); ++v) {
        if (row[v] < 0) continue;
        row[v] = int64_t(out.V.size());
        out.V.emplace_back(mesh.V(v, 0), mesh.V(v, 1), mesh.V(v, 2));
    }
    out.T.resize(TC.rows());
    for (Eigen::Index t = 0; t < TC.rows(); ++t)
        for (int k = 0; k < 4; ++k) out.T[t][k] = size_t(row[TC(t, k)]);

    // Positively oriented, as tetwild writes its tets and the insertion makes them: the
    // optimization starts from a valid mesh and keeps it valid. Exact, on the doubles.
    size_t n_inverted = 0;
    for (const auto& t : out.T) {
        if (utils::predicates::orient3d(out.V[t[0]], out.V[t[1]], out.V[t[2]], out.V[t[3]]) !=
            utils::predicates::Orientation::NEGATIVE) // Shewchuk's sign of a positive tet
            ++n_inverted;
    }
    if (n_inverted > 0) {
        log_and_throw_error(
            "{}: {} of {} tets are inverted or degenerate",
            what,
            n_inverted,
            out.T.size());
    }

    // A lower block's tags, as simwild's read_mesh_data() reads them: every row with a label,
    // each label one bit of an input set. EnvelopeSurface is simwild's embedded envelope, which
    // tetwild takes as `envelope` instead.
    struct Lower
    {
        const CellData* cells = nullptr;
        DataMatrix<uint8_t> member;
        std::vector<std::string> names;
    };
    const auto lower = [&](const CellType type, const std::string& kind) {
        Lower l;
        const auto found = mesh.cells.find(type);
        if (found == mesh.cells.end() || found->second.connectivity.rows() == 0) return l;
        l.cells = &found->second;
        const auto it = l.cells->attributes.find("tags");
        if (it == l.cells->attributes.end())
            log_and_throw_error("{}: its {}s need tags, one column per label", what, kind);
        l.member = membership(it->second);
        l.names = it->second.labels;
        if ((l.member.cast<int>().rowwise().sum().array() == 0).any())
            log_and_throw_error("{}: every tagged {} needs a tag", what, kind);
        if (std::count(l.names.begin(), l.names.end(), "EnvelopeSurface")) {
            log_and_throw_error(
                "{}: {}s labelled EnvelopeSurface, an embedded envelope: tetwild takes its "
                "envelope as `envelope`",
                what,
                kind);
        }
        if (l.names.size() > size_t(SurfaceTagAttributes::max_inputs)) {
            log_and_throw_error(
                "{}: {} {} labels; at most {} are supported",
                what,
                l.names.size(),
                kind,
                SurfaceTagAttributes::max_inputs);
        }
        return l;
    };
    const auto vertex = [&](const int64_t v, const std::string& kind) {
        if (row[v] < 0) log_and_throw_error("{}: a tagged {} has a vertex no tet has", what, kind);
        return size_t(row[v]);
    };

    if (const Lower faces = lower(CellType::Triangle, "face"); faces.cells) {
        const auto& C = faces.cells->connectivity;
        out.face_names = faces.names;
        for (Eigen::Index r = 0; r < C.rows(); ++r) {
            std::array<size_t, 3> f;
            for (int k = 0; k < 3; ++k) f[k] = vertex(C(r, k), "face");
            out.F.push_back(f);
            InputSet set = 0;
            for (Eigen::Index j = 0; j < faces.member.cols(); ++j)
                if (faces.member(r, j)) set |= InputSet(1) << j;
            out.F_sets.push_back(set);
        }
    }
    if (const Lower edges = lower(CellType::Edge, "edge"); edges.cells) {
        const auto& C = edges.cells->connectivity;
        out.feature_names = edges.names;
        std::map<size_t, size_t> vertex_of; // V row -> features.edge_vertices
        const auto curve_vertex = [&](const size_t v) {
            const auto [it, added] = vertex_of.emplace(v, out.features.edge_vertices.size());
            if (added) out.features.edge_vertices.push_back(out.V[v]);
            return it->second;
        };
        for (Eigen::Index r = 0; r < C.rows(); ++r) {
            const size_t a = vertex(C(r, 0), "edge"), b = vertex(C(r, 1), "edge");
            for (Eigen::Index j = 0; j < edges.member.cols(); ++j) {
                if (!edges.member(r, j)) continue;
                out.features.edges.push_back({{curve_vertex(a), curve_vertex(b)}});
                out.features.edge_input.push_back(int(j));
                out.features_placed.edge_tiling.push_back({std::array<size_t, 2>{{a, b}}});
            }
        }
    }
    if (const auto points = mesh.cells.find(CellType::Point); points != mesh.cells.end()) {
        const auto& C = points->second.connectivity;
        for (Eigen::Index r = 0; r < C.rows(); ++r) {
            const size_t v = vertex(C(r, 0), "point");
            out.features.points.push_back(out.V[v]);
            out.features_placed.point_vertex.push_back(int64_t(v));
        }
    }
    logger().info(
        "tagged mesh {}: {} tets, {} tagged faces ({} labels), {} feature edges ({} labels), {} "
        "points",
        what,
        out.T.size(),
        out.F.size(),
        out.face_names.size(),
        out.features.edges.size(),
        out.feature_names.size(),
        out.features.points.size());
    return out;
}

} // namespace wmtk::components::tetwild
