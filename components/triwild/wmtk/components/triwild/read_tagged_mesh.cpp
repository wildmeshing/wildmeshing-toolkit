#include "read_tagged_mesh.hpp"

#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/io/tagged_mesh_data.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/predicates.hpp>

#include <algorithm>
#include <array>
#include <map>
#include <set>
#include <variant>

namespace wmtk::components::triwild {

namespace {

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

bool is_triangle_mesh(const MeshData& mesh)
{
    return mesh.cells.count(CellType::Triangle) > 0 && !mesh.cells.count(CellType::Tetrahedron);
}

TaggedTriMesh read_tagged_mesh(const MeshData& mesh, const std::string& what)
{
    mesh.validate();
    const CellData& tris = mesh.cell(CellType::Triangle);
    if (tris.connectivity.rows() == 0) {
        log_and_throw_error("{}: a tagged mesh without triangles", what);
    }

    // No regions, first, as tetwild refuses volumes first: the reason worth reading.
    if (const auto it = tris.attributes.find("tags"); it != tris.attributes.end()) {
        const auto& member = membership(it->second);
        std::vector<std::string> regions;
        for (Eigen::Index j = 0; j < member.cols(); ++j) {
            const std::string& label = it->second.labels[size_t(j)];
            if (label != "ambient" && (member.col(j).array() != 0).any()) regions.push_back(label);
        }
        if (!regions.empty()) {
            log_and_throw_error(
                "{}: triangles tagged {}. triwild holds no regions, only tracked curves and "
                "points; a mesh with region tags is simwild's to remesh",
                what,
                regions);
        }
    }
    // Fields: tags are read, every other is dropped (io::ComponentIO::data_fields).
    std::set<std::string> dropped = io::ComponentIO::dropped_fields(mesh.vertex_attributes, {});
    for (const auto& [type, cells] : mesh.cells) {
        if (type == CellType::Triangle || type == CellType::Edge || type == CellType::Point) {
            dropped.merge(io::ComponentIO::dropped_fields(cells.attributes, {"tags"}));
        } else {
            log_and_throw_error(
                "{}: a tagged mesh for triwild has triangles and, besides them, edges (tracked "
                "curves) and points only",
                what);
        }
    }
    io::ComponentIO::log_dropped(what, dropped);

    TaggedTriMesh out;
    std::vector<int64_t> row(mesh.V.rows(), -1);
    const auto& TC = tris.connectivity;
    for (Eigen::Index i = 0; i < TC.size(); ++i) row[TC.data()[i]] = 0;
    int64_t n = 0;
    for (int64_t& r : row)
        if (r == 0) r = n++;
    out.V.resize(n, 2);
    for (Eigen::Index v = 0; v < mesh.V.rows(); ++v) {
        if (row[v] < 0) continue;
        if (mesh.V(v, 2) != 0) {
            log_and_throw_error("{}: a tagged mesh for triwild lies in the plane z = 0", what);
        }
        out.V.row(row[v]) << mesh.V(v, 0), mesh.V(v, 1);
    }
    out.F.resize(TC.rows(), 3);
    for (Eigen::Index t = 0; t < TC.rows(); ++t)
        for (int k = 0; k < 3; ++k) out.F(t, k) = int(row[TC(t, k)]);

    // Counter-clockwise, as triwild makes its triangles: exact, on the doubles.
    size_t n_inverted = 0;
    for (Eigen::Index t = 0; t < out.F.rows(); ++t) {
        const Vector2d a = out.V.row(out.F(t, 0)).transpose();
        const Vector2d b = out.V.row(out.F(t, 1)).transpose();
        const Vector2d c = out.V.row(out.F(t, 2)).transpose();
        if (utils::predicates::orient2d(a, b, c) != utils::predicates::Orientation::POSITIVE)
            ++n_inverted;
    }
    if (n_inverted > 0) {
        log_and_throw_error(
            "{}: {} of {} triangles are clockwise or degenerate",
            what,
            n_inverted,
            out.F.rows());
    }

    const auto vertex = [&](const int64_t v, const std::string& kind) {
        if (row[v] < 0) {
            log_and_throw_error("{}: a tagged {} has a vertex no triangle has", what, kind);
        }
        return size_t(row[v]);
    };
    if (const auto found = mesh.cells.find(CellType::Edge);
        found != mesh.cells.end() && found->second.connectivity.rows() > 0) {
        const auto& cells = found->second;
        const auto it = cells.attributes.find("tags");
        if (it == cells.attributes.end()) {
            log_and_throw_error("{}: its edges need tags, one column per label", what);
        }
        const auto& member = membership(it->second);
        out.edge_names = it->second.labels;
        if ((member.cast<int>().rowwise().sum().array() == 0).any())
            log_and_throw_error("{}: every tagged edge needs a tag", what);
        if (std::count(out.edge_names.begin(), out.edge_names.end(), "EnvelopeSurface")) {
            log_and_throw_error(
                "{}: edges labelled EnvelopeSurface, an embedded envelope: triwild takes its "
                "envelope as `envelope`",
                what);
        }
        if (out.edge_names.size() > size_t(SurfaceTagAttributes::max_inputs)) {
            log_and_throw_error(
                "{}: {} edge labels; at most {} are supported",
                what,
                out.edge_names.size(),
                SurfaceTagAttributes::max_inputs);
        }
        // One row per edge, by its vertex set: the labels united, the directions counted.
        struct Merged
        {
            size_t row;
            InputSet set = 0;
            bool along = false, against = false;
        };
        std::map<std::array<size_t, 2>, Merged> edges;
        std::vector<std::array<size_t, 2>> order;
        const auto& C = cells.connectivity;
        for (Eigen::Index r = 0; r < C.rows(); ++r) {
            const std::array<size_t, 2> e = {{vertex(C(r, 0), "edge"), vertex(C(r, 1), "edge")}};
            const std::array<size_t, 2> key = {{std::min(e[0], e[1]), std::max(e[0], e[1])}};
            auto [m, added] = edges.emplace(key, Merged{order.size()});
            if (added) order.push_back(e);
            for (Eigen::Index j = 0; j < member.cols(); ++j)
                if (member(r, j)) m->second.set |= InputSet(1) << j;
            (e == order[m->second.row] ? m->second.along : m->second.against) = true;
        }
        out.E.resize(order.size(), 2);
        out.E_sets.resize(order.size());
        out.E_orientation.resize(order.size());
        for (size_t i = 0; i < order.size(); ++i) {
            const auto& m = edges.at(
                {{std::min(order[i][0], order[i][1]), std::max(order[i][0], order[i][1])}});
            out.E(i, 0) = int(order[i][0]);
            out.E(i, 1) = int(order[i][1]);
            out.E_sets[i] = m.set;
            out.E_orientation[i] = int(m.along) - int(m.against);
        }
    }
    // The point block: one input point per row and label (io::read_point_tags()), as tetwild's.
    const io::PointTags points = io::read_point_tags(mesh, what);
    out.point_names = points.names;
    for (size_t k = 0; k < points.rows.size(); ++k) {
        out.point_vids.push_back(vertex(points.rows[k], "point"));
        out.point_labels.push_back(points.labels[k]);
    }
    logger().info(
        "tagged mesh {}: {} triangles, {} tracked edges ({} labels), {} points",
        what,
        out.F.rows(),
        out.E.rows(),
        out.edge_names.size(),
        out.point_vids.size());
    return out;
}

} // namespace wmtk::components::triwild
