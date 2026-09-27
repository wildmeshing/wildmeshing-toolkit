#include "MeshReduction.hpp"

#include "NumpyCompat.hpp"
#include "PythonFormat.hpp"

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/io.hpp>

#include <algorithm>
#include <cmath>

namespace wmtk::components::simwild::polyfem_helpers {

namespace {

std::vector<int64_t> sorted_copy(const std::vector<int64_t>& v)
{
    std::vector<int64_t> out = v;
    std::sort(out.begin(), out.end());
    return out;
}

} // namespace

ReducedBody classify_reduced_cell(
    const TagNames& tags,
    const std::set<std::string>& ambient_like,
    const std::vector<int64_t>& sorted_vertices)
{
    const bool has_ambient = tags.count("ambient") != 0;
    std::set<std::string> non_ambient;
    for (const auto& t : tags) {
        if (ambient_like.count(t) == 0) non_ambient.insert(t);
    }
    if (has_ambient && !non_ambient.empty()) {
        log_and_throw_error(
            "Element with vertices {} has both 'ambient' and non-ambient tags {} — forbidden in "
            "polyfem mesh reduction",
            python_list(sorted_vertices),
            python_list(non_ambient));
    }
    if (!tags.empty() && non_ambient.empty()) {
        return ReducedBody::ambient;
    }
    if (!non_ambient.empty()) {
        return ReducedBody::body;
    }
    return ReducedBody::skip; // tagless cell
}

ReducedMsh polyfem_reduced_msh(
    const TaggedMesh& mesh,
    const std::vector<std::string>& ambient_like_tags)
{
    ReducedMsh out;
    out.dim = mesh.mesh_dim;
    out.vertices = MatrixXd::Zero(mesh.total_n_nodes, 3);
    out.vertices.leftCols(mesh.mesh_dim) = mesh.coords;

    // Node id -> gmsh tag, for the refusal message, which names the cell by its node tags.
    std::vector<int64_t> tag_of_node(size_t(mesh.total_n_nodes));
    for (const auto& [tag, idx] : mesh.node_tag_to_idx) tag_of_node[size_t(idx)] = tag;

    std::set<std::string> ambient_like(ambient_like_tags.begin(), ambient_like_tags.end());
    ambient_like.insert("ambient");

    std::vector<size_t> ambient_cells;
    std::vector<size_t> body_cells;
    for (size_t c = 0; c < mesh.prim_nodes.size(); ++c) {
        std::vector<int64_t> node_tags;
        for (const int64_t v : mesh.prim_nodes[c]) node_tags.push_back(tag_of_node[size_t(v)]);
        switch (classify_reduced_cell(mesh.prim_tags[c], ambient_like, sorted_copy(node_tags))) {
        case ReducedBody::ambient: ambient_cells.push_back(c); break;
        case ReducedBody::body: body_cells.push_back(c); break;
        case ReducedBody::skip: break;
        }
    }

    out.n_ambient = int64_t(ambient_cells.size());
    out.cells.resize(Eigen::Index(ambient_cells.size() + body_cells.size()), mesh.mesh_dim + 1);
    Eigen::Index row = 0;
    for (const auto* cells : {&ambient_cells, &body_cells}) {
        for (const size_t c : *cells) {
            for (int k = 0; k <= mesh.mesh_dim; ++k) {
                out.cells(row, k) = int(mesh.prim_nodes[c][size_t(k)]);
            }
            ++row;
        }
    }
    return out;
}

void write_polyfem_reduced_msh(const std::string& output_msh, const ReducedMsh& reduced)
{
    const int64_t n_body = int64_t(reduced.cells.rows()) - reduced.n_ambient;
    const auto vertex = [&reduced](const size_t i) {
        return reduced.vertices.row(Eigen::Index(i));
    };
    const auto ambient_cell = [&reduced](const size_t i) {
        return reduced.cells.row(Eigen::Index(i));
    };
    const auto body_cell = [&reduced](const size_t i) {
        return reduced.cells.row(Eigen::Index(reduced.n_ambient) + Eigen::Index(i));
    };

    // Every node on the ambient entity. The body entity gets an empty node block, which MshData
    // takes as "the element vertex ids are global", so both groups index the one node block.
    wmtk::MshData msh;
    if (reduced.dim == 3) {
        msh.add_tet_vertices(size_t(reduced.vertices.rows()), vertex);
        msh.add_tets(size_t(reduced.n_ambient), ambient_cell);
        msh.add_physical_group("ambient");
        msh.add_tet_vertices();
        msh.add_tets(size_t(n_body), body_cell);
        msh.add_physical_group("body");
    } else {
        msh.add_face_vertices(size_t(reduced.vertices.rows()), vertex);
        msh.add_faces(size_t(reduced.n_ambient), ambient_cell);
        msh.add_physical_group("ambient");
        msh.add_face_vertices();
        msh.add_faces(size_t(n_body), body_cell);
        msh.add_physical_group("body");
    }
    msh.save(output_msh, /*binary=*/true);
    logger().info(
        "  reduced  : {}  ({} ambient + {} body {})",
        output_msh,
        reduced.n_ambient,
        n_body,
        reduced.dim == 3 ? "tets" : "triangles");
}

namespace {

/// The running sum `get_mesh_info` makes over rows [first, end) of `reduced.cells`; see the header.
double running_volume(const ReducedMsh& reduced, const Eigen::Index first, const Eigen::Index end)
{
    double vol = 0.0;
    for (Eigen::Index c = first; c < end; ++c) {
        std::array<std::array<double, 3>, 4> v{};
        for (int k = 0; k <= reduced.dim; ++k) {
            for (int d = 0; d < 3; ++d) v[k][d] = reduced.vertices(reduced.cells(c, k), d);
        }
        if (reduced.dim == 3) {
            double edges[3][3];
            for (int e = 0; e < 3; ++e) {
                for (int d = 0; d < 3; ++d) edges[e][d] = v[e + 1][d] - v[0][d];
            }
            vol += std::fabs(numpy_det3(edges)) / 6.0;
        } else {
            const double e1x = v[1][0] - v[0][0];
            const double e1y = v[1][1] - v[0][1];
            const double e2x = v[2][0] - v[0][0];
            const double e2y = v[2][1] - v[0][1];
            // Two rounded products, then a subtraction, then the halving: the Python is
            // `0.5 * abs(e1[0] * e2[1] - e1[1] * e2[0])` on numpy scalars, which never fuses.
            const double p1 = e1x * e2y;
            const double p2 = e1y * e2x;
            vol += 0.5 * std::fabs(p1 - p2);
        }
    }
    return vol;
}

} // namespace

MeshInfo get_mesh_info(const ReducedMsh& reduced)
{
    constexpr int ambient = ReducedMsh::ambient_tag;
    constexpr int body = ReducedMsh::body_tag;
    const Eigen::Index n_ambient = Eigen::Index(reduced.n_ambient);
    const Eigen::Index n_cells = reduced.cells.rows();
    MeshInfo info;
    info.tags = {ambient, body};
    info.dim = reduced.dim;
    info.name_to_tag = {{"ambient", ambient}, {"body", body}};
    info.tag_to_count = {{ambient, n_ambient}, {body, n_cells - n_ambient}};
    info.tag_to_volume = {
        {ambient, running_volume(reduced, 0, n_ambient)},
        {body, running_volume(reduced, n_ambient, n_cells)}};
    return info;
}

} // namespace wmtk::components::simwild::polyfem_helpers
