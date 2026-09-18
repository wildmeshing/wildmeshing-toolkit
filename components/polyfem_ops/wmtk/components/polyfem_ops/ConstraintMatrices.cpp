#include "ConstraintMatrices.hpp"

#include "NumpyCompat.hpp"

#include <wmtk/utils/Logger.hpp>

#include <igl/cotmatrix.h>
#include <igl/massmatrix.h>

#include <Eigen/Sparse>

#include <algorithm>
#include <cctype>
#include <map>
#include <set>
#include <string>

namespace wmtk::components::polyfem_ops {

namespace {

/// global node id -> local index, i.e. the Python `global_to_local` dict.
std::map<int64_t, int64_t> global_to_local(const std::vector<int64_t>& node_ids)
{
    std::map<int64_t, int64_t> out;
    for (size_t i = 0; i < node_ids.size(); ++i) {
        out[node_ids[i]] = static_cast<int64_t>(i);
    }
    return out;
}

/// The local (V, F) pair the Python builds with `V = coords[node_ids]` and F the faces rewritten
/// in local indices, handed to igl unchanged.
void local_patch(
    const MatrixXd& coords,
    const std::vector<int64_t>& node_ids,
    const std::vector<std::array<int64_t, 3>>& interface_faces,
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F)
{
    const std::map<int64_t, int64_t> g2l = global_to_local(node_ids);
    V.resize(static_cast<Eigen::Index>(node_ids.size()), coords.cols());
    for (size_t i = 0; i < node_ids.size(); ++i) {
        V.row(static_cast<Eigen::Index>(i)) = coords.row(node_ids[i]);
    }
    F.resize(static_cast<Eigen::Index>(interface_faces.size()), 3);
    for (size_t i = 0; i < interface_faces.size(); ++i) {
        for (int k = 0; k < 3; ++k) {
            F(static_cast<Eigen::Index>(i), k) =
                static_cast<int>(g2l.at(interface_faces[i][static_cast<size_t>(k)]));
        }
    }
}

/// The length of one interface edge, as `np.linalg.norm(coords[a] - coords[b])`.
double edge_length(const MatrixXd& coords, int64_t a, int64_t b)
{
    std::array<double, 3> d{};
    for (Eigen::Index k = 0; k < coords.cols(); ++k) {
        d[static_cast<size_t>(k)] = coords(a, k) - coords(b, k);
    }
    return numpy_norm(d.data(), static_cast<int64_t>(coords.cols()));
}

/// Diagonal triplets with the given values: rows = cols = 0..n-1, as the Python's
/// `np.arange(n, dtype=np.int32)` pair.
Triplets diagonal(std::vector<double> values)
{
    Triplets out;
    const size_t n = values.size();
    out.rows.resize(n);
    out.cols.resize(n);
    for (size_t i = 0; i < n; ++i) {
        out.rows[i] = static_cast<int32_t>(i);
        out.cols[i] = static_cast<int32_t>(i);
    }
    out.values = std::move(values);
    return out;
}

} // namespace

Triplets get_mass_matrix(
    const MatrixXd& coords,
    const std::vector<std::array<int64_t, 2>>& interface_edges,
    const std::vector<int64_t>& node_ids,
    bool graph,
    const std::vector<std::array<int64_t, 3>>& interface_faces)
{
    const size_t n = node_ids.size();

    if (graph) {
        return diagonal(std::vector<double>(n, 1.0));
    }

    if (!interface_faces.empty()) {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        local_patch(coords, node_ids, interface_faces, V, F);
        Eigen::SparseMatrix<double> M;
        igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_BARYCENTRIC, M);
        const Eigen::VectorXd diag = M.diagonal();
        return diagonal(std::vector<double>(diag.data(), diag.data() + diag.size()));
    }

    const std::map<int64_t, int64_t> g2l = global_to_local(node_ids);
    std::vector<double> vertex_mass(n, 0.0);
    for (const auto& e : interface_edges) {
        const auto ia = g2l.find(e[0]);
        const auto ib = g2l.find(e[1]);
        if (ia == g2l.end() || ib == g2l.end()) continue;
        const double half = edge_length(coords, e[0], e[1]) / 2.0;
        vertex_mass[static_cast<size_t>(ia->second)] += half;
        vertex_mass[static_cast<size_t>(ib->second)] += half;
    }
    return diagonal(std::move(vertex_mass));
}

Triplets get_stiffness_matrix(
    const MatrixXd& coords,
    const std::vector<std::array<int64_t, 2>>& interface_edges,
    const std::vector<int64_t>& node_ids,
    bool graph,
    const std::vector<std::array<int64_t, 3>>& interface_faces)
{
    Triplets out;

    if (!graph && !interface_faces.empty()) {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        local_patch(coords, node_ids, interface_faces, V, F);
        // igl.cotmatrix: off-diagonal >= 0, diagonal <= 0 (negative semi-definite Laplacian).
        // Negate to match our convention: off-diagonal <= 0, diagonal >= 0.
        Eigen::SparseMatrix<double> L;
        igl::cotmatrix(V, F, L);
        // Column-major traversal with ascending row inside a column: that is Eigen's storage
        // order, and it is exactly what the Python gets from `.tocoo()` on the CSC matrix the
        // igl binding hands back. Explicit zeros are kept on both sides (neither igl's
        // setFromTriplets nor scipy's converter prunes them), so the entry counts agree too.
        for (Eigen::Index c = 0; c < L.outerSize(); ++c) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(L, c); it; ++it) {
                out.rows.push_back(static_cast<int32_t>(it.row()));
                out.cols.push_back(static_cast<int32_t>(c));
                out.values.push_back(-it.value());
            }
        }
        return out;
    }

    const std::map<int64_t, int64_t> g2l = global_to_local(node_ids);
    // The Python accumulates into a defaultdict and then writes `stiffness.keys()` /
    // `stiffness.values()`, so the entry order is the order in which each (row, col) pair was
    // FIRST touched. The index map plus the parallel key/value vectors reproduce that.
    std::map<std::array<int64_t, 2>, size_t> index_of;
    std::vector<std::array<int64_t, 2>> keys;
    std::vector<double> vals;
    const auto bump = [&](int64_t r, int64_t c, double delta) {
        const std::array<int64_t, 2> key{r, c};
        auto [it, inserted] = index_of.emplace(key, keys.size());
        if (inserted) {
            keys.push_back(key);
            vals.push_back(0.0);
        }
        vals[it->second] += delta;
    };

    for (const auto& e : interface_edges) {
        const auto ia = g2l.find(e[0]);
        const auto ib = g2l.find(e[1]);
        if (ia == g2l.end() || ib == g2l.end()) continue;
        const int64_t la = ia->second;
        const int64_t lb = ib->second;
        const double weight = graph ? 1.0 : 1.0 / edge_length(coords, e[0], e[1]);
        bump(la, lb, -weight);
        bump(lb, la, -weight);
        bump(la, la, weight);
        bump(lb, lb, weight);
    }

    out.rows.reserve(keys.size());
    out.cols.reserve(keys.size());
    for (const auto& k : keys) {
        out.rows.push_back(static_cast<int32_t>(k[0]));
        out.cols.push_back(static_cast<int32_t>(k[1]));
    }
    out.values = std::move(vals);
    return out;
}

Triplets get_laplacian_matrix(
    const MatrixXd& coords,
    const std::vector<std::array<int64_t, 2>>& interface_edges,
    const std::vector<int64_t>& node_ids,
    bool graph,
    const std::vector<std::array<int64_t, 3>>& interface_faces)
{
    Triplets s = get_stiffness_matrix(coords, interface_edges, node_ids, graph, interface_faces);
    const Triplets m = get_mass_matrix(coords, interface_edges, node_ids, graph, interface_faces);

    // M is diagonal, so M_values[i] is the i-th local diagonal entry. The Python inverts the
    // whole diagonal first (`M_inv = 1.0 / M_values`) and then gathers, which is what makes a
    // massless node an inf rather than an error; keep that.
    std::vector<double> m_inv(m.values.size());
    for (size_t i = 0; i < m.values.size(); ++i) {
        m_inv[i] = 1.0 / m.values[i];
    }
    for (size_t i = 0; i < s.values.size(); ++i) {
        s.values[i] = m_inv[static_cast<size_t>(s.rows[i])] * s.values[i];
    }
    return s;
}

std::optional<std::vector<int>> parse_axes(const nlohmann::json& spec, int dim)
{
    if (spec.is_null()) {
        return std::nullopt;
    }
    std::set<int> picked;
    if (spec.is_string()) {
        // `"xyz"[:dim]`: an axis letter the mesh does not have is an error, and because the
        // allowed letters are a prefix of "xyz" the position in them IS the component index.
        const std::string allowed = std::string("xyz").substr(0, static_cast<size_t>(dim));
        for (char ch : spec.get<std::string>()) {
            const char lower = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
            const size_t pos = allowed.find(lower);
            if (pos == std::string::npos) {
                log_and_throw_error("axis '{}' invalid for a {}D mesh", lower, dim);
            }
            picked.insert(static_cast<int>(pos));
        }
    } else {
        for (const auto& a : spec) {
            picked.insert(a.get<int>());
        }
    }
    return std::vector<int>(picked.begin(), picked.end());
}

} // namespace wmtk::components::polyfem_ops
