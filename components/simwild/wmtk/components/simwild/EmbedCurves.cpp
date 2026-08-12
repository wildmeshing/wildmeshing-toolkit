#include "EmbedCurves.hpp"

#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/utils/EmbedSegments.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SimplifySegments.hpp>
#include <wmtk/utils/WindingNumber.hpp>

#include <algorithm>
#include <fstream>
#include <map>
#include <queue>
#include <set>

namespace wmtk::components::simwild {

namespace {

Vector2d apply_transform(const Matrix3d& M, const Vector2d& p)
{
    const Eigen::Vector3d q = M * Eigen::Vector3d(p[0], p[1], 1.0);
    return Vector2d(q[0], q[1]);
}

} // namespace

EmbedCurves::EmbedCurves(
    const std::vector<std::string>& input_paths,
    const std::vector<Matrix3d>& input_transform,
    const double tol_rel,
    const double tol_abs)
{
    assert(input_paths.size() == input_transform.size());

    utils::read_input_curves(input_paths, tol_rel, m_V_curves, m_E_curves, m_Vs, m_Es);

    // Transform each input in place, then rebuild the union from the transformed copies. The
    // union has to agree with the per-input meshes: it is what gets arranged, they are what
    // the winding number is evaluated against.
    bool any_transform = false;
    for (size_t i = 0; i < m_Vs.size(); ++i) {
        if (input_transform[i].isIdentity()) {
            continue;
        }
        any_transform = true;
        for (int j = 0; j < m_Vs[i].rows(); ++j) {
            m_Vs[i].row(j) = apply_transform(input_transform[i], Vector2d(m_Vs[i].row(j)));
        }
    }
    if (any_transform) {
        int nv = 0, ne = 0;
        for (size_t i = 0; i < m_Vs.size(); ++i) {
            nv += int(m_Vs[i].rows());
            ne += int(m_Es[i].rows());
        }
        m_V_curves.resize(nv, 2);
        m_E_curves.resize(ne, 2);
        int voff = 0, eoff = 0;
        for (size_t i = 0; i < m_Vs.size(); ++i) {
            m_V_curves.block(voff, 0, m_Vs[i].rows(), 2) = m_Vs[i];
            m_E_curves.block(eoff, 0, m_Es[i].rows(), 2) = m_Es[i].array() + voff;
            voff += int(m_Vs[i].rows());
            eoff += int(m_Es[i].rows());
        }
    }

    // Which input each row of the concatenated network came from. read_input_curves
    // concatenates in order, and the transform rebuild above preserves that, so the blocks
    // are contiguous. Carried across simplify_curves so the arrangement's provenance can be
    // read back in input terms.
    m_E_input.clear();
    for (size_t i = 0; i < m_Es.size(); ++i) {
        m_E_input.resize(m_E_input.size() + size_t(m_Es[i].rows()), i);
    }
    if (m_E_input.size() != size_t(m_E_curves.rows())) {
        m_E_input.clear(); // the concatenation is not the plain per-input one; do not guess
    }

    if (m_V_curves.rows() == 0 || m_E_curves.rows() == 0) {
        log_and_throw_error(
            "No curves read. A 2D input must be an OBJ with 'l' polyline records; a file with "
            "'f' records is a surface and belongs on the 3D route.");
    }

    logger().info(
        "Read {} curve network(s): #V = {}, #E = {}",
        m_Vs.size(),
        m_V_curves.rows(),
        m_E_curves.rows());
}

std::pair<Vector2d, Vector2d> EmbedCurves::bbox_curves_minmax() const
{
    return {m_V_curves.colwise().minCoeff(), m_V_curves.colwise().maxCoeff()};
}

void EmbedCurves::simplify_curves(
    const double eps,
    const bool use_exact_envelope,
    const int num_threads)
{
    SampleEnvelope envelope(use_exact_envelope);
    envelope.init(m_V_curves, m_E_curves, eps);

    const int nv_before = int(m_V_curves.rows());
    const int ne_before = int(m_E_curves.rows());
    // Link condition on: simplification here may not change the topology of the curve
    // network, because the winding-number tags are evaluated against the untouched per-input
    // copies and merging two curves would make them disagree.
    std::vector<int> surviving_edges;
    const size_t removed = utils::simplify_segments(
        m_V_curves,
        m_E_curves,
        envelope,
        /*use_link_condition=*/true,
        m_E_input.empty() ? nullptr : &surviving_edges);
    if (!m_E_input.empty()) {
        std::vector<size_t> E_input(surviving_edges.size());
        for (size_t k = 0; k < surviving_edges.size(); ++k) {
            E_input[k] = m_E_input[size_t(surviving_edges[k])];
        }
        m_E_input = std::move(E_input);
    }
    logger().info(
        "input simplification: #V {} -> {}, #E {} -> {} ({} vertices removed), envelope {} "
        "(eps {:.4})",
        nv_before,
        m_V_curves.rows(),
        ne_before,
        m_E_curves.rows(),
        removed,
        envelope.use_exact ? "EXACT" : "sampled",
        eps);
}

bool EmbedCurves::embed_curves(const bool tag_from_winding_number)
{
    std::vector<Vector2r> V_rational;
    std::vector<std::vector<int>> E_sources;
    utils::embed_segments(
        m_V_curves,
        m_E_curves,
        m_V_emb,
        V_rational,
        m_F_emb,
        m_E_constrained,
        tag_from_winding_number ? nullptr : &E_sources);

    if (!tag_from_winding_number) {
        if (m_E_input.empty()) {
            log_and_throw_error(
                "Provenance tagging asked for, but the input segments could not be traced "
                "back to their curve networks.");
        }
        // Input segment -> input curve, for every constrained output edge.
        m_E_constrained_inputs.assign(E_sources.size(), {});
        for (size_t e = 0; e < E_sources.size(); ++e) {
            auto& inputs = m_E_constrained_inputs[e];
            for (const int s : E_sources[e]) {
                const size_t in = m_E_input[size_t(s)];
                if (std::find(inputs.begin(), inputs.end(), in) == inputs.end()) {
                    inputs.push_back(in);
                }
            }
            std::sort(inputs.begin(), inputs.end());
        }
    }

    m_V_emb_r.resize(V_rational.size(), 2);
    for (size_t i = 0; i < V_rational.size(); ++i) {
        m_V_emb_r(i, 0) = V_rational[i][0];
        m_V_emb_r(i, 1) = V_rational[i][1];
    }

    // "All rounded" means every arrangement vertex has an exact double representation. A
    // crossing between two segments generally does not: it is an SSI point, and rounding it
    // can make two exactly-distinct vertices collide on the same double. That is precisely
    // the geometry the exact-coordinate path exists for.
    bool all_rounded = true;
    for (int i = 0; i < m_V_emb.rows(); ++i) {
        if (to_rational(Vector2d(m_V_emb.row(i))) != Vector2r(m_V_emb_r.row(i))) {
            all_rounded = false;
            break;
        }
    }

    if (tag_from_winding_number) {
        this->tag_from_winding_number();
    } else {
        tag_from_provenance();
    }

    logger().info(
        "2D embedding: #V = {}, #F = {}, all rounded = {}",
        m_V_emb.rows(),
        m_F_emb.rows(),
        all_rounded);

    return all_rounded;
}

void EmbedCurves::tag_from_provenance()
{
    if (m_E_constrained_inputs.size() != size_t(m_E_constrained.rows())) {
        log_and_throw_error(
            "tag_from_provenance: segment provenance was not collected ({} entries for {} "
            "constrained edges)",
            m_E_constrained_inputs.size(),
            m_E_constrained.rows());
    }

    // The constrained edges, keyed by vertex pair, with the inputs they belong to.
    std::map<std::pair<int, int>, const std::vector<size_t>*> constrained;
    for (size_t i = 0; i < m_E_constrained_inputs.size(); ++i) {
        int a = m_E_constrained(i, 0), b = m_E_constrained(i, 1);
        if (a > b) {
            std::swap(a, b);
        }
        constrained[{a, b}] = &m_E_constrained_inputs[i];
    }

    // Triangle adjacency across the arrangement: for each face, the neighbour opposite each
    // of its three vertices, i.e. across edge (j+1, j+2).
    std::map<std::pair<int, int>, std::array<int, 2>> edge_faces;
    for (int f = 0; f < m_F_emb.rows(); ++f) {
        for (int j = 0; j < 3; ++j) {
            int a = m_F_emb(f, (j + 1) % 3), b = m_F_emb(f, (j + 2) % 3);
            if (a > b) {
                std::swap(a, b);
            }
            auto [it, inserted] =
                edge_faces.try_emplace(std::pair{a, b}, std::array<int, 2>{-1, -1});
            if (inserted) {
                it->second[0] = f;
            } else if (it->second[1] < 0) {
                it->second[1] = f;
            } else {
                log_and_throw_error(
                    "tag_from_provenance: edge ({}, {}) is shared by more than two faces",
                    a,
                    b);
            }
        }
    }

    // Seed: a face touching the lexicographically smallest vertex. The arrangement
    // triangulates the bounding box of the input plus a padding grid, so that corner is
    // outside every input curve and its face carries no tags.
    int seed = 0;
    {
        int seed_v = 0;
        for (int i = 1; i < m_V_emb.rows(); ++i) {
            if (std::make_pair(m_V_emb(i, 0), m_V_emb(i, 1)) <
                std::make_pair(m_V_emb(seed_v, 0), m_V_emb(seed_v, 1))) {
                seed_v = i;
            }
        }
        for (int f = 0; f < m_F_emb.rows(); ++f) {
            if (m_F_emb(f, 0) == seed_v || m_F_emb(f, 1) == seed_v || m_F_emb(f, 2) == seed_v) {
                seed = f;
                break;
            }
        }
    }

    std::vector<std::set<size_t>> tags(m_F_emb.rows());
    std::vector<bool> visited(m_F_emb.rows(), false);
    std::queue<int> q;
    q.push(seed);
    visited[seed] = true;

    size_t n_inconsistent = 0;
    while (!q.empty()) {
        const int f = q.front();
        q.pop();
        for (int j = 0; j < 3; ++j) {
            int a = m_F_emb(f, (j + 1) % 3), b = m_F_emb(f, (j + 2) % 3);
            if (a > b) {
                std::swap(a, b);
            }
            const auto& pair = edge_faces.at({a, b});
            const int nb = pair[0] == f ? pair[1] : pair[0];
            if (nb < 0) {
                continue; // the outer boundary of the arrangement
            }

            std::set<size_t> t = tags[f];
            const auto it = constrained.find({a, b});
            if (it != constrained.end()) {
                for (const size_t in : *it->second) {
                    if (!t.insert(in).second) {
                        t.erase(in);
                    }
                }
            }

            if (!visited[nb]) {
                visited[nb] = true;
                tags[nb] = std::move(t);
                q.push(nb);
            } else if (tags[nb] != t) {
                ++n_inconsistent;
            }
        }
    }

    if (n_inconsistent > 0) {
        log_and_throw_error(
            "tag_from_provenance: {} edge crossings disagree with the tags already assigned. "
            "That means an input curve is not closed, so inside/outside is not decided by the "
            "curve alone -- use the winding-number tagging for this input.",
            n_inconsistent);
    }
    for (size_t i = 0; i < visited.size(); ++i) {
        if (!visited[i]) {
            log_and_throw_error("tag_from_provenance: face {} was not reached", i);
        }
    }

    m_F_tags.resize(m_F_emb.rows(), m_Vs.size());
    size_t n_tagged = 0;
    for (size_t i = 0; i < tags.size(); ++i) {
        for (const size_t t : tags[i]) {
            m_F_tags.coeffRef(int(i), int(t)) = 1;
        }
        if (!tags[i].empty()) {
            ++n_tagged;
        }
    }
    m_F_tags.makeCompressed();
    logger().info(
        "Tagged {} of {} faces from the arrangement's segment provenance.",
        n_tagged,
        tags.size());
}

void EmbedCurves::tag_from_winding_number()
{
    MatrixXd C(m_F_emb.rows(), 2);
    for (int i = 0; i < m_F_emb.rows(); ++i) {
        C.row(i) =
            (m_V_emb.row(m_F_emb(i, 0)) + m_V_emb.row(m_F_emb(i, 1)) + m_V_emb.row(m_F_emb(i, 2))) /
            3.0;
    }

    m_F_tags.resize(m_F_emb.rows(), m_Vs.size());
    for (size_t input_idx = 0; input_idx < m_Vs.size(); ++input_idx) {
        const MatrixXd& V = m_Vs[input_idx];
        MatrixXi E = m_Es[input_idx];

        VectorXd W;
        utils::winding_number_2d(V, E, C, W, m_num_threads);

        if (W.size() > 0 && W.maxCoeff() <= 0.5) {
            // Nothing is inside, which for a closed curve means it is wound the other way.
            // The 3D tag_from_winding_number does not do this and silently produces an
            // all-zero tag column; triwild does, and so does this.
            logger().info("Correcting winding number for input {}", input_idx);
            for (int i = 0; i < E.rows(); ++i) {
                std::swap(E(i, 0), E(i, 1));
            }
            utils::winding_number_2d(V, E, C, W, m_num_threads);
        }
        if (W.size() == 0 || W.maxCoeff() <= 0.5) {
            logger().warn(
                "No winding number above 0.5 for input {}: it tags no face. An open polyline "
                "encloses nothing, so it cannot define a material.",
                input_idx);
        }

        for (int i = 0; i < W.size(); ++i) {
            if (W(i) > 0.5) {
                m_F_tags.coeffRef(i, int(input_idx)) = 1;
            }
        }
    }
    m_F_tags.makeCompressed();
}

void EmbedCurves::consolidate()
{
    std::vector<int> old2new(m_V_emb.rows(), -1);
    int n_new = 0;
    for (int i = 0; i < m_F_emb.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            const int v = m_F_emb(i, j);
            if (old2new[v] < 0) {
                old2new[v] = n_new++;
            }
        }
    }
    if (n_new == m_V_emb.rows()) {
        return; // every vertex is referenced, which is the usual case
    }

    MatrixXd V(n_new, 2);
    MatrixXr Vr(n_new, 2);
    for (int i = 0; i < m_V_emb.rows(); ++i) {
        if (old2new[i] < 0) {
            continue;
        }
        V.row(old2new[i]) = m_V_emb.row(i);
        Vr.row(old2new[i]) = m_V_emb_r.row(i);
    }
    for (int i = 0; i < m_F_emb.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            m_F_emb(i, j) = old2new[m_F_emb(i, j)];
        }
    }
    for (int i = 0; i < m_E_constrained.rows(); ++i) {
        for (int j = 0; j < 2; ++j) {
            m_E_constrained(i, j) = old2new[m_E_constrained(i, j)];
        }
    }
    m_V_emb = V;
    m_V_emb_r = Vr;
    logger().info("consolidate: #V {} -> {}", old2new.size(), n_new);
}

void EmbedCurves::write_curves_obj(const std::string& filename) const
{
    std::ofstream out(filename);
    for (int i = 0; i < m_V_curves.rows(); ++i) {
        out << "v " << m_V_curves(i, 0) << " " << m_V_curves(i, 1) << " 0\n";
    }
    for (int i = 0; i < m_E_curves.rows(); ++i) {
        out << "l " << m_E_curves(i, 0) + 1 << " " << m_E_curves(i, 1) + 1 << "\n";
    }
}

} // namespace wmtk::components::simwild
