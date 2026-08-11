#include "EmbedCurves.hpp"

#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/utils/EmbedSegments.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SimplifySegments.hpp>
#include <wmtk/utils/WindingNumber.hpp>

#include <fstream>

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
    const size_t removed =
        utils::simplify_segments(m_V_curves, m_E_curves, envelope, /*use_link_condition=*/true);
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

bool EmbedCurves::embed_curves()
{
    std::vector<Vector2r> V_rational;
    utils::embed_segments(m_V_curves, m_E_curves, m_V_emb, V_rational, m_F_emb, m_E_constrained);

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

    tag_from_winding_number();

    logger().info(
        "2D embedding: #V = {}, #F = {}, all rounded = {}",
        m_V_emb.rows(),
        m_F_emb.rows(),
        all_rounded);

    return all_rounded;
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
