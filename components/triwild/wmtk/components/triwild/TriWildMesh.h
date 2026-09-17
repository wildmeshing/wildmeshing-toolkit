#pragma once

#include <wmtk/TriOptimizerMesh.h>

#include <wmtk/SurfaceTagAttributes.h>
#include <cstdlib>

#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include "Parameters.h"

#include <atomic>
#include <memory>
#include <set>

namespace wmtk::components::triwild {

/// The attribute types now live on the shared base; keep the unqualified names working.
using VertexAttributes = wmtk::TriOptimizerMesh::VertexAttributes;
using EdgeAttributes = wmtk::TriOptimizerMesh::EdgeAttributes;
using FaceAttributes = wmtk::TriOptimizerMesh::FaceAttributes;

/// A vertex that stands for no input feature point. See VertexExtras::m_feature_id.
inline constexpr size_t NO_FEATURE = std::numeric_limits<size_t>::max();

class TriWildMesh : public wmtk::TriOptimizerMesh
{
public:
    /**
     * @brief triwild's per-vertex additions to the shared VertexAttributes.
     *
     * Registered with the base's m_vertex_attr_group, so it is resized, protected and rolled
     * back exactly like the shared collection -- which matters, because collapse_edge_after and
     * split_edge_after both write it.
     */
    struct VertexExtras
    {
        /**
         * Index into TriWildMesh::m_feature_points, or NO_FEATURE.
         *
         * A feature point is a 0-dimensional feature of the curve network: an open polyline's
         * endpoint, or a junction. This is the 2D counterpart of tetwild's
         * m_is_on_open_boundary, with one deliberate difference -- it names WHICH feature the
         * vertex stands for, not merely that it stands for one.
         *
         * That difference is the whole point. tetwild asks "is the survivor inside the
         * envelope of the boundary?", a containment question, and in 3D a boundary is a curve
         * with many edges so collapsing one shortens it rather than deleting it. In 2D the
         * feature is a single point, and a containment test passes trivially when one endpoint
         * is collapsed onto another -- the target IS a feature point, distance zero -- while
         * the first endpoint quietly stops being represented. Preserving features is a
         * COVERAGE property, so the constraint has to bind a vertex to a specific point.
         */
        size_t m_feature_id = std::numeric_limits<size_t>::max();
    };
    wmtk::AttributeCollection<VertexExtras> m_vertex_extra;

    /// The base holds only wmtk::OptimizerParameters; this is the same object, typed, for the
    /// triwild-only fields (features, high valence, smoothing schedule, box).
    Parameters& m_tri_params;

    std::vector<Vector2d> m_V_envelope;
    std::vector<Vector2i> m_E_envelope;
    /**
     * Radius of the ball a feature-carrying vertex must stay inside.
     *
     * Presently equal to m_envelope_eps, but deliberately a separate name: the two are
     * different quantities -- the envelope bounds how far the CURVE may deviate, this bounds
     * how far a SURVIVOR VERTEX may sit from an anchor -- so tying them means a tighter
     * envelope silently tightens feature retention by the same factor. Measured on
     * triwild20k 189017 at eps_rel 1e-4, giving this its own (10x wider) value cut collapse
     * refusals 75270 -> 61666 and delayed the mesh blow-up by two iterations. That was not
     * enough to fix that model on its own -- the collapse length gate was the real cause --
     * so the value is left alone here and only the coupling is made visible.
     */
    double m_feature_eps = -1;

    /**
     * Anchor positions of the curve network's 0-dimensional features, indexed by
     * VertexExtras::m_feature_id.
     *
     * Filled in init_mesh from the arrangement's constrained edges: a vertex whose valence in
     * that edge set is neither 0 nor 2 is a feature -- valence 1 is an open polyline's
     * endpoint, valence >= 3 a junction. Reading it off the arrangement rather than the raw
     * input is deliberate: it is the network the optimizer actually starts from, so a
     * junction the simplification was allowed to merge (which it may, with
     * simplify_use_link_condition off) is correctly absent, and no provenance or
     * position-matching plumbing is needed.
     *
     * A vertex carrying feature f may never end up further than m_envelope_eps from
     * m_feature_points[f]. Since the anchor never moves, no spatial index is required: the
     * feature id is a direct lookup.
     */
    std::vector<Vector2d> m_feature_points;
    /// Collapses refused because they would drop or displace a feature point. Diagnostic.
    std::atomic<size_t> m_feature_rejects = 0;

    /**
     * @brief May vertex `vid` sit at `p`?
     *
     * False only when the vertex stands for a feature point and `p` is more than
     * m_envelope_eps away from it. Smoothing is free to move a feature vertex inside that
     * ball -- it is a containment test, not a freeze, so the optimizer keeps the quality it
     * can get near features.
     */
    bool smoothing_position_is_allowed(const size_t vid, const Vector2d& p) const override;
    void split_after_vertex(const size_t vid) override
    {
        m_vertex_extra[vid].m_feature_id = NO_FEATURE;
    }

    /**
     * @brief {feature points still represented within eps, total feature points}.
     *
     * The invariant preserve_feature_points maintains, measured on the finished mesh rather
     * than assumed from the per-operation checks.
     */
    std::pair<size_t, size_t> feature_retention(double* worst_ratio = nullptr) const;

    /// True iff collapsing v1 into v2 would drop or displace a feature point.
    bool collapse_breaks_feature(const size_t v1_id, const size_t v2_id) const;


    TriWildMesh(Parameters& _m_params, double envelope_eps, int _num_threads = 0)
        : wmtk::TriOptimizerMesh(_m_params)
        , m_tri_params(_m_params)
        , m_feature_eps(envelope_eps)
    {
        m_vertex_attr_group.add(&m_vertex_extra);
        m_envelope_eps = envelope_eps;
        NUM_THREADS = _num_threads;

        optimization::deactivate_opt_logger();

        double& wa = m_params.w_amips;
        double& we = m_params.w_envelope;
        we = 1 - wa;
        logger().info("w_envelope = {}", we);
    }

    ~TriWildMesh() {}

public:
    /**
     * @brief Init mesh from IGL-style matrices.
     *
     * @param V #Vx2 vertices of the initial mesh
     * @param F #Fx3 vertex IDs for all faces
     * @param E #Ex2 vertex IDs for all constraint edges
     * @param tag_names Names for each tag.
     * @param V_rational the arrangement's EXACT vertex positions, matching V row for row.
     *        V is only their rounding, and two exactly distinct vertices can round to the
     *        same double, so V alone cannot tell a genuine degeneracy from a rounding
     *        collision. Pass an empty vector when no exact positions exist (unit tests
     *        building a mesh from doubles); every vertex is then taken as rounded.
     * @param V_env,E_env the curves the envelope is built around. These are the *original*
     *        input curves, not the arrangement's constrained edges: the optimizer has to
     *        stay near what the user gave us, not near the simplified version of it. Same
     *        arrangement as tetwild, which hands its optimizer the envelope built on the
     *        unsimplified input surface.
     * @param free_point_vids rows of V that stand for input FREE POINTS -- input vertices
     *        with no incident segment. Each is anchored in m_feature_points exactly like a
     *        polyline endpoint: same ball, same collapse policy, same retention audit. They
     *        cannot be derived here the way endpoints and junctions are, because their
     *        valence in E is 0 -- the same valence as every background-grid vertex.
     */
    void init_mesh(
        const MatrixXd& V,
        const std::vector<Vector2r>& V_rational,
        const MatrixXi& F,
        const MatrixXi& E,
        const std::vector<std::string>& tag_names,
        const MatrixXd& V_env,
        const MatrixXi& E_env,
        const std::vector<size_t>& free_point_vids = {});

    void init_surfaces_and_boundaries();

    void init_envelope(const MatrixXd& V, const MatrixXi& F);

    /**
     * @brief The old global sizing-field update (KNN R-ball around every low-quality
     * triangle). Superseded by refine_sizing_around_worst, but kept compiled and callable
     * so the two can be compared -- tetwild and simwild keep theirs for the same reason.
     */

    /**
     * @brief Escape a stuck max energy by refining the sizing field around the worst
     * elements.
     *
     * Finds the m_params.stuck_refine_num_worst triangles with the highest energy (0 =>
     * all of them above the filter energy), gathers all vertices within
     * m_params.stuck_refine_rings graph rings of them, and multiplies each such vertex's
     * m_sizing_scalar by m_params.stuck_refine_factor (clamped at
     * m_params.stuck_refine_min_scalar). Then runs gradation_smooth_sizing so the refined
     * region blends smoothly into the surrounding resolution.
     *
     * @return the number of vertices refined.
     */
    size_t refine_sizing_around_worst(double max_energy) override;

    void write_msh_groups(std::string file, const bool write_envelope = true);

    void write_vtu(const std::string& path) const;

public:
    /**
     * @brief Tag every face with the inputs it lies inside, by winding number.
     *
     * `Vs`/`Es` are the per-input meshes as read when the initial mesh was built (2D, x/y
     * only) -- they are passed in rather than re-read from disk.
     */
    void compute_winding_numbers(const std::vector<MatrixXd>& Vs, const std::vector<MatrixXi>& Es);

    /// Remove the faces that lie inside no input (needs compute_winding_numbers).
    void filter_with_input_winding_number();
    /// Remove the flood-fill region that dominates the mesh boundary (needs flood_fill).
    void filter_with_flood_fill();

    int flood_fill();

protected:
    void write_smoothing_debug_output(const std::string& path) const override { write_vtu(path); }

    void collapse_pass_begin() override { m_feature_rejects = 0; }
    void collapse_pass_end(size_t) override
    {
        if (const size_t n = m_feature_rejects.load(); n > 0) {
            logger().info(
                "[feature] {} collapses refused to keep a feature point (polyline endpoint, "
                "junction, or input free point) within {:.6} of its input position",
                n,
                m_envelope_eps);
        }
    }
    bool collapse_before_vertex(size_t v1, size_t v2) override
    {
        if (!collapse_breaks_feature(v1, v2)) return true;
        m_feature_rejects.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    void collapse_after_vertex(size_t v1, size_t v2) override
    {
        if (m_vertex_extra[v2].m_feature_id == NO_FEATURE) {
            m_vertex_extra[v2].m_feature_id = m_vertex_extra[v1].m_feature_id;
        }
    }
};


} // namespace wmtk::components::triwild
