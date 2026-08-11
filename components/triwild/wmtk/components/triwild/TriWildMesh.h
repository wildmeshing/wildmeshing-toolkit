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

    /// Iterations mesh_improvement actually used. Reported so a run that needs the whole
    /// budget is visible as such, and asserted against in the integration tests.
    int m_iterations_used = 0;
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
    bool smoothing_position_is_allowed(const size_t vid, const Vector2d& p) const;

    /**
     * @brief {feature points still represented within eps, total feature points}.
     *
     * The invariant preserve_feature_points maintains, measured on the finished mesh rather
     * than assumed from the per-operation checks.
     */
    std::pair<size_t, size_t> feature_retention(double* worst_ratio = nullptr) const;

    /// True iff collapsing v1 into v2 would drop or displace a feature point.
    bool collapse_breaks_feature(const size_t v1_id, const size_t v2_id) const;


    /// Position hooks for the shared 2D smoothing driver. triwild keeps both a working
    /// double position and an exact rational one, so writing goes through here.
    Vector2d smoothing_position(const size_t vid) const;
    void set_smoothing_position(const size_t vid, const Vector2d& p);
    std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(const size_t vid) const;
    std::shared_ptr<SampleEnvelope> smoothing_containment_envelope(const size_t vid) const;

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
     */
    void init_mesh(
        const MatrixXd& V,
        const std::vector<Vector2r>& V_rational,
        const MatrixXi& F,
        const MatrixXi& E,
        const std::vector<std::string>& tag_names,
        const MatrixXd& V_env,
        const MatrixXi& E_env);

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
    size_t refine_sizing_around_worst(double max_energy);

    /// The longest edge of each current worst triangle. split_all_edges force-splits
    /// exactly these edges (bypassing the length gate), so a stuck sliver's long edge is
    /// split immediately without changing the sizing field. Populated serially by
    /// refine_sizing_around_worst; read-only during the parallel split pass, then cleared
    /// once split_all_edges has consumed it.
    std::set<simplex::Edge> m_force_split_edges;

    /// Count of force-splits taken in the current split pass (atomic_ref from the parallel
    /// split; reset + logged by split_all_edges). Diagnostic only.
    size_t m_force_split_count = 0;

    /// Per-pass claim for the high-valence split gate: one slot per vertex, reset at the
    /// start of every split pass. A high-valence vertex accepts the first valence-increasing
    /// split of the pass and refuses the rest, so refinement spreads instead of piling onto
    /// the same vertex. Atomic because splits run in parallel; a plain array of unique_ptr
    /// rather than a vector because std::atomic is not movable.
    std::unique_ptr<std::atomic<int>[]> m_high_valence_claim;
    size_t m_high_valence_claim_size = 0;
    /// Splits refused by that gate in the current pass, reported once per pass.
    std::atomic<size_t> m_high_valence_rejects = 0;

    /// True iff edge (v1,v2) is a worst triangle's longest edge queued for force-split.
    bool is_force_split_edge(size_t v1, size_t v2) const
    {
        return m_force_split_edges.find(simplex::Edge(v1, v2)) != m_force_split_edges.end();
    }

    void write_msh_groups(std::string file, const bool write_envelope = true);

    void write_vtu(const std::string& path) const;

public:
    void split_all_edges();
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& loc) override;

    void collapse_all_edges(bool is_limit_length = true);
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    void smooth_all_vertices(const size_t n_iters = 1);
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    void mesh_improvement(int max_its = 80);

    std::tuple<double, double> local_operations(
        const std::array<int, 4>& ops,
        bool collapse_limit_length = true);

    /**
     * @brief m_quality threshold above which a face is "active" (worth operating on) for
     * the skip-good-regions filter.
     *
     * Unlike the tet applications, m_quality here *is* the AMIPS2D energy (tetwild stores
     * AMIPS^3 and cube-roots it), so the threshold is the energy directly.
     */
    double active_quality_threshold() const
    {
        return m_params.skip_good_regions_margin * m_params.stop_energy;
    }

    /**
     * @brief vids of the vertices incident to at least one "active" face
     * (m_quality >= active_quality_threshold()). Used by the skip-good-regions filter to
     * restrict smoothing to non-good regions (smoothing a vertex surrounded by good faces
     * does nothing).
     */
    std::vector<size_t> active_vertices() const;

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

private:
    ////// Operations

    struct SplitInfoCache
    {
        //        VertexAttributes vertex_info;
        size_t v1_id;
        size_t v2_id;
        /// Worst quality among the elements incident to the edge BEFORE the split, so
        /// split_edge_after can tell "this split created a degenerate element" from "this
        /// split subdivided a region that was already degenerate".
        double max_quality_before = 0.;

        EdgeAttributes old_e_attrs;

        // std::vector<std::pair<EdgeAttributes, std::array<size_t, 2>>> changed_edges;
        std::map<simplex::Edge, EdgeAttributes> changed_edges;

        /**
         * All faces incident to the splitted edge, identified by the link vertex (the vertex
         * opposite to the splitted edge).
         */
        std::map<size_t, FaceAttributes> faces;
    };
    wmtk::threading::enumerable_thread_specific<SplitInfoCache> split_cache;

    struct CollapseInfoCache
    {
        size_t v1_id;
        size_t v2_id;
        double max_energy;
        double edge_length;

        std::vector<std::pair<EdgeAttributes, std::array<size_t, 2>>> changed_edges;
        // all faces incident to the delete vertex (v1) that are on the tracked surface
        std::vector<std::array<size_t, 2>> surface_edges;
        std::vector<size_t> changed_fids;
        std::vector<double> changed_energies;
    };
    wmtk::threading::enumerable_thread_specific<CollapseInfoCache> collapse_cache;

};


} // namespace wmtk::components::triwild
