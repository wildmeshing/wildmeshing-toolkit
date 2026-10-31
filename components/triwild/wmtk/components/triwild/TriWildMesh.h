#pragma once

#include <wmtk/MeshData.hpp>

#include <wmtk/TriOptimizerMesh.h>

#include <wmtk/SurfaceTagAttributes.h>
#include <cstdlib>

#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/CompositeEnvelopes.hpp>
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

#include <memory>
#include <set>

namespace wmtk::components::triwild {

/// The attribute types now live on the shared base; keep the unqualified names working.
using VertexAttributes = wmtk::TriOptimizerMesh::VertexAttributes;
using EdgeAttributes = wmtk::TriOptimizerMesh::EdgeAttributes;
using FaceAttributes = wmtk::TriOptimizerMesh::FaceAttributes;

class TriWildMesh : public wmtk::TriOptimizerMesh
{
public:
    /// The base holds only wmtk::OptimizerParameters; this is the same object, typed, for the
    /// triwild-only fields (features, high valence, smoothing schedule, box).
    Parameters& m_tri_params;

    std::vector<Vector2d> m_V_envelope;
    std::vector<Vector2i> m_E_envelope;

    // One envelope per input curve file, when there are several: the base's m_input_envelopes,
    // built by init_mesh around the same curves as m_envelope, at the same eps and predicate,
    // one input per file. Empty with one input, and then every segment is held to m_envelope
    // as it always was. The containment and the pull are the base's defaults.

    // The curve network's 0-dimensional features are anchored by the base
    // (TriOptimizerMesh::anchor_feature_points()), called by init_mesh once the tracked
    // segments are set: the open polylines' endpoints this class holds (holds_order2_vertex())
    // and the input free points, each in a ball of its kind's ratio x m_envelope_eps. Reading
    // the endpoints off the arrangement rather than the raw input is deliberate: it is the
    // network the optimizer actually starts from, and no provenance or position-matching
    // plumbing is needed.

    /**
     * The polyline endpoints only (1 segment), not the junctions (3 and more).
     *
     * The erosion the anchors exist for (see init_mesh) is a property of ENDPOINTS: a polyline
     * eats its own tip. A junction cannot erode a curve away -- every curve through it stays a
     * constrained edge and the envelope still holds it within eps -- while anchoring it costs a
     * great deal, because on a self-intersecting input nearly every arrangement vertex is a
     * crossing: on 2D model 242427, 79615 of 87610 vertices are junctions, so almost every edge
     * joins two of them and collapse is refused there. In August 2026 that model never left
     * MAX_ENERGY with junctions anchored (1e50 for the whole run, the split pass growing it from
     * 13k to 621k vertices) and converged to 10.48 in 12 iterations with them free; in October
     * it converged either way but kept 2.6 times the vertices anchored (14257 against 5476,
     * stop_energy 10, 8 threads). The two models the endpoint guard was introduced for (215292
     * and 134005) were bit-identical either way. They are the 2D twin of tetwild's non-manifold
     * edges, which tetwild does not hold either (TetWildMesh::holds_order2_edge()).
     */
    bool holds_order2_vertex(const size_t n_segments) const override { return n_segments == 1; }

    TriWildMesh(Parameters& _m_params, double envelope_eps, int _num_threads = 0)
        : wmtk::TriOptimizerMesh(_m_params)
        , m_tri_params(_m_params)
    {
        // Registered unconditionally, as the ids always were: the driver exports and audits
        // the anchors.
        enable_feature_point_tracking();
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
     * @param E_orientation optional: per row of E, the input's direction on it, measured along
     *        the row as written (embed_segments' E_out_orientation). When given, the tracked
     *        edges carry it and m_tracks_orientation is set.
     * @param free_point_vids rows of V that stand for input FREE POINTS -- input vertices
     *        with no incident segment. Each is anchored in m_feature_points exactly like a
     *        polyline endpoint: same ball, same collapse policy, same retention audit. They
     *        cannot be derived here the way endpoints and junctions are, because their
     *        valence in E is 0 -- the same valence as every background-grid vertex.
     * @param E_env_input,E_input_set with several inputs: per row of E_env, the input it came
     *        from, which builds m_input_envelopes; per row of E, its input set. Null => one
     *        input, and every tracked segment gets input 0.
     */
    void init_mesh(
        const MatrixXd& V,
        const std::vector<Vector2r>& V_rational,
        const MatrixXi& F,
        const MatrixXi& E,
        const std::vector<std::string>& tag_names,
        const MatrixXd& V_env,
        const MatrixXi& E_env,
        const std::vector<int>* E_orientation = nullptr,
        const std::vector<size_t>& free_point_vids = {},
        const std::vector<int>* E_env_input = nullptr,
        const std::vector<InputSet>* E_input_set = nullptr);

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


    void write_vtu(const std::string& path, MeshMap* output = nullptr) const;

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

    /**
     * @brief Winding number of every face's centroid w.r.t. the tracked curves, oriented as the
     * input was (m_tracks_orientation must be set), into FaceAttributes::m_winding_number.
     *
     * The tracked counterpart of compute_winding_numbers: evaluated on the curves the mesh
     * actually conforms to rather than on the input, and over all inputs at once -- coincident
     * curves add up, opposite ones cancel. Like the input winding number, a result with nothing
     * inside is taken as an inside-out input and flipped.
     */
    void compute_tracked_winding_number();
    /// Remove the faces whose tracked winding number is at most 1/2.
    void filter_with_tracked_winding_number();
    /// Remove the flood-fill region that dominates the mesh boundary (needs flood_fill).
    void filter_with_flood_fill();

    int flood_fill();
    /// Whether flood_fill ran, so that the writers leave the part ids out rather than writing
    /// their defaults as if they were parts.
    bool m_has_parts = false;

protected:
    void write_smoothing_debug_output(const std::string& path) const override { write_vtu(path); }
};


} // namespace wmtk::components::triwild
