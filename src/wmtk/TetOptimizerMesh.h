#pragma once

#include <wmtk/OptimizerParameters.h>
#include <wmtk/SurfaceTagAttributes.h>
#include <wmtk/TetMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>

#include <igl/Timer.h>

#include <atomic>
#include <functional>
#include <memory>
#include <set>
#include <vector>

namespace wmtk {

/**
 * @brief What tetwild and simwild's 3D mesh share.
 *
 * Seeded by MOVING tetwild's implementation here, not by designing an abstraction: tetwild is
 * the more heavily measured of the two, so the move leaves it byte-identical and confines any
 * output change to the commit where simwild adopts the base.
 *
 * The 2D counterpart is wmtk::TriOptimizerMesh. The two are deliberately NOT unified with each
 * other -- see that class for why.
 *
 * **Cell attributes stay per-application.** tetwild's tets carry winding numbers and a
 * flood-fill id, simwild's carry a CellTag set, and an AttributeCollection member cannot change
 * type in a derived class. The base therefore reaches the one field it needs -- the quality --
 * through cell_quality()/set_cell_quality(). That costs a virtual call, but only in
 * active_vertices() and get_max_avg_energy(): of everything moved here, those are the only two
 * that touch a cell attribute at all, and both are once-per-pass sweeps rather than
 * per-operation code.
 */
class TetOptimizerMesh : public wmtk::TetMesh
{
public:
    struct VertexAttributes
    {
        Vector3r m_pos; // exact position in rational
        Vector3d m_posf; // position as double
        /**
         * If a vertex cannot be rounded without inverting a tet, the exact position must be
         * used. Once the vertex can be rounded to double precision, the rational
         * representation is obsolete.
         */
        bool m_is_rounded = false;

        bool m_is_on_surface = false;
        /**
         * The order of a vertex in a TetMesh is as follows:
         * 0: vertex is not on the surface
         * 1: vertex is on the surface
         * 2: vertex is on the boundary of the surface or a non-manifold edge
         * 3: vertex is at the boundary of a non-manifold edge or a non-manifold vertex
         */
        size_t m_order = 0;
        std::vector<int> on_bbox_faces;

        double m_sizing_scalar = 1;

        /// Required for multi-threading.
        size_t partition_id = 0;

        VertexAttributes() {}
        VertexAttributes(const Vector3r& p);
    };

    using FaceAttributes = wmtk::SurfaceTagAttributes;

    using VertAttCol = AttributeCollection<VertexAttributes>;
    using FaceAttCol = AttributeCollection<FaceAttributes>;

    VertAttCol m_vertex_attribute;
    FaceAttCol m_face_attribute;

    /**
     * @brief What p_vertex_attrs points at, so a derived class can register more.
     *
     * VertexAttributes holds only what both 3D applications need. tetwild adds a per-vertex
     * open-boundary flag through here; simwild never sets one, so it does not carry the field.
     */
    AttributeContainerGroup m_vertex_attr_group;

    /**
     * @brief The sentinel get_quality returns for an element AMIPS cannot score.
     *
     * Not an energy: a positively oriented tet whose volume is too small for AMIPS, or one that
     * produces inf/nan, gets this instead. The cell quality holds AMIPS^3, so it surfaces in
     * the logs as its cube root, cbrt(1e50) = 4.6e16. 1e50 rather than double::max because
     * every downstream ratio must stay finite -- avg_energy sums qualities and would go inf.
     */
    static constexpr double MAX_ENERGY = 1e50;

    /// Only the fields shared by all three applications; each derived class keeps a typed
    /// reference to its own Parameters for the rest.
    OptimizerParameters& m_params;

    /// Surface envelope: what a surface vertex is pulled toward and checked against.
    std::shared_ptr<SampleEnvelope> m_envelope;

    /// Scale factors putting AMIPS (dimensionless) and the envelope energy (a squared
    /// distance) on a comparable footing.
    double m_s_amips = 1.;
    double m_s_envelope = -1.;

    double time_env = 0.0;
    igl::Timer isout_timer;

    /// Per-thread Newton solver for smoothing; created on first use.
    wmtk::threading::enumerable_thread_specific<std::unique_ptr<polysolve::nonlinear::Solver>>
        m_solver;

    /// Why smoothing attempts were refused, reported once per pass.
    optimization::SmoothRejectCounters m_smooth_rejects;

    /**
     * @brief True when every vertex is known to be rounded.
     *
     * Only trusted when true, and only round_all_vertices() sets it that way. Any code that
     * leaves a vertex un-rounded must clear it, or the sweep will skip the vertex forever.
     * Atomic because operations that clear it run in parallel.
     */
    std::atomic<bool> m_all_rounded = false;

    /// Whether the current collapse pass applies the target-length limit; read by
    /// collapse_edge_before, which is where that limit is enforced.
    bool m_collapse_limit_length = true;

    /// The longest edge of each current worst tet. split_all_edges force-splits exactly these
    /// edges (bypassing the length gate). Populated serially by refine_sizing_around_worst;
    /// read-only during the parallel split pass, then cleared once consumed.
    std::set<simplex::Edge> m_force_split_edges;
    /// Force-splits taken in the current split pass. Diagnostic only.
    size_t m_force_split_count = 0;

    explicit TetOptimizerMesh(OptimizerParameters& params, std::shared_ptr<SampleEnvelope> env)
        : m_params(params)
        , m_envelope(std::move(env))
    {
        m_vertex_attr_group.add(&m_vertex_attribute);
        p_vertex_attrs = &m_vertex_attr_group;
        p_face_attrs = &m_face_attribute;
    }
    ~TetOptimizerMesh() override = default;

    /**
     * @brief The quality of cell `tid`, and how to write it.
     *
     * The cell attribute types differ between the applications, so the base reaches the one
     * field it shares through these. Both are AMIPS^3; see MAX_ENERGY.
     */
    virtual double cell_quality(const size_t tid) const = 0;
    virtual void set_cell_quality(const size_t tid, const double q) = 0;

    // TODO This should not be here but inside wmtk
    void compute_vertex_partition();
    void compute_vertex_partition_morton();

    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }

    double get_length2(const Tuple& l) const;

    bool is_inverted(const std::array<size_t, 4>& vs) const;
    bool is_inverted(const Tuple& loc) const;
    /// Inversion check using only the double positions.
    bool is_inverted_f(const Tuple& loc) const;

    double get_quality(const std::array<size_t, 4>& vs) const;
    double get_quality(const Tuple& loc) const;
    std::tuple<double, double> get_max_avg_energy();

    /**
     * @brief Round a vertex position to floating point, if that inverts no incident tet.
     * @return True if successful or already rounded, false otherwise.
     */
    bool round(const Tuple& v);
    /**
     * @brief Try to round every un-rounded vertex; returns the number reclaimed.
     *
     * Skipped outright when m_all_rounded says there is nothing to do.
     */
    size_t round_all_vertices();

    bool is_edge_on_surface(const Tuple& loc);
    bool is_edge_on_bbox(const Tuple& loc);
    /**
     * @brief How many of the faces incident to edge `e` are on the tracked surface.
     *
     * Unlike is_edge_on_surface(), this does NOT short-circuit on the (possibly stale)
     * per-vertex flag -- it counts the incident tets' face attributes directly.
     */
    int edge_incident_surface_face_count(const Tuple& e);

    bool vertex_is_on_surface(const size_t vid) const override;
    bool face_is_on_surface(const size_t fid) const override;
    size_t get_order_of_vertex(const size_t vid) const override;

    std::vector<std::array<size_t, 3>> get_faces_by_condition(
        std::function<bool(const FaceAttributes&)> cond) const;

    void output_faces(std::string file, std::function<bool(const FaceAttributes&)> cond);

    /// Grade the refined sizing region into its surroundings (monotone, only lowers).
    void gradation_smooth_sizing(double grade, const std::vector<size_t>& seeds);

    /**
     * @brief Cell-quality threshold above which a tet is "active" (worth operating on) for the
     * skip-good-regions filter.
     *
     * The quality stores AMIPS^3 and the energy is its cube root, so a tet is active when its
     * energy is at least skip_good_regions_margin * stop_energy, i.e. the quality is at least
     * (margin * stop_energy)^3.
     */
    double active_quality_threshold() const
    {
        const double e = m_params.skip_good_regions_margin * m_params.stop_energy;
        return e * e * e;
    }

    /// vids of the vertices incident to at least one "active" cell, for the
    /// skip-good-regions filter.
    std::vector<size_t> active_vertices() const;

    double swap_edge_44_energy(const std::vector<std::array<size_t, 4>>& tets, const int op_case)
        override;
    double swap_edge_56_energy(const std::vector<std::array<size_t, 4>>& tets, const int op_case)
        override;

    /**
     * @brief Envelope the resulting surface triangles are checked against.
     *
     * Virtual because tetwild keeps one surface envelope, so its pull target and containment
     * test coincide, while simwild has a separate working envelope.
     */
    virtual std::shared_ptr<SampleEnvelope> smoothing_containment_envelope(const size_t vid) const;

    bool smooth_before(const Tuple& t) override;

    bool invariants(const std::vector<Tuple>& t) override;
};

} // namespace wmtk
