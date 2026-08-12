#pragma once

#include <wmtk/OptimizerParameters.h>
#include <wmtk/RationalPositions.h>
#include <wmtk/SurfaceTagAttributes.h>
#include <wmtk/TetMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/utils/SurfaceTopology.hpp>

#include <igl/Timer.h>

#include <atomic>
#include <functional>
#include <map>
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
 * through cell_quality()/set_cell_quality(). This keeps the shared pass, swap and smoothing
 * implementations independent of the derived cell-attribute layout; the virtual dispatch is
 * negligible beside their geometric predicates and energy evaluations.
 */
class TetOptimizerMesh : public wmtk::TetMesh, public wmtk::RationalPositions
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

    /// Whether the current collapse pass applies the target-length limit; read by
    /// collapse_edge_before, which is where that limit is enforced.
    bool m_collapse_limit_length = true;

    /// The longest edge of each current worst tet. split_all_edges force-splits exactly these
    /// edges (bypassing the length gate). Populated serially by refine_sizing_around_worst;
    /// read-only during the parallel split pass, then cleared once consumed.
    std::set<simplex::Edge> m_force_split_edges;
    /// Force-splits taken in the current split pass. Diagnostic only.
    size_t m_force_split_count = 0;

    bool is_force_split_edge(const size_t v1, const size_t v2) const
    {
        return m_force_split_edges.find(simplex::Edge(v1, v2)) != m_force_split_edges.end();
    }

    /// Per-pass claims for the shared high-valence split gate.
    std::unique_ptr<std::atomic<int>[]> m_high_valence_claim;
    size_t m_high_valence_claim_size = 0;
    std::atomic<size_t> m_high_valence_rejects = 0;

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
     * field it shares through these. Both are AMIPS^3; see MAX_ENERGY. Topological operations
     * and smoothing use the accessors so the shared algorithms remain independent of each
     * application's cell-attribute type.
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

    /// Shared TetWild/SimWild outer optimization schedule.
    int m_iterations_used = 0;
    int m_debug_print_counter = 0;
    void mesh_improvement(int max_its = 80);
    std::tuple<double, double> local_operations(
        const std::array<int, 4>& ops,
        bool collapse_limit_length = true);

    /**
     * @brief Round a vertex position to floating point, if that inverts no incident tet.
     * @return True if successful or already rounded, false otherwise.
     */
    bool round(const Tuple& v);

protected:
    // RationalPositions supplies round_all_vertices() and round_and_check_all_rounded() on
    // top of these three.
    std::vector<size_t> all_vertex_ids() const override;
    bool vertex_is_rounded(const size_t vid) const override
    {
        return m_vertex_attribute.at(vid).m_is_rounded;
    }
    // Equivalent to round() on the tuple get_vertices() would have yielded: round() reads only
    // the vid and the one-ring, and the one-ring query is answered from
    // m_vertex_connectivity[vid] regardless of which cell the tuple names.
    bool round_vertex(const size_t vid) override { return round(tuple_from_vertex(vid)); }

public:
    bool is_edge_on_surface(const Tuple& loc);
    bool is_edge_on_bbox(const Tuple& loc);
    /**
     * @brief Whether edge `e` lies on the *boundary* of the tracked surface.
     *
     * Flipping such an edge would change the surface's boundary loops, so prepare_surface_flip
     * refuses it. Defaults to false, which is exact for simwild: its surface is the interface
     * between differently tagged tets, and around an interior edge the tags change an even
     * number of times going around the tet ring, so an interface can never terminate there. It
     * can only end where it meets the bbox -- and is_edge_on_bbox() has already rejected those
     * edges by the time this is asked. tetwild overrides it because its surface is the input
     * mesh, which may be non-watertight anywhere.
     */
    virtual bool is_open_boundary_edge(const Tuple& e) { return false; }
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
    virtual std::vector<size_t> active_vertices() const;

    // TetWild's split engine. SimWild inherits it and customizes only application data,
    // annotation-only placement, and order-2 metadata through the hooks below.
    void split_all_edges();
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& loc) override;

    // TetWild's collapse engine. SimWild customizes only application policy/metadata.
    void collapse_all_edges(bool is_limit_length = true);
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    // TetWild's complete swap engine. SimWild inherits these operations and customizes only
    // the tag bookkeeping through the hooks below.
    size_t swap_all_edges_32();
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    size_t swap_all_edges_44();
    bool swap_edge_44_before(const Tuple& t) override;
    bool swap_edge_44_accept_case(const std::array<size_t, 2>& new_edge) override;
    bool swap_edge_44_after(const Tuple& t) override;

    size_t swap_all_edges_56();
    bool swap_edge_56_before(const Tuple& t) override;
    bool swap_edge_56_accept_case(const std::array<size_t, 3>& new_face) override;
    bool swap_edge_56_after(const Tuple& t) override;

    size_t swap_all_faces();
    bool swap_face_before(const Tuple& t) override;
    bool swap_face_after(const Tuple& t) override;
    size_t swap_all_edges_all();

    bool prepare_surface_flip(const Tuple& t, const std::vector<size_t>& incident_tets);

    using SurfaceTopoSignature = wmtk::utils::SurfaceTopoSignature;
    SurfaceTopoSignature surface_topology_signature() const
    {
        return wmtk::utils::surface_topology_signature(*this, [this](size_t fid) {
            return m_face_attribute[fid].m_is_surface_fs;
        });
    }
    void warn_if_surface_topology_changed(const SurfaceTopoSignature& before, const char* where)
        const
    {
        wmtk::utils::warn_if_surface_topology_changed(before, surface_topology_signature(), where);
    }

    // Operation diagnostics shared by TetWild and SimWild.
    std::atomic<int> cnt_swap = 0;
    std::atomic<int> cnt_surface_swap = 0;
    std::atomic<int> cnt_surface_swap_32 = 0, cnt_surface_swap_44 = 0, cnt_surface_swap_56 = 0;

    double swap_edge_44_energy(const std::vector<std::array<size_t, 4>>& tets, const int op_case)
        override;
    double swap_edge_56_energy(const std::vector<std::array<size_t, 4>>& tets, const int op_case)
        override;

    /**
     * @brief Envelope the resulting surface triangles are checked against.
     *
     * The surface envelope is shared by both applications. A derived mesh may additionally
     * select an order-2 feature envelope for the pull energy.
     */
    virtual std::shared_ptr<SampleEnvelope> smoothing_containment_envelope(const size_t vid) const;
    virtual std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(const size_t vid) const = 0;

    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;
    void smooth_all_vertices(const size_t n_iters = 1);

    bool invariants(const std::vector<Tuple>& t) override;

protected:
    /// Quality metric used by the driver. TetWild uses absolute AMIPS; SimWild overrides
    /// this with quality normalized by each cell's tag-dependent target.
    virtual std::tuple<double, double> optimization_quality_stats();
    virtual double optimization_stop_metric() const { return m_params.stop_energy; }
    virtual size_t refine_sizing_around_worst(double max_metric) = 0;
    virtual void write_optimization_debug_output(const std::string& path) = 0;
    virtual void optimization_sanity_checks_extra() {}
    virtual bool optimization_stop_at_float() const { return false; }

    virtual bool collapse_before_vertex(size_t, size_t, double) const { return true; }
    virtual bool collapse_quality_allowed(size_t v1, double quality, double ring_max) const
    {
        return !m_vertex_attribute.at(v1).m_is_rounded || quality <= ring_max;
    }
    virtual bool collapse_is_order_2_edge(const std::array<size_t, 2>&) { return false; }
    virtual bool
    collapse_after_connectivity(size_t, size_t, const std::vector<std::array<size_t, 2>>&)
    {
        return true;
    }
    virtual void collapse_after_vertex(size_t, size_t) {}

    /// Cache application cell data before a split. TetWild needs none; SimWild caches tags.
    virtual bool split_before_cells(const Tuple&, const std::vector<Tuple>&) { return true; }
    /// Restore application cell data on the children made by a split.
    virtual bool split_after_cells(size_t, size_t, size_t, const std::vector<Tuple>&)
    {
        return true;
    }
    /// Optional annotation-only adjustment of the midpoint. Ordinary optimization returns true
    /// without changing the shared TetWild position.
    virtual bool split_adjust_position(size_t, const std::vector<Tuple>&) { return true; }
    /// Application metadata not represented by the shared vertex attributes.
    virtual void split_after_vertex(size_t, bool) {}

    virtual bool allow_surface_swap() const = 0;
    virtual bool check_surface_topology() const = 0;

    /// Application data attached to the old cells. TetWild has none; SimWild caches tags.
    virtual bool swap_before_interior(const std::vector<size_t>&) { return true; }
    virtual bool swap_before_surface(const std::vector<size_t>&, size_t, size_t, size_t, size_t)
    {
        return true;
    }
    /// Propagate application data to the cells made by a successful topological swap.
    virtual bool swap_after_cells(const std::vector<size_t>&, bool) { return true; }

    struct SwapInfoCache
    {
        double max_energy;
        std::map<std::array<size_t, 3>, FaceAttributes> changed_faces;

        bool is_surface_flip = false;
        size_t sf_a = 0, sf_b = 0, sf_c = 0, sf_d = 0;
        FaceAttributes sf_face_attr;
    };
    wmtk::threading::enumerable_thread_specific<SwapInfoCache> swap_cache;

    struct SplitInfoCache
    {
        size_t v1_id = 0;
        size_t v2_id = 0;
        bool is_edge_on_surface = false;
        bool is_edge_open_boundary = false;
        size_t edge_order = 0;
        double max_quality_before = 0.;
        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
    };
    wmtk::threading::enumerable_thread_specific<SplitInfoCache> split_cache;

    struct CollapseInfoCache
    {
        size_t v1_id = 0;
        size_t v2_id = 0;
        double max_energy = 0.;
        double edge_length = 0.;
        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
        std::vector<std::array<size_t, 3>> surface_faces;
        std::vector<std::array<size_t, 2>> boundary_edges;
        std::vector<size_t> changed_tids;
        std::vector<double> changed_energies;
    };
    wmtk::threading::enumerable_thread_specific<CollapseInfoCache> collapse_cache;
};

} // namespace wmtk
