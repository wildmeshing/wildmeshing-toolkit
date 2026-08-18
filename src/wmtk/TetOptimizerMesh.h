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
#include <cmath>
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
     * @brief What p_face_attrs points at, so a derived class can register more.
     *
     * The counterpart of m_vertex_attr_group for faces. An application whose faces carry data
     * the shared operations know nothing about -- topological_offset labels each face by which
     * part of the construction produced it -- registers a second collection here, and it is
     * resized, protected and rolled back with the shared one.
     *
     * Note what this does NOT do: the shared operations propagate face data by copying
     * SurfaceTagAttributes values around (the split and collapse caches, the swap tracker), and
     * they only ever see their own collection. Anything registered here survives the mesh
     * changing shape, but its values are the registering application's to maintain.
     */
    AttributeContainerGroup m_face_attr_group;

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
        m_face_attr_group.add(&m_face_attribute);
        p_face_attrs = &m_face_attr_group;
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

    /**
     * @brief A cell's quality relative to the quality it is required to reach; <= 1 means it
     * meets it.
     *
     * The only quality that is comparable ACROSS cells. Raw cell_quality is not, once an
     * application gives different regions different targets: the loose region's raw ceiling
     * then hides degradation in the strict one, because a strict cell may get much worse and
     * still sit below a number set somewhere it has nothing to do with. Shared code that
     * compares one cell's quality against another's must go through here.
     *
     * The cube root is because cell_quality stores AMIPS^3, so that this means the same thing
     * as its 2D twin: AMIPS relative to target, not AMIPS^3 relative to target. TetWild's
     * target is uniform, so this is a monotone function of the raw quality divided by a
     * constant -- neither of which can change the outcome of a before/after comparison, so its
     * behaviour is unchanged.
     */
    virtual double quality_rel(const size_t tid) const
    {
        return std::cbrt(cell_quality(tid)) / m_params.stop_energy;
    }

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

    /**
     * @brief Envelope the tracked-surface triangle `vids` must stay inside.
     *
     * Null means this triangle carries no containment requirement, and every containment check
     * on it is skipped. Both applications that track a single surface keep one envelope for it,
     * so the default answers with that; an application tracking more than one surface (see
     * SurfaceTagAttributes::m_surface_class) overrides this to answer per class -- the offset
     * boundary, for instance, is free to move and is checked against nothing.
     *
     * Keyed on the vertex ids rather than the fid because every caller is a topological
     * operation asking about the triangles it is ABOUT to create or has just created, where
     * only the vids are known for certain.
     */
    virtual std::shared_ptr<SampleEnvelope> surface_envelope_for_face(
        const std::array<size_t, 3>& vids) const
    {
        return m_envelope;
    }

    /// Whether the tracked-surface triangle (a,b,c) is outside its containment envelope. False
    /// when it has none. Every surface containment check in the operations goes through here.
    bool surface_triangle_is_outside(const size_t a, const size_t b, const size_t c) const
    {
        const std::shared_ptr<SampleEnvelope> env = surface_envelope_for_face({{a, b, c}});
        if (!env) return false;
        const auto& VA = m_vertex_attribute;
        return env->is_outside({{VA[a].m_posf, VA[b].m_posf, VA[c].m_posf}});
    }

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

    /**
     * @brief Coarsen the mesh without letting the max energy rise.
     *
     * The 3D twin of TriOptimizerMesh::coarsen_mesh; see OptimizerParameters::coarsen_pass for
     * what makes the collapse in this pass different, and collapse_edge_after for how it is
     * undone when it does not pay off. A no-op when m_params.coarsen_pass is false.
     *
     * @return the number of collapses accepted across all rounds.
     */
    size_t coarsen_mesh();

    /**
     * @brief One collapse under the coarsening rules, outside a coarsening pass.
     *
     * The pass sets the mode once and leaves it set for its whole duration, because every
     * worker reads it; this is the single-operation form, for callers that want one and are
     * not sharing the mesh with other threads.
     */
    bool coarsen_collapse_edge(const Tuple& e, std::vector<Tuple>& new_tets);

    /// What coarsen_mesh() achieved, for the run report. Zeroed when the pass is off.
    struct CoarsenStats
    {
        size_t accepted = 0;
        size_t cells_before = 0;
        size_t cells_after = 0;
        double max_energy_before = 0.;
        double max_energy_after = 0.;
    };
    CoarsenStats m_coarsen_stats;

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

    /**
     * @brief An extra term the application adds to this vertex's smoothing objective, or null.
     *
     * The extension point that lets an application place a vertex by MINIMISING something
     * rather than by computing a position and then defending it. topological_offset uses it for
     * the offset surface: the term is w (Phi - c)^2, whose minimum is the offset itself, so an
     * offset-surface vertex goes through this same smoother -- the same line search, the same
     * exact inversion test, the same quality veto -- as every other vertex, instead of through
     * the hand-rolled quadrics/Laplacian blend that bypassed all three.
     *
     * Null by default, so TetWild and SimWild compose exactly the energy they composed before.
     * The 2D twin is TriOptimizerMesh::smoothing_extra_energy.
     */
    virtual std::shared_ptr<polysolve::nonlinear::Problem> smoothing_extra_energy(
        const size_t /*vid*/) const
    {
        return nullptr;
    }

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

    /**
     * @brief Whether the loop opens with an UNLIMITED-LENGTH collapse pass.
     *
     * mesh_improvement() begins with local_operations({{0,1,0,0}}, false) -- a collapse pass
     * with collapse_limit_length FALSE, so no edge is too short to be collapsed and no edge is
     * too long to be collapsed either. For TetWild and SimWild that is exactly right: it strips
     * the redundancy left by insertion before the real work starts, and their tracked surface is
     * held in place by an envelope throughout.
     *
     * It is wrong for an application whose tracked surface is what the optimization exists to
     * PLACE, and which therefore has no envelope holding it. topological_offset's offset surface
     * is such a surface: measured on topological_offset_3d_convex, the opening pass alone took it
     * from 1172 faces to 326 -- a 72% decimation -- before the loop had run at all, and nothing
     * downstream can undo that. The sizing field can ask for refinement afterwards; it cannot
     * refuse a collapse.
     *
     * This governs the three BARE coarsening steps -- the opening pass, the closing pass, and
     * coarsen_mesh() -- and not the collapse inside the loop, which is interleaved with splits
     * and smoothing and answers to the criterion. Measured on the same model with only the
     * closing pass left enabled: 1172 faces to 350, so gating the opening one alone is not
     * enough.
     */
    virtual bool optimization_bare_coarsen_passes() const { return true; }
    virtual void write_optimization_debug_output(const std::string& path) = 0;
    virtual void optimization_sanity_checks_extra() {}
    virtual bool optimization_stop_at_float() const { return false; }

    /// Non-const: an override may cache what it measured before the collapse so its `after`
    /// counterpart can tell a regression from a defect that was already there.
    virtual bool collapse_before_vertex(size_t, size_t, double) { return true; }
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

    /**
     * @brief The survivor's sizing scalar after a collapse merges `removed` into it.
     *
     * The default keeps the survivor's own value -- today's behaviour, byte-identical for
     * every existing application. The offset overrides it to min: its stall-driven
     * refinement lowers scalars on band vertices, and a collapse that discards the removed
     * vertex's finer value onto a coarser survivor un-refines the FIELD by topology, so
     * each round re-lowers from scratch and the band never actually gets finer. min is
     * the field's established convention (refinement and gradation smoothing only ever
     * lower a scalar).
     */
    virtual double collapse_merged_sizing(double /*removed*/, double survivor) const
    {
        return survivor;
    }

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
        /// Coarsening pass only: the worst relative quality in the region the composite may
        /// disturb, measured before the collapse. See collapse_edge_after.
        double region_max_rel_before = 0.;
    };
    wmtk::threading::enumerable_thread_specific<CollapseInfoCache> collapse_cache;

    /// Set for the duration of coarsen_mesh(); read-only while a pass is running.
    bool m_coarsen_mode = false;

private:
    /**
     * @brief Per-thread buffers for the coarsening composite, so it allocates nothing.
     *
     * `saved_*` holds one vertex's attributes and the quality of its incident cells, which is
     * everything an individual smooth writes -- smooth_vertex_3d returns false only after
     * having written both, deliberately (see its contract), so a rejected smooth has to be
     * undone by its caller.
     */
    struct CoarsenScratch
    {
        std::vector<size_t> ring; // vertices to re-smooth, BFS order
        std::vector<size_t> frontier;
        std::vector<size_t> next;
        std::vector<size_t> one_ring;
        std::vector<uint32_t> stamp;
        uint32_t epoch = 0;
        VertexAttributes saved_vertex;
        std::vector<std::pair<size_t, double>> saved_qualities;
    };
    wmtk::threading::enumerable_thread_specific<CoarsenScratch> coarsen_scratch;

    /// Vertices within @p n edges of @p seeds, in BFS order. Uses coarsen_scratch.
    const std::vector<size_t>&
    collect_vertex_ball(const size_t* seeds, size_t n_seeds, int n, CoarsenScratch& scr) const;

    /// Worst relative quality (quality_rel) over the cells incident to any vertex of
    /// @p vids.
    double region_max_quality_rel(const std::vector<size_t>& vids) const;

    /// One smoothing attempt on @p vid, restoring everything it wrote if it is rejected.
    bool smooth_vertex_reversible(size_t vid, CoarsenScratch& scr);

    /// @param exact_ball_lock claim the full lock_ring ball via try_set_edge_mutex_n_ring,
    /// bypassing make_locker's default-radius substitution of the partial two-ring helper
    /// (which under-claims in 3D -- see "Ring lockers -- NOT balls" in TetMesh.h). Required
    /// whenever the operation writes beyond the seed's one-ring, as the coarsening
    /// composite's ring smoothing does.
    size_t collapse_all_edges_impl(
        bool is_limit_length,
        int lock_ring,
        size_t max_passes = 0,
        bool exact_ball_lock = false);
};

} // namespace wmtk
