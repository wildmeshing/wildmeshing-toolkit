#pragma once

#include <wmtk/OptimizerParameters.h>
#include <wmtk/RationalPositions.h>
#include <wmtk/SurfaceTagAttributes.h>
#include <wmtk/TriMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>

#include <polysolve/nonlinear/Problem.hpp>

#include <atomic>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <vector>

namespace wmtk {

/**
 * @brief What triwild and simwild's 2D mesh share.
 *
 * Seeded by MOVING triwild's implementation here, not by designing an abstraction: triwild is
 * the more heavily measured of the two, so the move leaves it byte-identical and confines any
 * output change to the commit where simwild adopts the base.
 *
 * The 3D counterpart is wmtk::TetOptimizerMesh. The two are deliberately NOT unified with each
 * other: the same algorithms at two dimensions, but written against different primitives, and
 * one shared copy would need the quality convention (2D stores AMIPS2D, 3D stores AMIPS^3) and
 * the stop condition (absolute energy vs per-cell relative quality) abstracted first. Two
 * readable copies beat one over-parameterized one.
 */
class TriOptimizerMesh : public wmtk::TriMesh, public wmtk::RationalPositions
{
public:
    struct VertexAttributes
    {
        Vector2d m_posf; // position as double
        Vector2r m_pos; // exact position in rational
        /**
         * If a vertex cannot be rounded without inverting an incident face, the exact position
         * must be used. Once the vertex can be rounded to double precision, the rational
         * representation is obsolete.
         */
        bool m_is_rounded = false;

        bool m_is_on_surface = false;
        std::vector<int> on_bbox_faces;

        double m_sizing_scalar = 1;

        size_t partition_id = 0;

        VertexAttributes() {}
        VertexAttributes(const Vector2d& p)
            : m_posf(p)
            , m_pos(to_rational(p))
            , m_is_rounded(true)
        {}
        VertexAttributes(const Vector2r& p)
            : m_posf(to_double(p))
            , m_pos(p)
        {}
    };

    using EdgeAttributes = wmtk::SurfaceTagAttributes;

    struct FaceAttributes
    {
        double m_quality;
        double m_winding_number = 0;
        std::set<int64_t> tags;
        int part_id = -1;
    };

    using VertAttCol = AttributeCollection<VertexAttributes>;
    using EdgeAttCol = AttributeCollection<EdgeAttributes>;
    using FaceAttCol = AttributeCollection<FaceAttributes>;

    VertAttCol m_vertex_attribute;
    EdgeAttCol m_edge_attribute;
    FaceAttCol m_face_attribute;

    /**
     * @brief What p_vertex_attrs points at, so a derived class can register more.
     *
     * VertexAttributes holds only what both 2D applications need. triwild adds a per-vertex
     * feature id through here; simwild-2D has no 0-dimensional features and registers nothing,
     * so it does not carry the field.
     */
    AttributeContainerGroup m_vertex_attr_group;

    /**
     * @brief What p_edge_attrs points at, so a derived class can register more.
     *
     * The 2D counterpart of wmtk::TetOptimizerMesh::m_face_attr_group. An application whose
     * edges carry data the shared operations know nothing about -- topological_offset labels
     * each edge by which part of the construction produced it -- registers a second collection
     * here, and it is resized, protected and rolled back with the shared one.
     *
     * Note what this does NOT do: the shared operations propagate edge data by copying
     * SurfaceTagAttributes values around (the split and collapse caches, the swap cache), and
     * they only ever see their own collection. Anything registered here survives the mesh
     * changing shape, but its values are the registering application's to maintain.
     */
    AttributeContainerGroup m_edge_attr_group;

    /**
     * @brief What p_face_attrs points at, so a derived class can register more.
     *
     * Unlike the 3D mesh, whose cell attributes are per-application, the 2D base owns a
     * concrete FaceAttributes -- so a derived class that needs an extra per-face field cannot
     * add it there and registers a second collection here instead. Same caveat as the edge
     * group: the shared operations propagate face data by copying FaceAttributes values
     * around and never see this, so its values are the registering application's to maintain.
     */
    AttributeContainerGroup m_face_attr_group;

    /**
     * @brief The sentinel get_quality returns for a face AMIPS2D cannot score.
     *
     * Not an energy: a positively oriented triangle whose area is too small for AMIPS, or one
     * that produces inf/nan, gets this instead. Unlike the 3D mesh this stores AMIPS2D
     * directly, so it surfaces in the logs verbatim as 1e+50. 1e50 rather than double::max
     * because every downstream ratio must stay finite.
     */
    static constexpr double MAX_ENERGY = 1e50;

    /// Only the fields shared by all three applications; each derived class keeps a typed
    /// reference to its own Parameters for the rest.
    OptimizerParameters& m_params;

    std::shared_ptr<SampleEnvelope> m_envelope;
    double m_envelope_eps = -1;

    /// Scale factors putting AMIPS (dimensionless) and the envelope energy (a squared
    /// distance) on a comparable footing.
    double m_s_amips = -1;
    double m_s_envelope = -1;

    wmtk::threading::enumerable_thread_specific<std::unique_ptr<polysolve::nonlinear::Solver>>
        m_solver;

    /// Why smoothing attempts were refused, reported once per pass.
    optimization::SmoothRejectCounters m_smooth_rejects;

    bool m_collapse_limit_length = true;
    int m_debug_print_counter = 0;

    size_t m_tags_count = 0;
    std::map<int64_t, std::string> m_tag_id_to_name;
    std::map<std::string, int64_t> m_tag_name_to_id;

    explicit TriOptimizerMesh(OptimizerParameters& params)
        : m_params(params)
    {
        m_vertex_attr_group.add(&m_vertex_attribute);
        p_vertex_attrs = &m_vertex_attr_group;
        m_edge_attr_group.add(&m_edge_attribute);
        p_edge_attrs = &m_edge_attr_group;
        m_face_attr_group.add(&m_face_attribute);
        p_face_attrs = &m_face_attr_group;

        m_s_amips = 1.;
        /**
         * eps makes it such that the energy is relative to the envelope thickness. As it's a
         * squared energy, we need eps^2.
         */
        m_s_envelope = 1. / (m_params.eps * m_params.eps);
    }
    ~TriOptimizerMesh() override = default;

    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }
    void partition_mesh();
    void partition_mesh_morton();

    double get_length2(const Tuple& l) const;

    /**
     * @brief Orientation check, exact for the coordinates the vertices actually carry.
     *
     * Takes the floating path when all three vertices are rounded --
     * wmtk::utils::predicates::orient2d is exact for the doubles it is handed -- and the Rational
     * cross product otherwise. The rational branch exists because for an un-rounded vertex the
     * double is the wrong number, not because orient2d is imprecise.
     */
    bool is_inverted(const std::array<size_t, 3>& vs) const;
    bool is_inverted(const Tuple& loc) const;
    bool is_inverted(const size_t fid) const;
    /// Inversion check using only the double positions.
    bool is_inverted_f(const Tuple& loc) const;
    bool is_inverted_f(const size_t fid) const;

    double get_quality(const std::array<size_t, 3>& vs) const;
    double get_quality(const Tuple& loc) const;
    double get_quality(const size_t fid) const;

    std::tuple<double, double> get_max_avg_energy();

    /// Shared TriWild/SimWild outer optimization schedule.
    int m_iterations_used = 0;
    void mesh_improvement(int max_its = 80);
    std::tuple<double, double> local_operations(
        const std::array<int, 4>& ops,
        bool collapse_limit_length = true);

    std::set<simplex::Edge> m_force_split_edges;
    size_t m_force_split_count = 0;
    std::unique_ptr<std::atomic<int>[]> m_high_valence_claim;
    size_t m_high_valence_claim_size = 0;
    std::atomic<size_t> m_high_valence_rejects = 0;

    bool is_force_split_edge(const size_t v1, const size_t v2) const
    {
        return m_force_split_edges.find(simplex::Edge(v1, v2)) != m_force_split_edges.end();
    }

    void split_all_edges();
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& loc) override;

    void collapse_all_edges(bool is_limit_length = true);
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    /**
     * @brief Coarsen the mesh without letting the max energy rise.
     *
     * Alternates coarsening-collapse passes with ordinary smoothing passes, as the main
     * optimization does, for at most m_params.coarsen_max_rounds rounds or until a round
     * accepts nothing. See OptimizerParameters::coarsen_pass for what makes the collapse in
     * this pass different, and collapse_edge_after for how it is undone when it does not pay
     * off. A no-op when m_params.coarsen_pass is false.
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
    bool coarsen_collapse_edge(const Tuple& e, std::vector<Tuple>& new_tris);

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

    /**
     * @brief Run TriWild's quality-improving interior edge-flip pass.
     *
     * This operation is shared verbatim by TriWild and SimWild. Face tags are copied from
     * the old pair to the new pair, so a tag-homogeneous SimWild mesh follows exactly the
     * same path as TriWild.
     */
    size_t swap_all_edges();
    double swap_weight(const Tuple& t) const;
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    /**
     * @brief Run TriWild's vertex-smoothing pass.
     *
     * TriWild is the behavioral source of truth. SimWild shares the pass verbatim, including
     * the single m_envelope used for both smoothing energy and containment, the surface
     * quality veto, and skip_good_regions selection.
     */
    void smooth_all_vertices(size_t n_iters = 1);
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    Vector2d smoothing_position(size_t vid) const;
    void set_smoothing_position(size_t vid, const Vector2d& p);
    virtual bool smoothing_position_is_allowed(size_t vid, const Vector2d& p) const = 0;

    double active_quality_threshold() const
    {
        return m_params.skip_good_regions_margin * m_params.stop_energy;
    }
    virtual std::vector<size_t> active_vertices() const;

    /**
     * @brief A face's quality relative to the quality it is required to reach; <= 1 means it
     * meets it.
     *
     * The only quality that is comparable ACROSS faces. Raw m_quality is not, once an
     * application gives different regions different targets: the loose region's raw ceiling
     * then hides degradation in the strict one, because a strict face may get much worse and
     * still sit below a number set somewhere it has nothing to do with. Shared code that
     * compares one face's quality against another's must go through here.
     *
     * TriWild's target is uniform, so this is the raw quality over a constant -- which cancels
     * out of any before/after comparison and leaves its behaviour unchanged.
     */
    virtual double quality_rel(const size_t fid) const
    {
        return m_face_attribute.at(fid).m_quality / m_params.stop_energy;
    }

    /**
     * @brief Round a vertex position to floating point, if that inverts no incident face.
     * @return True if successful or already rounded, false otherwise.
     */
    bool round(const Tuple& v);

protected:
    /// Quality metric used by the driver. TriWild uses absolute AMIPS; SimWild overrides
    /// this with quality normalized by each face's tag-dependent target.
    virtual std::tuple<double, double> optimization_quality_stats();
    virtual double optimization_stop_metric() const { return m_params.stop_energy; }
    virtual size_t refine_sizing_around_worst(double max_metric) = 0;
    virtual bool optimization_stop_at_float() const { return false; }

    virtual void collapse_pass_begin() {}
    virtual void collapse_pass_end(size_t) {}
    virtual bool collapse_before_vertex(size_t, size_t) { return true; }
    virtual bool collapse_quality_allowed(size_t v1, size_t, double q, double ring_max) const
    {
        return !m_vertex_attribute.at(v1).m_is_rounded || q <= m_params.stop_energy ||
               q <= ring_max;
    }
    virtual void collapse_after_vertex(size_t, size_t) {}

    virtual bool split_adjust_position(size_t, const std::vector<Tuple>&) { return true; }
    virtual void split_after_vertex(size_t) {}

    virtual void write_smoothing_debug_output(const std::string& path) const = 0;

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
    bool is_edge_on_surface(const Tuple& loc) const;
    bool is_edge_on_surface(const std::array<size_t, 2>& vids) const;
    bool is_edge_on_bbox(const Tuple& loc) const;
    bool is_edge_on_bbox(const std::array<size_t, 2>& vids) const;

    bool vertex_is_on_surface(const size_t vid) const override
    {
        return m_vertex_attribute.at(vid).m_is_on_surface ||
               !m_vertex_attribute.at(vid).on_bbox_faces.empty();
    }
    bool edge_is_on_surface(const std::array<size_t, 2>& vids) const override
    {
        if (!vertex_is_on_surface(vids[0]) || !vertex_is_on_surface(vids[1])) {
            return false;
        }

        const auto [_, eid] = tuple_from_edge(vids);
        bool on_surface = m_edge_attribute.at(eid).m_is_surface_fs;
        bool on_bbox = m_edge_attribute.at(eid).m_is_bbox_fs >= 0;
        return on_surface || on_bbox;
    }

    /**
     * @brief Envelope the tracked-surface segment `vids` must stay inside.
     *
     * The 2D counterpart of wmtk::TetOptimizerMesh::surface_envelope_for_face. Null means this
     * segment carries no containment requirement, and every containment check on it is skipped.
     * Both applications that track a single surface keep one envelope for it, so the default
     * answers with that; an application tracking more than one (see
     * SurfaceTagAttributes::m_surface_class) overrides this to answer per class.
     *
     * Keyed on the vertex ids rather than the eid because every caller is a topological
     * operation asking about segments it is ABOUT to create or has just created.
     */
    virtual std::shared_ptr<SampleEnvelope> surface_envelope_for_edge(
        const std::array<size_t, 2>& vids) const
    {
        return m_envelope;
    }

    /// Whether the tracked-surface segment (a,b) is outside its containment envelope. False
    /// when it has none. Every surface containment check in the operations goes through here.
    bool surface_segment_is_outside(const size_t a, const size_t b) const
    {
        const std::shared_ptr<SampleEnvelope> env = surface_envelope_for_edge({{a, b}});
        if (!env) return false;
        const auto& VA = m_vertex_attribute;
        return env->is_outside(std::array<Vector2d, 2>{{VA[a].m_posf, VA[b].m_posf}});
    }

    std::vector<std::array<size_t, 2>> get_edges_by_condition(
        std::function<bool(const EdgeAttributes&)> cond) const;

    /**
     * @brief Monotone (only-decreasing) gradation smoothing of the sizing field.
     *
     * Enforces m_sizing_scalar[v] <= grade * m_sizing_scalar[u] for every edge (u,v),
     * propagating outward from `seeds` with a min-relaxation. It never raises a sizing value,
     * so it only ever spreads more refinement into the halo around already-refined vertices,
     * avoiding sharp resolution jumps.
     */
    void gradation_smooth_sizing(double grade, const std::vector<size_t>& seeds);

protected:
    struct SplitInfoCache
    {
        size_t v1_id = 0;
        size_t v2_id = 0;
        double max_quality_before = 0.;
        EdgeAttributes old_e_attrs;
        std::map<simplex::Edge, EdgeAttributes> changed_edges;
        std::map<size_t, FaceAttributes> faces;
    };
    wmtk::threading::enumerable_thread_specific<SplitInfoCache> split_cache;

    struct CollapseInfoCache
    {
        size_t v1_id = 0;
        size_t v2_id = 0;
        double max_energy = 0.;
        double edge_length = 0.;
        std::vector<std::pair<EdgeAttributes, std::array<size_t, 2>>> changed_edges;
        std::vector<std::array<size_t, 2>> surface_edges;
        std::vector<size_t> changed_fids;
        std::vector<double> changed_energies;
        /// Coarsening pass only: the worst relative quality in the region the composite may
        /// disturb, measured before the collapse. See collapse_edge_after.
        double region_max_rel_before = 0.;
    };
    wmtk::threading::enumerable_thread_specific<CollapseInfoCache> collapse_cache;

    /// Set for the duration of coarsen_mesh(); read-only while a pass is running.
    bool m_coarsen_mode = false;

private:
    struct SwapInfoCache
    {
        double max_energy;
        std::map<simplex::Edge, EdgeAttributes> changed_edges;
        std::set<int64_t> face_tags;
    };
    wmtk::threading::enumerable_thread_specific<SwapInfoCache> swap_cache;

    /**
     * @brief Per-thread buffers for the coarsening composite, so it allocates nothing.
     *
     * `saved` holds one vertex's attributes and the quality of its incident faces, which is
     * everything an individual smooth writes -- smooth_vertex_2d returns false only after
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

    /// Worst relative quality (quality_rel) over the faces incident to any vertex of
    /// @p vids.
    double region_max_quality_rel(const std::vector<size_t>& vids) const;

    /// One smoothing attempt on @p vid, restoring everything it wrote if it is rejected.
    bool smooth_vertex_reversible(size_t vid, CoarsenScratch& scr);

    size_t collapse_all_edges_impl(bool is_limit_length, int lock_ring);
};

} // namespace wmtk
