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
        p_edge_attrs = &m_edge_attribute;
        p_face_attrs = &m_face_attribute;

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
     * Takes the floating path when all three vertices are rounded -- igl::predicates::orient2d
     * is exact for the doubles it is handed -- and the Rational cross product otherwise. The
     * rational branch exists because for an un-rounded vertex the double is the wrong number,
     * not because orient2d is imprecise.
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
     * @brief Round a vertex position to floating point, if that inverts no incident face.
     * @return True if successful or already rounded, false otherwise.
     */
    bool round(const Tuple& v);

protected:
    virtual void collapse_pass_begin() {}
    virtual void collapse_pass_end(size_t) {}
    virtual bool collapse_before_vertex(size_t, size_t) { return true; }
    virtual bool collapse_quality_allowed(size_t v1, size_t, double q, double ring_max) const
    {
        return !m_vertex_attribute.at(v1).m_is_rounded ||
               q <= m_params.stop_energy || q <= ring_max;
    }
    virtual void collapse_after_vertex(size_t, size_t) {}

    virtual bool split_adjust_position(size_t, const std::vector<Tuple>&) { return true; }
    virtual void split_after_vertex(size_t) {}

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
    };
    wmtk::threading::enumerable_thread_specific<CollapseInfoCache> collapse_cache;

private:
    struct SwapInfoCache
    {
        double max_energy;
        std::map<simplex::Edge, EdgeAttributes> changed_edges;
        std::set<int64_t> face_tags;
    };
    wmtk::threading::enumerable_thread_specific<SwapInfoCache> swap_cache;
};

} // namespace wmtk
