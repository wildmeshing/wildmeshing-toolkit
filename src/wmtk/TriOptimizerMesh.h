#pragma once

#include <wmtk/OptimizerParameters.h>
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
class TriOptimizerMesh : public wmtk::TriMesh
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
         *
         * Carried here rather than in the derived class because an attribute collection has
         * one element type; simwild's 2D mesh has no 0-dimensional features and leaves it at
         * NO_FEATURE.
         */
        size_t m_feature_id = std::numeric_limits<size_t>::max();

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

    /**
     * @brief True when every vertex is known to be rounded.
     *
     * Only trusted when true, and only round_all_vertices() sets it that way. Any code that
     * leaves a vertex un-rounded must clear it, or the sweep will skip the vertex forever.
     * Atomic because operations that clear it run in parallel.
     */
    std::atomic<bool> m_all_rounded = false;

    bool m_collapse_limit_length = true;
    int m_debug_print_counter = 0;

    size_t m_tags_count = 0;
    std::map<int64_t, std::string> m_tag_id_to_name;
    std::map<std::string, int64_t> m_tag_name_to_id;

    explicit TriOptimizerMesh(OptimizerParameters& params)
        : m_params(params)
    {
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

    /**
     * @brief Round a vertex position to floating point, if that inverts no incident face.
     * @return True if successful or already rounded, false otherwise.
     */
    bool round(const Tuple& v);
    /**
     * @brief Try to round every un-rounded vertex; returns the number reclaimed.
     *
     * round() is otherwise only attempted as a side effect of another operation (smoothing
     * the vertex, or the merged vertex of a collapse), and neither reaches a vertex that only
     * becomes roundable later: smoothing skips "good" regions by default. Without a sweep such
     * a vertex keeps exact coordinates into the output for no geometric reason -- and
     * split_edge_after introduces them unconditionally whenever the rounded midpoint would
     * invert, so the sweep is what keeps that from reaching the output.
     *
     * Skipped outright when m_all_rounded says there is nothing to do.
     */
    size_t round_all_vertices();

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
};

} // namespace wmtk
