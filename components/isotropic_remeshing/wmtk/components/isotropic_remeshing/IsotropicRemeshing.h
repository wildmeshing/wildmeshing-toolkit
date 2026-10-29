#pragma once

#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/envelope/BoundaryEnvelope.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include "wmtk/AttributeCollection.hpp"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/write_triangle_mesh.h>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <memory>
#include <optional>
#include <queue>

namespace wmtk::components::isotropic_remeshing {

struct VertexAttributes
{
    Eigen::Vector3d pos;
    // TODO: in fact, partition id should not be vertex attribute, it is a fixed marker to distinguish tuple/operations.
    size_t partition_id;
    bool freeze = false;
    /// The vertex is, or descends from, a vertex on a boundary edge of the input: merged from
    /// one by a collapse, or inserted by a split on a boundary edge touching one. Only set when
    /// create_mesh() gets a boundary_eps: it marks which boundary edges the boundary envelope
    /// judges. See wmtk::BoundaryEnvelope.
    bool input_boundary = false;
};

/// A vertex's neighbours across feature edges (see IsotropicRemeshing::set_feature_edges).
struct FeatureAttributes
{
    std::vector<size_t> nbrs;
    /// The id of the feature edge to each of nbrs, in the same order; -1 for none.
    std::vector<int64_t> ids;
    /// Set by IsotropicRemeshing::set_fixed_vertices: never moved, never removed.
    bool fixed = false;
    /// The id set_fixed_vertices gave it; -1 for none.
    int64_t vertex_id = -1;
};

/// A face's values of the per-face fields (see IsotropicRemeshing::set_face_fields).
struct FaceFieldAttributes
{
    std::vector<double> values;
};

/// The triangles an operation replaces, as IsotropicRemeshing::set_check_inversions sees them.
struct ReplacedSurface
{
    /// Their area vectors, summed.
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    /// The ones that do not face along `normal`.
    size_t against = 0;
    /// The folds in the stars of their vertices: triangles that do not face along their star's
    /// summed area vector, over those stars.
    size_t star_folds = 0;
    /// Whether they, or the stars around them, fold the surface over already.
    bool folded() const { return against > 0 || star_folds > 0; }
};

class IsotropicRemeshing : public wmtk::TriMesh
{
public:
    wmtk::Envelope m_envelope;
    bool m_has_envelope = false;
    /// Tube around the boundary edges of the mesh given to create_mesh(), used instead of
    /// freezing the boundary when that gets a positive boundary_eps. See there.
    /// initialized() is false otherwise, and for a closed input.
    wmtk::BoundaryEnvelope m_boundary_envelope;
    /// Around the feature edges set_feature_envelope() saw; initialized() is false without one.
    wmtk::Envelope m_feature_envelope;

    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    VertAttCol vertex_attrs;
    /// Read and written only while the mesh has feature edges or fixed vertices.
    wmtk::AttributeCollection<FeatureAttributes> feature_attrs;
    /// The mesh's face attributes once set_face_fields() is called; untouched before.
    wmtk::AttributeCollection<FaceFieldAttributes> face_field_attrs;

    /**
     * @brief Curves the remeshing keeps: label boundaries, say.
     *
     * A vertex on two feature edges is on a curve; one with one, or with three or more, ends a
     * curve or joins several, and stays where it is. A split of a feature edge leaves two
     * feature halves. A curve vertex collapses only along its curve, onto the survivor's
     * position -- with set_feature_envelope(), onto the midpoint unless the survivor stays put;
     * a vertex off every curve collapses as before, onto the survivor's position if that one is
     * on a curve. A feature edge is never swapped, and a curve vertex is smoothed
     * along its curve only. No swap or collapse makes a triangle holding two curve edges, and
     * remove_feature_ears() removes the ones the input has.
     *
     * These are the rules wmtk::history::Replay carries feature edges by, so a recorded history
     * holds the curves at every step.
     */
    void set_feature_edges(const std::vector<std::array<size_t, 2>>& edges);
    /**
     * @brief The same, with an id for each edge (-1 for none), which the remeshing carries: the
     * halves of a split keep it, and a collapse along a curve leaves the curve its id. A vertex
     * on two feature edges of different ids ends one curve and starts the next, so it stays put
     * like a junction (see stays_put). `ids` has one entry per edge.
     */
    void set_feature_edges(
        const std::vector<std::array<size_t, 2>>& edges,
        const std::vector<int64_t>& ids);
    /// The feature edges, smaller vertex id first.
    std::vector<std::array<size_t, 2>> feature_edges() const;
    bool is_feature_edge(size_t a, size_t b) const;
    /// The id set_feature_edges gave the feature edge (a, b): -1 for none, or for no feature edge.
    int64_t feature_edge_id(size_t a, size_t b) const;
    size_t feature_degree(size_t v) const;

    /**
     * @brief Vertices the remeshing keeps exactly: B-rep corners, say, which may sit inside a
     * curve between two regions as well as where curves meet.
     *
     * A fixed vertex stays put (see stays_put), and carries its id to the output. Call after
     * create_mesh().
     */
    void set_fixed_vertices(const std::vector<std::pair<size_t, int64_t>>& vertices);
    bool is_fixed(size_t v) const { return m_has_fixed && feature_attrs.at(v).fixed; }
    /**
     * @brief Whether `v` is never moved and never removed: a curve's end, a junction of curves, a
     * vertex between feature edges of different ids, or a fixed vertex.
     *
     * A collapse removes the vertex of its tuple and keeps the other; one that would remove such
     * a vertex is refused (see may_collapse_away).
     */
    bool stays_put(size_t v) const;
    /**
     * @brief Whether the curve rules let a collapse of `t` remove t.vid(): it does not stay put,
     * and if it is on a curve, `t` runs along it. The rest of collapse_edge_before still applies.
     *
     * With fixed vertices or edge ids, collapse_remeshing turns a collapse around when its
     * tuple's vertex may not go and the other end may: the queue holds an edge in one direction
     * only, and a fixed vertex is mostly the end that has to stay.
     */
    bool may_collapse_away(const Tuple& t) const;

    /**
     * @brief Per-face values the remeshing carries: per-face labels, say, whose boundaries are
     * feature edges. A split gives both halves of a face its values; a collapse or swap keeps
     * the values of the faces it changes, which is right where no swapped or collapsed edge
     * separates faces of different values -- where the boundaries are feature edges, as
     * set_feature_edges keeps them. `values` has one entry per face of the mesh given to
     * create_mesh(), in that order; call after create_mesh().
     */
    void set_face_fields(const std::vector<std::vector<double>>& values);
    bool has_face_fields() const { return m_has_face_fields; }

    /**
     * @brief Keep the curves near the feature edges as they are now -- the input's, called
     * before remeshing: every feature edge an operation leaves must lie within `eps` of them.
     *
     * The surface envelope does not hold a curve. Where its two sides meet at an angle, a curve
     * vertex smoothed across a bend of the curve, or a chord a collapse leaves across one, cuts
     * the corner along one of the sides and stays inside the surface envelope; where they meet
     * smoothly, a curve can wander across the surface. This tube judges the curves as
     * BoundaryEnvelope judges the boundary, with the surface envelope's predicate (exact or
     * sampled, as create_mesh() was given); a split, which moves nothing, is not judged. With
     * the tube, a collapse along a curve puts the survivor at the midpoint, as any other
     * collapse, unless it stays put (see stays_put): the tube refuses it where that leaves the
     * curve. Off until called; eps <= 0 turns it off.
     */
    void set_feature_envelope(double eps);
    /// TriMesh::consolidate_mesh, plus the renumbering of the feature edges.
    void consolidate_mesh();
    /**
     * @brief Removes the triangles that hold two feature edges -- "ears".
     *
     * Three consecutive curve vertices a, b, c that span a triangle leave b that triangle alone
     * on one side, and smoothing b along the curve would flatten it. Swapping the edge (a, c)
     * gives b two triangles there; where that is not possible -- (a, c) is a curve or the
     * boundary, or the swap would make a new ear or turn a triangle over -- b collapses along
     * the curve and the corner goes. Where it cannot either -- the moved b would turn a triangle
     * over -- (a, c) is split at its midpoint, which gives b two triangles there and moves
     * nothing; at a junction, which smoothing does not move, the corner stays. Where a curve
     * zigzags, removing one ear can make the next, so call it until it removes none. No other
     * swap or collapse makes an ear, but the input can hold them. A triangle with curves (or the
     * boundary) all round is a region of its own, not an ear, and stays. Serial; returns the
     * number removed.
     */
    size_t remove_feature_ears();
    /// Whether `v` is on a curve and one triangle holds both of its curve edges.
    bool is_feature_ear_tip(const Tuple& v) const;

    /**
     * @brief Refuse a collapse, swap or smoothing that folds the surface over.
     *
     * The triangles an operation changes or removes have a normal, their area vectors summed;
     * the operation may leave no more triangles that do not face along it -- face against it, or
     * are flat -- than there were, nor along the normal of the triangles it leaves. Nor may it
     * leave more folds in the stars of the vertices it touches, each star against its own
     * normal: what it changes is in its neighbours' stars too. So it makes no fold, and may undo
     * one the input has. A triangle faces along a normal when the exact orient3d says so (see
     * faces_along in the .cpp). A split moves no vertex and is not checked. On by default.
     */
    void set_check_inversions(bool check) { m_check_inversions = check; }
    /**
     * @brief Swap an edge to raise the worse shape quality of its two triangles (see
     * compute_swap_quality), instead of to bring the valences of its four vertices to 6 (4 on
     * the boundary). Off by default.
     *
     * Valence leaves a triangle as it is when the swap would not change the valences much: a
     * flat one between two long edges, or a needle a curve leaves, which only the quality
     * notices. The remeshing app of the topology-first B-rep pipeline swaps by quality.
     */
    void set_swap_quality(bool quality) { m_swap_quality = quality; }
    /**
     * @brief Refuse a collapse that leaves an edge longer than 4/3 of the target length -- the
     * split threshold -- as Botsch and Kobbelt's remeshing does. Off by default: a collapse is
     * then judged by the edge it removes alone, and those of the last iteration leave edges
     * that no split comes after to cut. cleanup_triangles() holds its collapses to the same
     * bound, on or off (see there).
     */
    void set_limit_collapse_length(bool limit) { m_limit_collapse_length = limit; }
    /**
     * @brief Repair, at the end of every iteration, the triangles whose smallest angle is under
     * `degrees`; 0, the default, repairs none. Call before create_mesh(), which then keeps the
     * input's area vectors for the fold test below.
     *
     * See cleanup_triangles(). The passes leave such triangles where their criteria do not see
     * them: next to a curve, where a vertex may not leave it and no swap or collapse may make an
     * ear, a triangle between a curve vertex and one off the curves can stay thin; and in a fold
     * the input brings, which the splits copy into the remeshing and the fold check then holds,
     * every operation on it can count as folding.
     */
    void set_cleanup_angle(double degrees) { m_cleanup_angle = degrees; }
    /**
     * @brief Repairs the triangles whose smallest angle is under `degrees`, worst first.
     *
     * Each gets the first of these that is allowed: a collapse of its shortest edge, either way;
     * a swap of one of its edges, longest first; a move of one of its vertices, or else of one of
     * the triangles around it -- one off the curves within its tangent plane, one inside a curve
     * along it -- to where the smallest angle of its star is largest. Allowed means the curve
     * rules, the envelopes and the fixed vertices hold, as everywhere; the operation folds nothing
     * by the rule of the fold check (see set_check_inversions; on or off, the repairs keep it);
     * a collapse leaves no edge longer than 4/3 of the target of the last collapse_remeshing(),
     * with set_limit_collapse_length() or without -- the repairs mend angles and are not to
     * coarsen the mesh, and those after the last iteration have no split after them; and it
     * raises the smallest angle of the triangles it leaves over that of the triangles around
     * the one repaired (of its star, for a move). Where what it replaces folds already, the
     * fold check, which judges against those triangles, can count a repair of the fold as one
     * more: there an operation that leaves fewer triangles facing against the input next to them
     * -- the input triangles within half their size plus the envelope, of the same face fields
     * where set_face_fields() gave some -- than there were is allowed too, even if no angle
     * improves. Only there: where two sheets of the input are closer than that, a thin wall, the
     * far one cancels the near one, and the input's orientation is noise. Rounds until one repairs
     * nothing, 10 at most. Serial; returns the number of repairs. Throws unless set_cleanup_angle()
     * came before create_mesh().
     */
    size_t cleanup_triangles(double degrees);
    /// The operations set_check_inversions() refused, since the mesh was made.
    size_t refused_inversions() const { return m_refused_inversions; }

    int retry_limit = 10;
    IsotropicRemeshing(
        std::vector<Eigen::Vector3d> _m_vertex_positions,
        int num_threads = 1,
        bool use_exact = true);

    ~IsotropicRemeshing() {}

    /**
     * @param m_freeze     freeze frozen_verts and, unless boundary_eps is positive, every
     *                     vertex of the open boundary
     * @param eps          surface envelope thickness; 0 = no envelope
     * @param boundary_eps how the open boundary is held.
     *   0 (default): m_freeze decides -- frozen, or held only by the surface envelope, which is
     *     a containment test and does not notice an outline retracting along the surface.
     *   > 0: no boundary vertex is frozen; instead every boundary edge an operation leaves that
     *     descends from the input's boundary (touches an input_boundary vertex) must lie within
     *     boundary_eps of the boundary edges of this input -- the tube ShortestEdgeCollapse
     *     uses. It judges all four operations: a smoothed boundary vertex has to keep its
     *     boundary edges in the tube, a collapse between a boundary vertex and an interior one
     *     keeps the boundary vertex in place (the midpoint would pull the outline into the
     *     surface for the tube to refuse), and the vertex splitting a boundary edge inherits
     *     the lineage. Holes and slits narrower than the tube can close when the link
     *     condition is off. Boundary an operation opens elsewhere is not judged.
     */
    void create_mesh(
        size_t n_vertices,
        const std::vector<std::array<size_t, 3>>& tris,
        const std::vector<size_t>& frozen_verts = std::vector<size_t>(),
        bool m_freeze = true,
        double eps = 0,
        double boundary_eps = 0);

    struct PositionInfoCache
    {
        Eigen::Vector3d v1p;
        Eigen::Vector3d v2p;
        size_t partition_id;
        // Collapse: of t.vid() (v1) and of the other endpoint (v2). All false without a
        // boundary envelope, where no vertex is flagged.
        bool v1_input_boundary = false;
        bool v2_input_boundary = false;
        // An input_boundary vertex that is still on the boundary.
        bool v1_on_boundary = false;
        bool v2_on_boundary = false;
        // Split: the new vertex lands on a boundary edge that touches an input_boundary vertex.
        bool split_input_boundary = false;
        // Feature curves: the edge's endpoints (t.vid() first), whether it is a feature edge and
        // its id, and whether a collapse leaves the survivor where it is.
        size_t v1 = 0, v2 = 0;
        bool feature_edge = false;
        int64_t feature_id = -1;
        bool keep_v2_position = false;
        // Collapse and swap, checking inversions: what the operation replaces.
        ReplacedSurface replaced;
        // A split is under way: its invariants skip the feature envelope.
        bool splitting = false;
        // Collapse and swap, in cleanup_triangles(): where what the operation replaces folds
        // already, its triangles that face against the input; 0 elsewhere, where it is not read.
        size_t input_folds = 0;
    };
    wmtk::threading::enumerable_thread_specific<PositionInfoCache> position_cache;

    void cache_edge_positions(const Tuple& t);

    bool invariants(const std::vector<Tuple>& new_tris) override;

    // TODO: this should not be here
    void partition_mesh();

    // TODO: morton should not be here, but inside wmtk
    void partition_mesh_morton();

    size_t get_partition_id(const Tuple& loc) const
    {
        return vertex_attrs[loc.vid(*this)].partition_id;
    }

    bool smooth_all_vertices();

    Eigen::Vector3d smooth(const Tuple& t);


    Eigen::Vector3d tangential_smooth(const Tuple& t);

    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& tris) const;
    std::vector<TriMesh::Tuple> new_edges_after_swap(const TriMesh::Tuple& t) const;
    std::vector<TriMesh::Tuple> replace_edges_after_split(
        const std::vector<TriMesh::Tuple>& tris,
        const size_t vid_threshold) const;
    std::vector<TriMesh::Tuple> new_sub_edges_after_split(
        const std::vector<TriMesh::Tuple>& tris) const;


    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;

    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    double compute_edge_cost_collapse(const TriMesh::Tuple& t, double L) const;
    double compute_edge_cost_split(const TriMesh::Tuple& t, double L) const;
    double compute_vertex_valence(const TriMesh::Tuple& t) const;
    /// 1 / the worse shape quality (4 sqrt(3) area / sum of squared edge lengths) of the two
    /// triangles at `t`, less the same after swapping it: positive where the swap helps the
    /// worse one. -1 where `t` has not two faces.
    double compute_swap_quality(const TriMesh::Tuple& t) const;
    /**
     * @brief Report statistics.
     *
     * Returns a vector with:
     * average_length
     * max length
     * min length
     * average valence
     * max valence
     * min valence
     */
    std::vector<double> average_len_valen();
    bool split_remeshing(double L);
    bool collapse_remeshing(double L);
    bool swap_remeshing();
    bool uniform_remeshing(double L, int interations);
    bool write_triangle_mesh(std::string path);

private:
    wmtk::AttributeContainerGroup m_vertex_group;
    bool m_has_features = false;
    bool m_has_fixed = false;
    /// Some feature edge has an id other than -1.
    bool m_has_edge_ids = false;
    bool m_has_face_fields = false;
    bool m_has_feature_envelope = false;
    /// Set while remove_feature_ears() runs (serially): its collapses may make an ear.
    bool m_removing_ears = false;
    bool m_check_inversions = true;
    bool m_swap_quality = false;
    bool m_limit_collapse_length = false;
    double m_cleanup_angle = 0;
    /// While cleanup_triangles() runs: the smallest angle around the triangle it repairs, which
    /// a collapse or swap has to beat. 0 otherwise.
    double m_cleanup_floor = 0;
    bool m_cleaning = false;
    /// Where cleanup_triangles() moves a vertex, in place of the smoothing's position.
    std::optional<Eigen::Vector3d> m_smooth_target;
    /// With a cleanup angle: the envelope's eps, and the input's area vectors and face fields.
    double m_envelope_eps = 0;
    std::vector<Eigen::Vector3d> m_input_area;
    std::vector<std::vector<double>> m_input_values;
    /// The target length of the last collapse_remeshing(), for the limit above.
    double m_collapse_target = 0;
    std::atomic<size_t> m_refused_inversions = 0;

    /// The area vectors of the triangles, summed (twice over).
    Eigen::Vector3d area_normal(const std::vector<Tuple>& tris) const;
    /// How many of the triangles do not face along n.
    size_t not_facing(const std::vector<Tuple>& tris, const Eigen::Vector3d& n) const;
    /// Over the vertices of the triangles, the triangles of each one's star that do not face along
    /// that star's area_normal. An operation may read these stars: one that changes a star holds
    /// its centre, which is in the one-ring of what this operation changes, and so held by it.
    size_t star_folds(const std::vector<Tuple>& tris) const;
    /// The triangles an operation is about to replace.
    ReplacedSurface replaced(const std::vector<Tuple>& tris) const;
    /// Whether the triangles an operation left fold the surface over more than the `before` they
    /// replaced.
    bool more_folds(const std::vector<Tuple>& tris, const ReplacedSurface& before) const;
    /// The same; counts a refusal.
    bool folds(const std::vector<Tuple>& tris, const ReplacedSurface& before);

    /// The smallest angle of the triangles, in degrees.
    double min_angle(const std::vector<Tuple>& tris) const;
    /// The area vectors of the input triangles next to `f`, summed: see cleanup_triangles().
    Eigen::Vector3d input_normal(const Tuple& f) const;
    /// How many of the triangles do not face along input_normal().
    size_t input_folds(const std::vector<Tuple>& tris) const;
    /// Whether the triangles a repair left fold no more than the `before` they replaced and beat
    /// `floor`; or, where `before` folds already, face against the input less often than its
    /// `input_folds_before`. See cleanup_triangles().
    bool cleanup_allows(
        const std::vector<Tuple>& tris,
        const ReplacedSurface& before,
        size_t input_folds_before,
        double floor) const;
    /// Moves `v` to where the smallest angle of its star is largest, above `floor`.
    bool cleanup_move(size_t v, double floor);
};

} // namespace wmtk::components::isotropic_remeshing
