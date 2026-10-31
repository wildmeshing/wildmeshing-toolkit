#pragma once

#include <wmtk/OptimizerParameters.h>
#include <wmtk/RationalPositions.h>
#include <wmtk/SurfaceTagAttributes.h>
#include <wmtk/TetMesh.h>
#include <wmtk/ExactPosition.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/history/VolumeHistory.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/utils/SurfaceTopology.hpp>

#include <igl/Timer.h>

#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

namespace wmtk {
namespace utils {
struct EmbedFeaturesResult;
}

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

        /**
         * @name Exact position
         *
         * Stored only while it differs from m_posf; see ExactPosition. Read it with pos() (or
         * pos_coord_equals() for one coordinate); set it with set_pos() when it is a genuinely
         * exact value, or set_pos_to_posf() when it is m_posf -- a rounded vertex, or one
         * placed at a double position.
         * @{
         */
        Vector3r pos() const { return m_exact.value(m_posf); }
        bool pos_coord_equals(int k, const Rational& r) const
        {
            return m_exact.coord_equals(k, m_posf, r);
        }
        void set_pos(const Vector3r& p) { m_exact.store(p); }
        void set_pos_to_posf() { m_exact.clear(); }
        ExactPosition<Vector3r, Vector3d> m_exact;
        /** @} */

        VertexAttributes() {}
        VertexAttributes(const Vector3r& p);
    };

    using FaceAttributes = wmtk::SurfaceTagAttributes;

    using VertAttCol = AttributeCollection<VertexAttributes>;
    using FaceAttCol = AttributeCollection<FaceAttributes>;

    VertAttCol m_vertex_attribute;
    FaceAttCol m_face_attribute;

    /**
     * @brief The application set FaceAttributes::m_orientation at insertion, so the tracked
     * surface carries the input's orientation and the operations keep it.
     *
     * The operations maintain the orientation whether or not this is set (it is 0 everywhere on
     * a mesh that never set it, and stays 0); the flag says whether a consumer may trust it. See
     * tracked_surface_boundary() for the invariant it obeys. The orientation never changes
     * which operations run: it only feeds the tracked winding number.
     */
    bool m_tracks_orientation = false;

    /**
     * @brief Surface flips that joined two faces not oriented alike (a fold of the tracked
     * surface, or unequal multiplicities).
     *
     * The flip runs anyway -- orientation must not change the operations -- so the oriented
     * surface no longer has the input's boundary there, and the tracked winding number is not
     * an integer near the flipped faces. Nonzero only on a tracked surface that was folded or
     * inconsistently oriented to begin with.
     */
    std::atomic<size_t> m_inconsistent_orientation_flips = 0;

    /**
     * @brief The tracked surface as an oriented soup: each tracked face emitted |m_orientation|
     * times, in the orientation the input gave it. Faces whose sheets cancel are left out.
     * Vertex ids index m_vertex_attribute.
     */
    std::vector<std::array<size_t, 3>> oriented_tracked_faces() const;

    /**
     * @brief Mesh edges where the oriented tracked surface has a boundary.
     *
     * The tracked surface is an integer 2-chain, and every operation preserves its boundary --
     * except a surface flip of two faces not oriented alike, which is counted in
     * m_inconsistent_orientation_flips: a closed input stays a cycle (signed sum of incident
     * oriented tracked faces is 0 on every edge), and an open one keeps its boundary on the
     * input's open boundary. Returns the edges
     * with a nonzero sum, each with that sum, for the caller to compare with what the input
     * allows. One pass over every face, then a map over the tracked faces' edges.
     */
    std::vector<std::pair<std::array<size_t, 2>, int>> tracked_surface_boundary() const;

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
     * @brief What p_edge_attrs points at, once a collection is registered with it.
     *
     * The 3D counterpart of wmtk::TriOptimizerMesh::m_edge_attr_group. Two owners keep a
     * per-edge collection: the feature-edge tags (enable_feature_edge_tracking()) and
     * topological_offset, which labels each edge by which part of the construction produced
     * it. With one p_edge_attrs slot only one of them could register, so feature tracking had
     * to refuse to run in the offset; registered here, each is resized, moved, protected and
     * rolled back with the others.
     *
     * Unlike the 2D group, which always holds the shared m_edge_attribute, this one starts
     * EMPTY -- the 3D base keeps no edge attribute of its own -- so p_edge_attrs stays null
     * until add_edge_attributes() registers the first collection. TetMesh skips every per-edge
     * step on a null p_edge_attrs (the owner migration in remove_tets_by_ids, the moves in
     * consolidate_mesh, the protect/rollback forwarding); pointing it at an empty group would
     * pay for all of them in the tetwild and simwild runs that track no edges.
     *
     * Same caveat as the 2D group: this moves the storage and nothing more. The shared
     * operations maintain the VALUES of m_feature_edge_attribute only (the split and collapse
     * restores); anything else registered here is its registering application's to maintain.
     */
    AttributeContainerGroup m_edge_attr_group;
    /**
     * @brief True while a pass of the shared operations runs.
     *
     * Set, and restored on exit or throw, by the two drivers every such pass goes through:
     * run_pass() (utils/RunPass.cpp) and the colored smoothing sweep
     * (utils::smooth_classes_checked). No existing state says this: the attribute collections'
     * protect flag is per thread and per operation, so it is false between two operations of a
     * pass and on every thread not running one. Only the calls that register a collection read
     * it; see throw_if_in_operation_pass().
     */
    bool m_in_operation_pass = false;
    /**
     * Registering a collection resizes it, which reallocates the storage under the pass's
     * readers, and an operation already in flight has not protected it -- nothing fails at the
     * call, the corruption shows up later. Thrown, not asserted: release builds compile
     * asserts out.
     */
    void throw_if_in_operation_pass(const char* call) const
    {
        if (m_in_operation_pass) {
            log_and_throw_error(
                "{}() called while an operation pass runs; call it before or after the pass.",
                call);
        }
    }
    /**
     * @brief Register an edge collection with m_edge_attr_group and point p_edge_attrs at the
     * group.
     *
     * Valid before or after init(), but not while an operation pass runs (it throws then; see
     * throw_if_in_operation_pass()). Before init() the storage is empty and the resize here is
     * a no-op; init() sizes the collection with the rest. After init() the resize brings it to
     * what init() and every capacity growth size edge collections to -- one entry per tet edge
     * slot, 6 x tet_storage_capacity() -- so the registered collections keep the invariant
     * the operations index by (size >= every slot id they can hand out), and the new entries
     * start at their default values. From then on growth, the owner migration in
     * remove_tets_by_ids, consolidation and rollback move it with the others.
     */
    void add_edge_attributes(AbstractAttributeContainer* c)
    {
        throw_if_in_operation_pass("add_edge_attributes");
        m_edge_attr_group.add(c);
        p_edge_attrs = &m_edge_attr_group;
        c->resize(6 * tet_storage_capacity());
    }

    /**
     * @brief Per-edge curve tags: which kinds of input curve this tet edge tiles.
     *
     * One bit per KIND of curve, because each kind is held to its own tube (see
     * tagged_edge_envelope()), and one edge can tile two kinds at once:
     *  - feature: the input feature curves the application was given, held by
     *    m_feature_envelope;
     *  - order2: the input's order-2 edges the application holds (holds_order2_edge()), held
     *    by m_order2_edge_envelope; set by tag_order2_edges().
     * An edge with any bit set is TAGGED, and the operations treat every tagged edge alike --
     * propagated through split, collapse and swap, never removed by a swap, order-2 substructure
     * for the link condition. Only the tube it is held to, and vertex_has_feature_edge(), read
     * the kind.
     *
     * Within a kind the tag is a union: which curve an edge belongs to is not carried through
     * the optimization, and the kind's edges are held by one tube around all its curves.
     * Tracked faces are different: each carries its input set (SurfaceTagAttributes::input_set())
     * through every operation and is held to its own inputs' envelopes, because one envelope
     * around every input let a solid's boundary ride a sheet crossing it -- measured, 12 faces up
     * to 3.61 eps off the solid -- and an audit at the end can find that but not prevent it.
     * Curves are expected to follow the faces once two of them close together can be measured
     * drifting the same way.
     */
    struct FeatureEdgeAttributes
    {
        enum Kind : uint8_t {
            feature = 1 << 0,
            order2 = 1 << 1,
        };
        /// The Kind bits of every curve this edge tiles; 0 = untagged.
        uint8_t m_kinds = 0;

        bool is_tagged() const { return m_kinds != 0; }
        bool has(Kind k) const { return (m_kinds & k) != 0; }
        void add(Kind k) { m_kinds |= k; }
        void remove(Kind k) { m_kinds &= uint8_t(~k); }
    };
    using EdgeAttCol = AttributeCollection<FeatureEdgeAttributes>;
    /// Sized, protected and rolled back only once enable_feature_edge_tracking() ran.
    EdgeAttCol m_feature_edge_attribute;

    /**
     * @brief Whether the shared operations maintain m_feature_edge_attribute.
     *
     * The operations are guarded by THIS flag, never by `p_edge_attrs != nullptr`:
     * topological_offset also registers an edge collection with m_edge_attr_group, with its
     * own attribute type, so a non-null p_edge_attrs says nothing about feature tags.
     */
    bool m_track_feature_edges = false;

    /**
     * @brief Envelope around the input feature curves: the tube an edge tagged `feature` must
     * stay in.
     *
     * The collapse guard's veto (see collapse_edge_before). Kept SEPARATE from the surface
     * envelope and the order-2 envelope: merging would let feature curves vouch for open
     * boundaries (and vice versa) in guard decisions. Null => feature edges are propagated
     * but not geometrically constrained.
     */
    std::shared_ptr<Envelope> m_feature_envelope;
    /**
     * @brief Envelope around the input's order-2 edges: the tube an edge tagged `order2` must
     * stay in.
     *
     * The order2 counterpart of m_feature_envelope, separate from it for the reason given there.
     * Null => order2 edges are propagated but not geometrically constrained. Built by
     * tag_order2_edges().
     */
    std::shared_ptr<Envelope> m_order2_edge_envelope;
    /// The tube edges of kind `k` are held to (one of the two above); null if there is none.
    const Envelope* tagged_edge_envelope(FeatureEdgeAttributes::Kind k) const
    {
        return (k == FeatureEdgeAttributes::feature ? m_feature_envelope : m_order2_edge_envelope)
            .get();
    }
    /// The first of `kinds`, in bit order, whose tube the segment (a, b) leaves; 0 when it stays
    /// inside every one of them. A kind without a tube constrains nothing.
    uint8_t tagged_edge_leaves_tube(
        uint8_t kinds,
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b) const;

    /**
     * @brief Turn on feature-edge tracking. The collection joins m_edge_attr_group beside
     * whatever the application registered there.
     *
     * Valid before or after init(), not during an operation pass (it throws then; see
     * add_edge_attributes(), which sizes it). After init() it starts with no edge tagged, and the
     * application tags the edges it needs. Late enabling is supported because an application may
     * learn only from the built mesh whether its input has feature curves -- topological_offset
     * reads that off the labels init() produces -- and enabling it for every input would make every
     * operation cache and restore feature data it does not have (+50% wall time on the offset's
     * triangle inputs). Idempotent: registering twice would move every tag twice.
     */
    void enable_feature_edge_tracking()
    {
        throw_if_in_operation_pass("enable_feature_edge_tracking");
        if (m_track_feature_edges) return;
        add_edge_attributes(&m_feature_edge_attribute);
        m_track_feature_edges = true;
    }

    /// Does any edge incident to `vid` carry the `feature` kind? Derived from the edge tags
    /// rather than from a vertex flag, so it cannot go stale. The feature kind only: tetwild's
    /// smoothing pull asks it to prefer the user's feature tube over the derived order-2 one.
    bool vertex_has_feature_edge(size_t vid) const;
    /// Does any edge incident to `vid` carry a tag, of any kind? Derived like the above.
    bool vertex_has_tagged_edge(size_t vid) const;
    /**
     * @brief The tube a vertex with a `feature` edge is pulled toward while smoothing,
     * m_feature_envelope; null for any other vertex or without the tube.
     *
     * Asked first by every application's smoothing_energy_envelope(): the feature curves are
     * user-supplied where order 2 is derived, so where a vertex is on both the explicit
     * constraint wins. A soft pull; the hard vetoes (the collapse guard, the tagged-edge
     * containment in smoothing) do not read it. Without it simwild pulled a feature vertex off
     * the surface toward the surface envelope, whose containment it is never held to.
     */
    std::shared_ptr<Envelope> feature_pull_envelope(size_t vid) const;
    /// Does any edge incident to `vid` carry kind `k`? False while edge tracking is off.
    bool vertex_has_edge_of_kind(size_t vid, FeatureEdgeAttributes::Kind k) const;

    /**
     * @brief Does the application hold an order-2 edge with `n_faces` tracked faces?
     *
     * The application's rule, asked at both ends of the tag's life: tag_order2_edges() tags the
     * edges it holds, and drop_stale_order2_tags() drops the tag of an edge a collapse leaves
     * with a count it does not hold. The default holds every order-2 edge, the open boundaries
     * (1 face) and the non-manifold edges (3 and more); simwild keeps it, its non-manifold edges
     * being where the pieces it was given meet. tetwild holds its open boundaries only, and its
     * override says why.
     */
    virtual bool holds_order2_edge(const size_t n_faces) const
    {
        return n_faces == 1 || n_faces >= 3;
    }

    /**
     * @brief Tag the input's order-2 edges the application holds (holds_order2_edge()) with the
     * order2 kind and build their tube, m_order2_edge_envelope, of half-width `eps`. Returns the
     * number of edges tagged.
     *
     * An order-2 edge has 1 tracked face (an open boundary) or 3 and more (a non-manifold
     * edge): it is where a sheet ends. The one-sided surface envelope cannot see a sheet
     * shortening there -- a rim sliding inward keeps every face on its sheet -- so these edges
     * need a tube of their own, and they need it whether or not topology is preserved: keeping
     * them in place is a geometric guarantee, separate from the topological one. Measured on
     * simwild, whose smoothing had no boundary check: an open square inside a cube lost 3 of its
     * 4 corners, its rim 7.4 tube radii off at the default eps and 14.8 at eps_rel 1e-3, the
     * same 0.038 at every eps. Tagged, the edges get every check the feature kind has: the
     * collapse guard, the smoothing containment, the swap refusal, the link condition's orders.
     *
     * Call once after init, with the tracked faces final. Membership is decided here, from the
     * tracked-face counts, and then carried: an edge the operations make order-2 is not the
     * input's and is not held, and one they make ordinary loses its tag (see
     * drop_stale_order2_tags()). Turns edge tracking on only when there is such an edge, since
     * tracking costs every operation its cache (see enable_feature_edge_tracking()).
     */
    size_t tag_order2_edges(double eps, bool exact);
    /**
     * @brief Build the feature-curve tube, m_feature_envelope, of half-width `eps` around the
     * input feature edges `E` over `V`, with the exact predicate when `exact`. Nothing without
     * edges.
     *
     * Around the ORIGINAL input curves, like every other envelope, not around the tet edges
     * tiling them (tag_feature_edges()): the tiling is exact, so the two are the same set at
     * init, and the input is what the guarantee is stated against.
     */
    void init_feature_envelope(
        const std::vector<Vector3d>& V,
        const std::vector<std::array<size_t, 2>>& E,
        double eps,
        bool exact);
    /**
     * @brief Tag every tet edge tiling an input feature edge (`placed`, from the insertion)
     * with the feature kind. Returns the number of tet edges tagged.
     *
     * The tiling pairs come out of the same arrangement as the connectivity, so a missing edge
     * is a contract violation, not an input problem: it throws. Call once after init, before
     * anything renumbers the vertices. Turns edge tracking on only when there is a feature
     * edge, as tag_order2_edges() does. The tiling is per input edge, so what an edge carries
     * beyond the kind can be looked up by its input edge.
     */
    size_t tag_feature_edges(const utils::EmbedFeaturesResult& placed);
    /**
     * @brief After a collapse: drop the order2 kind from each edge of `cache` (v_old renamed to
     * v_new) whose tracked-face count holds_order2_edge() no longer holds.
     *
     * The tag holds an edge while it is still order-2. Without preserve_topology a collapse may
     * close a slit, glue a sheet's rim onto another sheet or remove a lone triangle, and the
     * edge there is then an ordinary one; holding it in the tube only pins the surface. Measured
     * on 111388 (tetwild, no preserve_topology) with every order-2 edge tagged: after 4
     * iterations 49 of the 141 tagged edges had 2 faces again. With the open boundaries alone
     * tagged and no drop, it reached the stop energy in 22 iterations (4.997, stop 5), 16 with
     * it; test open-boundary-tags-drop-when-glued keeps 14 or 15 of 72 tags stale without it,
     * at every eps. Only a collapse changes a surviving edge's count: a split's halves have the
     * parent's faces, a face swap never removes a tracked face, and a surface flip replaces an
     * untagged two-face edge and keeps the four rim edges' counts. tetwild cleared its
     * open-boundary vertex flags the same way, after each collapse. The cache's edges have both
     * endpoints in the locked one-ring of the collapse, and every tet around such an edge contains
     * them, so no other thread can change the faces counted here.
     */
    void drop_stale_order2_tags(
        const std::map<std::array<size_t, 2>, uint8_t>& cache,
        size_t v_old,
        size_t v_new);
    /// The audit of that guarantee: how many edges carrying kind `k` lie outside its tube now.
    /// Zero after every run unless an operation lets a tagged edge out; 0 when tracking is off
    /// or the kind has no tube.
    size_t tagged_edges_outside(FeatureEdgeAttributes::Kind k) const;

    // Substructure widening for the link condition (preserve_topology): a tagged edge, of any
    // kind, is order-2 substructure, a vertex with two incident tagged edges is on a curve
    // (order 2), with one or three-plus it is a curve endpoint or junction (order 3). The
    // meaning of get_order_of_* is untouched.
    size_t substructure_order_of_edge(const std::array<size_t, 2>& vids) const override;
    size_t substructure_order_of_vertex(size_t vid) const override;
    void substructure_feature_neighbors(size_t vid, std::vector<size_t>& out) const override;
    /// Are all tagged edges at `vid` inside their kinds' tubes, at the CURRENT positions? Each
    /// kind an edge carries is tested in its own tube (tagged_edge_leaves_tube()). True when
    /// tracking is off or no kind has a tube. Smoothing calls this with the candidate position
    /// already written into the vertex attribute.
    bool feature_edges_at_vertex_inside(size_t vid) const;

    /**
     * @name Feature-point anchors
     *
     * The 3D twin of TriOptimizerMesh's block of the same name, which states the reasoning:
     * preserving a 0-dimensional feature -- an input point, or where a tagged curve ends or
     * three or more meet -- is a COVERAGE property, so each anchor vertex names WHICH feature
     * it stands for. Lifted from tetwild, itself a port of triwild, with the same semantics and
     * the same distance test.
     *
     * Opt-in: enable_feature_point_tracking() registers the per-vertex ids and has the shared
     * operations maintain and enforce them. A split vertex stands for no feature, and a collapse
     * survivor that stands for none takes over the collapsed vertex's; a collapse that would
     * leave a feature further than its radius from every vertex is refused
     * (CollapseReject::feature_point), and smoothing may move an anchor vertex only inside its
     * ball (the default smoothing_position_is_allowed()). No switch turns the holding off, as
     * none does for the curves and the surfaces; until 2026-10 tetwild and triwild had one,
     * preserve_feature_points, kept to reproduce the behaviour before the anchors.
     *
     * Which vertices are anchored is read off the tags, by anchor_feature_points(); the
     * application adds only its input points, which no tag marks.
     * @{
     */
    /// A vertex that stands for no feature point. See FeaturePointAttributes.
    static constexpr size_t NO_FEATURE_POINT = std::numeric_limits<size_t>::max();
    struct FeaturePointAttributes
    {
        /// Index into m_feature_points, or NO_FEATURE_POINT.
        size_t m_feature_point_id = NO_FEATURE_POINT;
    };
    using FeaturePointAttCol = AttributeCollection<FeaturePointAttributes>;
    /// Registered with m_vertex_attr_group only once enable_feature_point_tracking() ran.
    FeaturePointAttCol m_feature_point_attribute;
    /// Anchor positions, indexed by FeaturePointAttributes::m_feature_point_id. A vertex
    /// carrying feature f may never end up further than m_feature_point_radius[f] from
    /// m_feature_points[f]; the anchor never moves, so the id is a direct lookup.
    std::vector<Vector3d> m_feature_points;
    /// Radius of each anchor's ball, beside m_feature_points. Per anchor, because each kind of
    /// feature has its own radius (see anchor_feature_points()).
    std::vector<double> m_feature_point_radius;
    /// Whether the shared operations maintain and enforce the anchors
    /// (enable_feature_point_tracking()).
    bool m_track_feature_points = false;

    /**
     * @brief Turn on feature-point tracking.
     *
     * Valid before or after init(), not during an operation pass (it throws then; see
     * throw_if_in_operation_pass()). Before init() the storage is empty and the resize is a no-op;
     * init() sizes the ids with the rest of m_vertex_attr_group. After init() the resize brings
     * them to what init() and every capacity growth size vertex collections to,
     * vert_storage_capacity(), with every vertex NO_FEATURE_POINT; the application then anchors
     * what it needs. Late enabling is supported because an application may learn only from the
     * built mesh whether its input has feature points, and enabling it for every input would make
     * every operation cache and restore anchors it does not have. Idempotent.
     */
    void enable_feature_point_tracking()
    {
        throw_if_in_operation_pass("enable_feature_point_tracking");
        if (!m_track_feature_points) {
            m_vertex_attr_group.add(&m_feature_point_attribute);
            m_feature_point_attribute.resize(vert_storage_capacity());
        }
        m_track_feature_points = true;
    }

    /**
     * @brief Anchor vertex `vid` at its current position, in a ball of `radius`. A vertex
     * already anchored keeps its anchor, whose radius becomes the smaller of the two. Returns
     * whether a new anchor was made. Feature-point tracking must be on.
     */
    bool anchor_feature_point(size_t vid, double radius);

    /**
     * @brief Anchor the 0-dimensional features, after the edge tags are final: every vertex
     * with 1 or 3+ tagged edges (where a tagged curve ends, or three or more meet -- the vertices
     * the link condition counts as order 3), and every vertex of `input_point_vids`. Turns
     * feature-point tracking on when there is something to anchor, so a run without any pays no
     * per-operation cost. Returns the number of anchors made.
     *
     * Each anchor's radius is the smallest of the radii of the kinds it is on: `order2_radius`
     * if it has an order2 edge, `feature_radius` if it has a feature edge or is an input point.
     * The smallest, because ball(r) around a point of a curve held in a tube of r lies in that
     * tube, and a point is then held as tightly as every curve through it.
     *
     * At init only, as the tags. A collapse can make a curve end later, but only by gluing: a rim
     * edge stops having one face (its tag drops, drop_stale_order2_tags()), or two tagged edges
     * merge. Such an end sits where the rim meets the glued seam, and collapsing it along the
     * rim moves the other sheet's faces onto the next rim vertex, which must stay within that
     * sheet's envelope: it glues the rim one edge further where the two are already within eps,
     * the gluing preserve_topology off allows, and loses no coverage. Anchored, measured on the
     * tetwild integration configs (2026-10): the rims glued away all the same (366725: 2 and 3
     * of 181 rim edges left), at 5 times the iterations on 366725 and 15-33% more tets on crown
     * and 111388.
     */
    size_t anchor_feature_points(
        const std::vector<size_t>& input_point_vids,
        double order2_radius,
        double feature_radius);

    /**
     * True iff collapsing v1 into v2 would drop or displace a feature-point anchor. False
     * when tracking is off. Distance test included, as in 2D: two anchors within each
     * other's radius are allowed to merge -- the survivor covers both -- and forbidding that
     * deadlocks the mesh (measured in 2D, see TriOptimizerMesh).
     */
    bool collapse_breaks_feature_point(const size_t v1_id, const size_t v2_id) const;

    /// May vertex `vid` sit at `p` as far as its anchor goes? True unless the vertex stands
    /// for a feature and `p` is outside that feature's ball.
    bool feature_point_ball_contains(const size_t vid, const Vector3d& p) const;

    /// {anchors still represented within their radius, total}. Geometric, as in 2D: "some
    /// vertex is within the radius of this anchor", whether or not it carries the id.
    std::pair<size_t, size_t> feature_retention(double* worst_ratio = nullptr) const;
    /** @} */

    /// Per-vertex positional constraint, on top of the envelopes -- the 3D counterpart of
    /// TriOptimizerMesh's hook of the same name. The default is the feature-point ball, which
    /// answers true when the anchors are off; an application overriding this while anchoring
    /// feature points must AND its own answer with feature_point_ball_contains().
    virtual bool smoothing_position_is_allowed(const size_t vid, const Vector3d& p) const
    {
        return feature_point_ball_contains(vid, p);
    }

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
    std::shared_ptr<Envelope> m_envelope;

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
    /// How the solves smooth_after() makes ended -- iterations and stop status -- reported and
    /// reset once per pass, beside m_smooth_rejects. An application that solves on a path of its
    /// own keeps its own counters and reports them from log_smoothing_pass_accounting().
    optimization::NewtonCounters m_newton;

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

    /// Every pass grows the storage at its serial points -- between operations when serial,
    /// between rounds when parallel (see ExecutePass) -- so the optimizer only keeps a small
    /// slack instead of the preallocation factor.
    bool storage_grows_on_demand() const override { return true; }

protected:
    /// The high-valence claims are indexed by vertex id and sized to the storage at the start
    /// of a split pass; a split pass that grows the storage has to grow them with it, or the
    /// vertices it creates would fall outside the gate. Existing claims are kept.
    void on_slot_storage_grown() override
    {
        if (!m_high_valence_claim) return;
        const size_t n = std::max(vert_capacity(), m_vertex_attribute.size());
        if (n <= m_high_valence_claim_size) return;
        auto grown = std::make_unique<std::atomic<int>[]>(n);
        for (size_t i = 0; i < m_high_valence_claim_size; ++i) {
            grown[i].store(m_high_valence_claim[i].load(std::memory_order_relaxed));
        }
        m_high_valence_claim = std::move(grown);
        m_high_valence_claim_size = n;
    }

public:
    /// split_all_edges's gate: length^2 >= splitting_l2 * s^2, s the mean of the endpoints'
    /// sizing scalars, or a force-split edge.
    bool split_edge_is_due(const Tuple& e) const;
    /// The current split pass's ExecutePass::waits() and wait_defects().
    size_t m_split_order_waits = 0;
    size_t m_split_order_wait_defects = 0;

    explicit TetOptimizerMesh(OptimizerParameters& params, std::shared_ptr<Envelope> env)
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

    /**
     * @brief Update the attributes of the mesh after an iteration of operations.
     *
     * This is necessary in SimWild to update the surface flags of faces and vertices as operations
     * might have collapsed away regions that are now no longer on the surface. In TetWild, this is
     * a no-op.
     */
    virtual void update_attributes() {}
    /// update_attributes(), then the change it made to the tracked surface, if recording one.
    void update_attributes_and_history();

    /// Start recording the tracked surface (TetMesh::start_surface_history), at m_posf, with
    /// `metadata` (the application's parameters, say) saved along. Takes m_insertion_sources.
    void record_surface_history(const std::string& metadata);
    /**
     * @brief Bring the recorded surface up to date with the tracked faces, stop recording, log
     * what the history holds and return it, for the application to save (ComponentIO::
     * save_history). None if not recording.
     *
     * With `input`, the history of the input the mesh was made from (read, welded, simplified):
     * the history returned is that, then the insertion -- each tracked face on the input faces
     * it lies on, from m_insertion_sources -- then the recorded one, so it maps the input files
     * to the output.
     */
    std::optional<history::SurfaceHistory> finish_surface_history(
        const history::SurfaceHistory* input = nullptr);

    /// Start recording the volume around the tracked surface too (TetMesh::start_volume_history),
    /// at m_posf. The surface history must be recording.
    void record_volume_history();
    /// Stop recording the volume, and return it as the bytes of its file
    /// (history::VolumeRecorder::to_bytes; io::ComponentIO::save_volume_history saves them), its
    /// edits indexed as in the surface history finish_surface_history() returned, with the names
    /// of the tags (tet_tags()) the mesh files give them. Nothing if not recording.
    std::optional<std::vector<uint8_t>> finish_volume_history(
        const std::map<int64_t, std::string>& tag_names = {});

    /// A tracked face by its corners' positions, sorted.
    using PositionKey = std::array<std::array<double, 3>, 3>;
    static PositionKey position_key(const Vector3d& a, const Vector3d& b, const Vector3d& c);
    /// Where the tracked faces lie on the input the mesh was made from: by position key, sorted,
    /// the input triangles. Filled at insertion when a surface history is to start from the
    /// input; record_surface_history takes it.
    std::vector<std::pair<PositionKey, std::vector<uint32_t>>> m_insertion_sources;
    /// The volume being recorded, kept past stop_surface_history() until finished; the offset of
    /// the recorded surface edits in the surface history finish_surface_history() returned, and
    /// that history's key, which the volume history keeps.
    std::shared_ptr<history::VolumeRecorder> m_recorded_volume;
    uint64_t m_surface_edit_offset = 0;
    std::optional<history::SurfaceKey> m_finished_surface;

    /// The faces with m_is_surface_fs, for a surface history (TetMesh::start_surface_history).
    bool is_tracked_face(size_t tid, int j) const override;
    /// The input's orientation on the face (m_orientation), where insertion recorded one;
    /// ascending vertex order where it did not.
    using TetMesh::orient_tracked_face;
    std::array<size_t, 3> orient_tracked_face(
        const std::array<size_t, 3>& f,
        const std::vector<size_t>& tets) const override;
    /// The edges tagged `feature` (m_feature_edge_attribute) that are edges of tracked faces.
    std::vector<std::array<size_t, 2>> tracked_feature_edges() const override;

    /// m_params.check_surface_folds; see there.
    bool checks_tracked_surface() const override { return m_params.check_surface_folds; }
    void tracked_surface_before(const std::vector<size_t>& vs, bool all) override;
    bool tracked_surface_after(const std::vector<size_t>& vs, bool all) override;
    /// The operations check_surface_folds refused.
    std::atomic<size_t> m_surface_fold_refusals = 0;

    /// Shared TetWild/SimWild outer optimization schedule.
    int m_iterations_used = 0;
    std::shared_ptr<history::TrackedSurfaceRecorder> m_recorded_surface;
    /// What the tracked faces around an operation were before it, for check_surface_folds: their
    /// area vectors summed, how many do not face along that, and the folds in the stars of their
    /// vertices. `skip` where no vertex around it is on the surface.
    struct SurfaceFolds
    {
        bool skip = true;
        Eigen::Vector3d normal = Eigen::Vector3d::Zero();
        size_t against = 0;
        size_t star_folds = 0;
    };
    wmtk::threading::enumerable_thread_specific<SurfaceFolds> m_surface_folds_before;
    /// The tracked faces, oriented, their area vectors summed; and, over their vertices, the
    /// faces of each one's tracked star not facing along that star's sum.
    Eigen::Vector3d tracked_area_normal(const std::vector<std::array<size_t, 3>>& faces) const;
    size_t tracked_not_facing(
        const std::vector<std::array<size_t, 3>>& faces,
        const Eigen::Vector3d& n) const;
    size_t tracked_star_folds(const std::vector<std::array<size_t, 3>>& faces) const;
    /// The input triangles each face of the recorded surface's first state lies on.
    std::vector<std::vector<uint64_t>> m_recorded_sources;
    int m_debug_print_counter = 0;
    /// WHICH PASS THE NEXT DEBUG FRAME BELONGS TO, e.g. "split", "smooth", "collapse-skipped"
    /// (a group whose op count was 0 still writes its checkpoint frame). Set by every writer
    /// before it writes, so an application that renames frames (the 3D offset) can carry the
    /// pass name into the file name; the base's own "debug_<N>" names are unchanged. The
    /// TriOptimizerMesh twin.
    std::string m_debug_pass_name;
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
     * edges by the time this is asked. An open boundary of an input surface -- tetwild's, or a
     * simwild surface input's -- carries the order2 tag, and a tagged edge is never swapped
     * (swap_edge_before()), so it needs no answer here. topological_offset overrides it.
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
     * @brief Envelope the tracked-surface triangle `vids` must stay inside, given the
     * attributes `attr` it carries -- or, for a triangle an operation is creating, the ones the
     * operation is about to write there.
     *
     * Null means this triangle carries no containment requirement, and every containment check
     * on it is skipped. The default answers with the one surface envelope; tetwild answers with
     * the envelopes of the face's input set (SurfaceTagAttributes::input_set()), and an
     * application tracking more than one surface (see SurfaceTagAttributes::m_surface_class)
     * answers per class -- the offset boundary, for instance, is free to move and is checked
     * against nothing.
     *
     * Keyed on the vertex ids rather than the fid because every caller is a topological
     * operation asking about the triangles it is ABOUT to create or has just created, where
     * only the vids are known for certain. The attributes come from the caller for the same
     * reason: a new triangle's own slot is written only after the check -- a collapse checks
     * its faces before it writes their attributes back, a split before it writes its halves --
     * and until then may hold a recycled face's. Every operation knows the face it is deriving
     * the new one from, and passes what it will write.
     */
    virtual std::shared_ptr<Envelope> surface_envelope_for_face(
        const std::array<size_t, 3>& vids,
        const FaceAttributes& attr) const
    {
        return m_envelope;
    }

    /// Whether the tracked-surface triangle (a,b,c) with attributes `attr` (see
    /// surface_envelope_for_face()) is outside its containment envelope. False when it has none.
    /// Every surface containment check in the operations goes through here.
    bool surface_triangle_is_outside(
        const size_t a,
        const size_t b,
        const size_t c,
        const FaceAttributes& attr) const
    {
        const std::shared_ptr<Envelope> env = surface_envelope_for_face({{a, b, c}}, attr);
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
     * @brief Envelope the smoother checks tracked face `face` (attributes `attr`) at the moved
     * vertex `vid` against; null skips the face.
     *
     * By default the face's own containment, surface_envelope_for_face() -- the one every
     * operation's check takes, so a move and an operation hold a face to the same set. It used
     * to be ONE envelope per vertex for all its faces, which cannot be right once faces at one
     * vertex belong to different inputs: a vertex where the solid's boundary meets a sheet
     * crossing it would hold the solid's faces to the sheet's envelope too, or the reverse.
     * topological_offset keeps its per-vertex answer by overriding this.
     *
     * The pull (smoothing_energy_envelope()) stays one real envelope per vertex: the pull calls
     * the non-virtual Envelope queries, which a composite cannot answer.
     */
    virtual std::shared_ptr<Envelope> smoothing_containment_envelope(
        const size_t /*vid*/,
        const std::array<size_t, 3>& face,
        const FaceAttributes& attr) const
    {
        return surface_envelope_for_face(face, attr);
    }
    /**
     * @brief Envelope a surface vertex is pulled toward while smoothing. A real envelope, never a
     * composite (see smoothing_containment_envelope()). A derived mesh may select an order-2 or
     * feature envelope here.
     */
    virtual std::shared_ptr<Envelope> smoothing_energy_envelope(const size_t vid) const = 0;

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

    /**
     * @brief The number the shared smoother compares for one cell, given the cell's quality
     * (AMIPS^3) at the position being tested. Its quality veto compares the max of this over the
     * ring before and after the move, and the default smoothing_ring_energy() is the same max.
     *
     * The quality itself by default, so TetWild and SimWild compare exactly what they compared
     * before. An application whose smoother minimises another per-cell energy (through
     * smoothing_extra_energy()) overrides it, so that the smoother's tests judge a move by that
     * same energy. The 2D twin is TriOptimizerMesh::smoothing_cell_energy.
     */
    virtual double smoothing_cell_energy(const size_t /*tid*/, const double quality) const
    {
        return quality;
    }

    /**
     * @brief The number the shared smoother's projected step compares for a whole one-ring: a
     * vertex held on a surface takes the first projected candidate whose value is strictly below
     * the value before the move.
     *
     * `stored` true: the ring before the move, read from the stored cell qualities; it is asked
     * before the solve moves the vertex, so the mesh as it is is that ring too. `stored` false: the
     * ring at the candidate position, every cell of it already tested for inversion.
     *
     * By default the max over the ring of smoothing_cell_energy() -- what the projected step
     * compared before this hook existed -- so TetWild and SimWild are unchanged. An application
     * whose rules compare a sum of a per-cell energy over the cells a move changes overrides it
     * with that sum, so that the projected step picks its candidate by the same rule. 3D only.
     */
    virtual double smoothing_ring_energy(const std::vector<Tuple>& ring, const bool stored) const
    {
        double m = 0.;
        for (const Tuple& loc : ring) {
            const size_t tid = loc.tid(*this);
            m = std::max(
                m,
                smoothing_cell_energy(tid, stored ? cell_quality(tid) : get_quality(loc)));
        }
        return m;
    }

    /**
     * @brief The smoothing hooks. A derived class overriding them -- or invariants(), which a
     * smooth also runs -- must read every attribute outside the smoothed vertex's star (other
     * vertices, and the edges, faces and tets not incident to the vertex) through const access,
     * e.g. `std::as_const(m_vertex_attribute)[u]`.
     *
     * Parallel smoothing runs non-adjacent vertices concurrently without locks (see
     * OptimizerParameters::colored_smoothing). Two of them can share a neighbour, and a
     * non-const access inside the smooth records the entry, which a rejected smooth writes
     * back: two threads would write the same neighbour at once. Every smooth of that pass checks
     * this (TetMesh/TriMesh::m_check_smoothing_stays_in_star) and throws on the first offence.
     */
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;
    void smooth_all_vertices(const size_t n_iters = 1);
    /// Whether this pass smooths by color class (see OptimizerParameters::colored_smoothing).
    bool use_colored_smoothing() const;
    /// One smoothing sweep over `ops` (all "vertex_smooth"), one color class at a time.
    void smooth_vertices_colored(const std::vector<std::pair<std::string, Tuple>>& ops);
    /// Called by smooth_all_vertices() after each pass's own accounting lines: an application
    /// that solves some vertices on a path of its own logs, and then resets, its counters here.
    virtual void log_smoothing_pass_accounting() {}
    /// Called by smooth_all_vertices() at the end of EVERY smoothing pass, after its accounting
    /// (log_smoothing_pass_accounting()) and before its debug frame: an application that keeps
    /// state derived from the positions the pass just moved refreshes it here. Not called before
    /// the first pass.
    virtual void smoothing_pass_done() {}

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
     * mesh_improvement() begins with local_operations({{0,1,1,0}}, false) -- a collapse pass
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
     * This governs the two BARE coarsening passes -- the opening one and the closing one -- and
     * not coarsen_mesh(), which mesh_improvement() runs without this gate, nor the collapse inside
     * the loop, which is interleaved with splits and smoothing and answers to the criterion.
     * Measured on the same model with only the closing pass left enabled: 1172 faces to 350, so
     * gating the opening one alone is not enough.
     */
    virtual bool optimization_bare_coarsen_passes() const { return true; }
    /// Called at the end of each iteration of mesh_improvement(), after its operation groups and
    /// before the stop test (`it` from 0). No-op by default.
    virtual void optimization_iteration_done(int /*it*/) {}
    virtual void write_optimization_debug_output(const std::string& path) = 0;
    virtual void optimization_sanity_checks_extra() {}

    /// CHURN INSTRUMENTATION. Bumped at the start of every split pass in local_operations(), so
    /// the pass a vertex was born in can be compared against the pass a collapse removes it in.
    /// Zero until the first split pass runs, which is what lets a construction-era vertex be
    /// told apart from an optimization-split one. Never reset -- only differences mean anything.
    uint32_t m_op_epoch = 0;
    virtual bool optimization_stop_at_float() const { return false; }

    /**
     * @brief Whether an iteration that moved the metric from @p prev to @p cur is stalled, and
     * the sizing refinement should therefore fire.
     *
     * Exactly the expression mesh_improvement() used inline: the improvement is small next to
     * the distance the metric still has to cover, i.e. the mesh is not on course to reach the
     * target within about 1/stall_eps more iterations.
     *
     * It is a virtual because the scalar the driver has is a MAX over criteria, and for an
     * application with more than one criterion that max only ever reports on whichever is
     * currently worst -- so a run whose worst criterion is stuck refines every iteration even
     * while the others are improving. An application that knows its criteria apart overrides
     * this to ask the question of each of them.
     */
    virtual bool optimization_stalled(double prev, double cur)
    {
        return (prev - cur) <= m_params.stuck_refine_stall_eps * (cur - optimization_stop_metric());
    }

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
    /// Called once the feature-edge tags are restored after a collapse of v1 into v2, only
    /// when feature edges are tracked: the survivor's feature state is decided here.
    virtual void collapse_after_features(size_t, size_t) {}

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

    // ---- SWAP REJECTION INSTRUMENTATION (debug; see swap_reject_report()) ----
    // Every place a proposed swap can be turned down, counted. Added to find out why an offset
    // surface flip is never accepted; the counters cost one relaxed atomic increment on a path
    // that is already refusing the operation, and nothing reads them unless a caller asks.
    //
    // The refusals TetMesh makes itself, below every hook (the valence re-test, an open ring, a
    // face or edge that already exists, no allowed 4-4 / 5-6 case beating the current cells, out
    // of slots, a duplicate tet), were once visible here only as before_pass - after_enter. They
    // are now reported one by one through TetMesh::op_event(); see op_accounting_report(). The
    // per-turn line of this block still prints that difference as `connectivity`.
    //
    // Face swaps count their own reasons (face_*, and base_before / interior_hook / base_after /
    // after_cells on their path) in the per-kind table alone -- swap_reject_kind_only() -- so
    // swap_reject_report(), which has only ever described edge swaps, keeps meaning what it meant.
    enum class SwapReject : int {
        base_before, // TetMesh::swap_*_before said no
        base_after, // TetMesh::swap_*_after said no
        valence, // wrong incident-tet count for this swap kind
        bbox, // edge on the bounding box
        surface_not_allowed, // has surface faces and allow_surface_swap() is false
        feature_edge, // the swap would delete a tagged edge, any kind (it is the operated edge)
        prepare_flip, // prepare_surface_flip() refused (see the flip_* reasons)
        interior_hook, // swap_before_interior() refused
        flip_open_boundary, // flipping would change the surface's boundary loops
        flip_not_two_surf, // not exactly two incident surface faces
        flip_nonmanifold_edge, // more than two incident surface faces
        flip_cd_nonmanifold, // a surface face already sits on the new edge (c,d)
        flip_new_face_surface, // (a,c,d) or (b,c,d) is already tagged surface
        flip_app_refused, // swap_before_surface() refused (see the app_* reasons)
        flip_wrong_case, // 4-4 / 5-6: the retetrahedralization is not the one that makes
        // the surface diagonal (c,d), so this case is not the flip
        app_capture_label, // application: the swap ring spans two construction labels
        app_capture_tag, // application: the swap ring spans two cell tags
        app_fid_missing, // application: a face of the pair could not be found
        app_class_mismatch, // application: the two faces are different tracked surfaces
        app_mask_mismatch, // application: the two faces carry different boundary masks
        app_sag_raised, // application: the flip would raise the local sag
        after_inverted, // a created cell is inverted
        after_quality, // swap_quality_allowed() said no
        after_cells, // swap_after_cells() refused (see the app_after_* reasons)
        app_after_side_conflict, // application: two ring vertices of a new cell sit on two sides
        app_after_no_side, // application: a new cell holds no ring vertex to take a side from
        after_envelope, // a new surface triangle left the envelope
        face_tracked_surface, // face swap: the face is a tracked surface face
        face_tracked_bbox, // face swap: the face is on the bounding box
        face_inverted, // face swap: one of the three new cells would be inverted
        face_not_better, // face swap: a new cell is not strictly better than the two old ones
        COUNT
    };
    /// Counted alongside the reasons: how far proposals get. surface_attempt counts edges that
    /// entered the surface branch at all, so "the surface never even presents a candidate" is
    /// distinguishable from "candidates are presented and refused".
    ///
    /// In a screened pass (ExecutePass::screen_before_commit) the stages up to before_pass are
    /// counted by the dry run as well as by the real attempt. `screened` counts the dry runs
    /// that passed every check, i.e. the before_pass counts a real attempt repeats.
    enum class SwapStage : int {
        attempt,
        surface_attempt,
        before_pass,
        after_enter,
        accepted,
        screened,
        COUNT
    };

    static const char* swap_reject_name(SwapReject r);
    static const char* swap_stage_name(SwapStage s);
    bool swap_reject(SwapReject r) const
    {
        m_swap_reject[size_t(r)].fetch_add(1, std::memory_order_relaxed);
        return swap_reject_kind_only(r);
    }
    /// The per-kind table alone (see op_accounting_report()), charged to the operation running
    /// on this thread. Face swaps use it directly; swap_reject() calls it for edge swaps.
    bool swap_reject_kind_only(SwapReject r) const
    {
        m_swap_reject_by_kind[size_t(current_op_kind())][size_t(r)].fetch_add(
            1,
            std::memory_order_relaxed);
        return false;
    }
    void swap_stage(SwapStage s) const
    {
        m_swap_stage[size_t(s)].fetch_add(1, std::memory_order_relaxed);
    }
    void swap_counters_reset();
    /// One line per nonzero counter, for a per-turn log. Does not reset.
    std::string swap_reject_report() const;

    mutable std::array<std::atomic<long>, size_t(SwapReject::COUNT)> m_swap_reject{};
    mutable std::array<std::atomic<long>, size_t(SwapStage::COUNT)> m_swap_stage{};

    // ---- COLLAPSE REJECTION INSTRUMENTATION (see op_accounting_report()) ----
    // Every place a collapse is turned down in a hook, counted once, at the place that decided.
    // Before this, 85% of the collapse attempts on the deliverable cube (target_distance_rel 1e-2
    // / front_conv_rel 1e-4, turn 5: 213k of 232k) were refused with nothing recorded but the
    // ops guard's 367. What TetMesh refuses below the hooks (the link condition, a duplicate tet)
    // arrives through op_event(). Before-hook reasons come first, after-hook reasons from
    // after_base on.
    enum class CollapseReject : int {
        coarsen_length, // coarsening pass only: the edge is at or above the collapse target
        bbox, // v1 sits on a bounding-box face that v2 does not
        surface_leaves_envelope, // v1 on the surface, v2 not, and v2 not inside the envelope
        inverted, // a cell the collapse reshapes would invert
        quality, // collapse_quality_allowed() said no
        substructure_link, // preserve_topology, both ends on the surface: substructure link
        feature_leaves_tube, // an edge tagged `feature` the collapse moves would leave its tube
        order2_leaves_tube, // an edge tagged `order2` the collapse moves would leave its tube
        feature_point, // v1 stands for a feature point and v2 is outside its ball
        app_both_surfaces, // application: an end on the input complex and an end on the offset
        app_front_unlimited, // application: an unlimited-length pass may not remove a front vertex
        app_leaves_input, // application: v1 on the input complex, v2 not
        app_leaves_offset, // application: v1 on the offset surface, v2 not
        app_leaves_region, // application: v1 on a region boundary, v2 not
        app_order2, // application: an order-2 vertex would merge into a lower-order one
        app_substructure_link, // application: the substructure link condition, unconditional
        app_ops_guard, // application: the ops divergence guard
        after_base, // TetMesh::collapse_edge_after said no
        after_connectivity, // collapse_after_connectivity() said no
        after_envelope, // a surface face the survivor keeps left the envelope
        after_face_attribute, // a face attribute found no face left to land on
        after_order2_chord, // an order-2 edge the survivor keeps is no longer order-2 substructure
        after_feature_edge, // a tagged feature edge found no edge left to land on
        after_coarsen_region, // coarsening only: the region's worst quality rose
        COUNT
    };
    static const char* collapse_reject_name(CollapseReject r);
    bool collapse_reject(CollapseReject r) const
    {
        m_collapse_reject[size_t(r)].fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    mutable std::array<std::atomic<long>, size_t(CollapseReject::COUNT)> m_collapse_reject{};

    // ---- OPERATION ACCOUNTING (counting only) ----
    // TetMesh reports every operation's attempt and its one outcome through op_event(); this
    // counts them per kind. op_accounting_report() then checks, per kind, that the outcomes add
    // up to the attempts, and that the before- and after-hook refusals add up to the reasons the
    // hooks counted (SwapReject / CollapseReject). A nonzero remainder is a refusal nobody named.
    void op_event(OpKind k, OpEvent e) const override
    {
        if (e == OpEvent::attempt) current_op_kind() = k;
        m_op_events[size_t(k)][size_t(e)].fetch_add(1, std::memory_order_relaxed);
        // The edge swaps whose hooks count SwapStage; see swap_reject_report().
        if (e == OpEvent::screened &&
            (k == OpKind::swap_32 || k == OpKind::swap_44 || k == OpKind::swap_56)) {
            swap_stage(SwapStage::screened);
        }
    }
    /// The operation running on this thread, set at its `attempt`, so a hook's swap_reject()
    /// can be charged to the right kind without passing the kind through every hook. COUNT
    /// (its own row, never reported) until the thread's first operation.
    static OpKind& current_op_kind()
    {
        static thread_local OpKind k = OpKind::COUNT;
        return k;
    }
    /// One line for kind @p k: attempts, the outcome partition, each hook's refusals split by
    /// reason, and the two remainders that must be 0. Does not reset.
    std::string op_accounting_report(OpKind k) const;
    /// True when the report for @p k has a nonzero remainder: a refusal no counter named.
    bool op_accounting_unexplained(OpKind k) const;
    void op_accounting_reset();
    static const char* op_kind_name(OpKind k);
    static const char* op_event_name(OpEvent e);

    /// Where a SwapReject is decided, for the partition. A LEAF is the one place a refusal was
    /// decided and is counted once. A WRAPPER re-counts on its way out a refusal a leaf further
    /// in already counted: prepare_flip around the flip_* leaves and flip_app_refused,
    /// flip_app_refused around the app_* leaves, interior_hook around app_capture_*, after_cells
    /// around app_after_*. flip_wrong_case is counted once per candidate CASE inside TetMesh's
    /// case loop, not once per operation, so it is in neither sum.
    enum class SwapRejectRole { before_leaf, after_leaf, wrapper, per_case };
    static SwapRejectRole swap_reject_role(SwapReject r);
    /// A kind's hook refusals by reason: the nonzero leaves as "name=n" and their sums.
    struct OpHookReasons
    {
        std::string before, after;
        long before_sum = 0, after_sum = 0;
        bool counted = true; // false: this kind's hooks count no reasons (split)
    };
    OpHookReasons op_hook_reasons(OpKind k) const;
    long op_event_count(const OpKind k, const OpEvent e) const
    {
        return m_op_events[size_t(k)][size_t(e)].load(std::memory_order_relaxed);
    }
    /// Attempts minus every outcome: 0 unless an exit of the TetMesh operation reports nothing.
    long op_unaccounted(OpKind k) const;

    mutable std::array<std::array<std::atomic<long>, size_t(OpEvent::COUNT)>, size_t(OpKind::COUNT)>
        m_op_events{};
    mutable std::
        array<std::array<std::atomic<long>, size_t(SwapReject::COUNT)>, size_t(OpKind::COUNT) + 1>
            m_swap_reject_by_kind{};

    /// Whether a swap's quality outcome is acceptable, given the max cell energy over the cells
    /// it creates and over the cells it destroyed. The default is STRICT improvement, which is
    /// what every swap here required before this hook existed, so a subclass that does not
    /// override it sees no change.
    ///
    /// `is_surface_flip` distinguishes a surface diagonal flip (see prepare_surface_flip) from an
    /// interior swap, so an application can relax the rule for the surface it tracks without
    /// touching interior swaps. The twin on the collapse side is collapse_quality_allowed().
    ///
    /// NOT consulted by swap_edge_56_after(), which has never had a quality gate: it computes a
    /// max energy and discards it. That is pre-existing and is left alone rather than silently
    /// changed, since it would move 5-6 behaviour for every consumer of this class.
    virtual bool swap_quality_allowed(double after, double before, bool /*is_surface_flip*/) const
    {
        return after < before;
    }

    struct SwapInfoCache
    {
        double max_energy;
        std::map<std::array<size_t, 3>, FaceAttributes> changed_faces;
        /// Edge tag bits of the affected tets, by sorted vid pair (m_track_feature_edges).
        std::map<std::array<size_t, 2>, uint8_t> changed_edges;

        bool is_surface_flip = false;
        size_t sf_a = 0, sf_b = 0, sf_c = 0, sf_d = 0;
        FaceAttributes sf_face_attr;
        /// Orientation of the flipped pair, measured along [a,b,c] (and so along [b,a,d]).
        int sf_orientation = 0;
        /// The pair's two faces were not oriented alike; see prepare_surface_flip.
        bool sf_orientation_mismatch = false;
    };
    wmtk::threading::enumerable_thread_specific<SwapInfoCache> swap_cache;

    struct SplitInfoCache
    {
        size_t v1_id = 0;
        size_t v2_id = 0;
        bool is_edge_on_surface = false;
        bool is_edge_open_boundary = false;
        /// The split edge's tag bits (FeatureEdgeAttributes::m_kinds); its two halves get them.
        uint8_t edge_kinds = 0;
        size_t edge_order = 0;
        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
        /// Edge tag bits of the affected tets, by sorted vid pair (m_track_feature_edges).
        std::map<std::array<size_t, 2>, uint8_t> changed_edges;
    };
    wmtk::threading::enumerable_thread_specific<SplitInfoCache> split_cache;

    struct CollapseInfoCache
    {
        size_t v1_id = 0;
        size_t v2_id = 0;
        double max_energy = 0.;
        double edge_length = 0.;
        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
        /// The tracked faces (v1,x,y) the collapse moves, as (v2,x,y), each with the attributes
        /// it will carry -- its own, or the merged ones of changed_faces where it becomes one
        /// face with (v2,x,y) -- for the containment check, which runs before they are written.
        std::vector<std::pair<std::array<size_t, 3>, FaceAttributes>> surface_faces;
        /// Oriented faces (v1,x,y) the collapse renames in place to (v2,x,y), as {v2,x,y} with
        /// the orientation measured along that order. Their attribute stays in its slot, so
        /// without this the value would be read against the wrong vertex set.
        std::vector<std::pair<std::array<size_t, 3>, int>> renamed_orientations;
        std::vector<std::array<size_t, 2>> boundary_edges;
        std::vector<size_t> changed_tids;
        std::vector<double> changed_energies;
        /// Edge tag bits of the affected tets, by sorted vid pair (m_track_feature_edges).
        std::map<std::array<size_t, 2>, uint8_t> changed_edges;
        /// Coarsening pass only: the worst relative quality in the region the composite may
        /// disturb, measured before the collapse. See collapse_edge_after.
        double region_max_rel_before = 0.;
    };
    wmtk::threading::enumerable_thread_specific<CollapseInfoCache> collapse_cache;

    // ---- Feature-edge tag propagation (m_track_feature_edges) ----------------------------
    //
    // An edge attribute's slot is (lowest incident tet id) * 6 + local index, so it moves
    // whenever that tet dies. Any edge whose slot can move is an edge of an affected tet
    // (deleted tets are affected; created tets only affect their own six edges), so caching
    // every edge of every affected tet before the operation and rewriting afterwards covers
    // every slot that can move -- the same argument the face tags rely on.
    //
    // Restore comes in two forms because the operations differ in what they create:
    //  * split and the swaps CREATE tets, whose slots may be reused and hold stale values;
    //    restore_region walks every edge of every created tet and writes the cached value or
    //    the default. The created region provably contains every surviving cached edge.
    //  * collapse creates NO tets, and its link edges (la,lb) -- which contain neither
    //    endpoint, a case a tet FACE cannot exhibit -- can live on in tets outside any
    //    enumerable region. restore_remap walks the CACHE instead: remaps v1 -> v2, OR-merges
    //    colliding pairs, and writes each surviving edge wherever its slot now is. Returns
    //    false if a TAGGED cached edge no longer exists (the caller aborts, mirroring the
    //    face path's try_tuple_from_face bailout).

    /// Cache the tag bits of every edge of every tet in `tids`.
    void feature_edges_cache(
        const std::vector<size_t>& tids,
        std::map<std::array<size_t, 2>, uint8_t>& cache);
    /// Write back tags over every edge of every tet in `tets` (created regions).
    void feature_edges_restore_region(
        const std::vector<Tuple>& tets,
        const std::map<std::array<size_t, 2>, uint8_t>& cache);
    void feature_edges_restore_region(
        const std::vector<size_t>& tids,
        const std::map<std::array<size_t, 2>, uint8_t>& cache);
    /// Write back tags for every cached edge, with `v_old` renamed to `v_new` (collapse).
    bool feature_edges_restore_remap(
        const std::map<std::array<size_t, 2>, uint8_t>& cache,
        size_t v_old,
        size_t v_new);

    /// Hook: the split vertex was created on an edge tagged `feature` (or not). Runs only when
    /// m_track_feature_edges is set; the counterpart of split_after_vertex for the feature
    /// flag, separate so existing overrides keep their signature.
    virtual void split_after_vertex_feature(size_t, bool) {}


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
