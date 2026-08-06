#pragma once

#include <wmtk/TetMesh.h>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include "Parameters.h"
#include "SimplicialComplexBVH.hpp"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

using CellTag = std::set<int64_t>;


namespace wmtk::components::topological_offset {


const int64_t TEMP_OFFSET_TET_TAG = -1;
const CellTag TEMP_OFFSET_TET_TAG_SET{TEMP_OFFSET_TET_TAG};


// for all attributes:
// label: 0=default, 1=input, 2=offset
class VertexAttributes
{
public:
    Vector3d m_posf;
    int label = 0;
    size_t component_id = 0;
    bool m_is_on_surface = false; // is this vertex on the input surface
    bool m_is_on_offset = false; // is this vertex on the offset surface
    std::vector<int> on_bbox_faces;
    size_t partition_id =
        0; // for parallel execution, see TopoOffsetTetMesh::compute_vertex_partition()

    /**
     * The order of a vertex in a TetMesh is as follows:
     * 0: vertex is not on the surface
     * 1: vertex is on the surface
     * 2: vertex is on the boundary of the surface or a non-manifold edge
     * 3: vertex is at the boundary of a non-manifold edge or a non-manifold vertex
     */
    size_t m_order = 0;

    VertexAttributes() {};
    VertexAttributes(const Vector3d& p);
};


class EdgeAttributes
{
public:
    int label = 0; // label: 0=default, 1=input, 2=offset
};


class FaceAttributes
{
public:
    int label = 0; // label: 0=default, 1=input, 2=offset
    /**
     * Is this face a part of the input surfaces.
     */
    bool m_is_surface_fs = false;
    bool m_is_offset_fs = false;

    /**
     * Keep track which bbox side the face is on
     * -1: none
     * 0/1: x min/max
     * 2/3: y min/max
     * 4/5: z min/max
     *
     * This bbox side ID is used to keep the bbox from collapsing.
     */
    int m_is_bbox_fs = -1; //-1; 0~5

    void reset()
    {
        m_is_surface_fs = false;
        m_is_offset_fs = false;
        m_is_bbox_fs = -1;
    }

    void merge(const FaceAttributes& attr)
    {
        m_is_surface_fs = m_is_surface_fs || attr.m_is_surface_fs;
        m_is_offset_fs = m_is_offset_fs || attr.m_is_offset_fs;
        if (attr.m_is_bbox_fs >= 0) m_is_bbox_fs = attr.m_is_bbox_fs;
    }
};


class TetAttributes
{
public:
    int label = 0; // label: 0=default, 1=input, 2=offset
    CellTag tag;
    double m_quality = 0; // AMIPS energy, kept up to date by smoothing
};


/**
 * @brief one sample of the input complex's implicit offset field, taken near a point on an
 * offset-surface face. `normal` is Vector3d::Zero() if the sample point landed exactly on the
 * input complex, in which case no normal direction could be recovered.
 * @note mirrors a single sample of
 * https://github.com/wildmeshing/topological-offsets/blob/main/components/topological_offsets/wmtk/components/topological_offsets/internal/utils/Quadrics.cpp's
 * get_triangle_samples_and_area()
 */
struct OffsetSurfaceSample
{
    Vector3d point; // the sample point, on (or near a corner of) the face
    Vector3d nearest; // nearest point on the input complex to `point`
    Vector3d normal; // (point - nearest).normalized()
    double weight; // 1 for the face centroid, 0.1 for the 3 near-corner samples
};


class TopoOffsetTetMesh : public wmtk::TetMesh
{
public: // mode for splitting in marching tets
    enum class EdgeSplitMode {
        Midpoint = 0, // used for simplicial embedding steps
        Initial = 1, // this is used to initialize the complex. Its a little hacky

        // only one of these is used, hard coded in execute_offset()
        BinarySearch = 2, // bisection root finding algo
        LogRootFind = 3, // 'custom' root finding, using the fact that d(x) - d* < 0 at first vertex
        SphereTracing = 4, // use sphere tracing to compute the zero of the distance field

        Optimization = 5 // this is used in the optimization phase of the algorithm
    };

public:
    int m_vtu_counter = 0;
    std::array<size_t, 4> m_init_counts = {{0, 0, 0, 0}};
    size_t m_tags_count;
    SimplicialComplexBVH m_input_complex_bvh;
    EdgeSplitMode m_edge_split_mode = EdgeSplitMode::Midpoint;

    // tag map stuff
    std::map<std::string, int64_t> m_tag_name_to_id;
    std::map<int64_t, std::string> m_tag_id_to_name;
    CellTag m_offset_output_tag_ids;

    // if in 'singlebody' mode
    bool m_singlebody = false;
    int64_t m_single_tag;

    // dont actually use, just for retaining in output
    bool m_has_envelope;
    std::vector<Vector3d> m_V_envelope;
    std::vector<Vector3i> m_F_envelope;
    std::shared_ptr<SampleEnvelope> m_envelope;
    double m_envelope_eps = -1;

    Parameters& m_params;

    // smoothing
    // thread-specific: a polysolve solver is not safe to call concurrently from multiple
    // threads (smooth_all_vertices() runs with num_threads > 0 via ExecutePass), so each
    // thread needs its own instance -- matching SimWildMesh::m_solver.
    wmtk::threading::enumerable_thread_specific<std::unique_ptr<polysolve::nonlinear::Solver>>
        m_smooth_solver;

    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    VertAttCol m_vertex_attribute;
    EdgeAttCol m_edge_attribute;
    FaceAttCol m_face_attribute;
    TetAttCol m_tet_attribute;

    TopoOffsetTetMesh(Parameters& _m_params, int _num_threads = 0)
        : m_params(_m_params)
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_edge_attrs = &m_edge_attribute;
        p_face_attrs = &m_face_attribute;
        p_tet_attrs = &m_tet_attribute;

        m_collapse_check_link_condition = false;
        m_collapse_check_manifold = false;

        optimization::deactivate_opt_logger();
    }

    ~TopoOffsetTetMesh() {}

    /**
     * @brief initialize TetMesh from vertex, tet, and tag data
     * @param V: #V by 3 vertex matrix
     * @param T: #T by 4 tet matrix
     * @param T_tags: #T by #tags tag matrix
     * @param V_env: #V_env by 3 EnvelopeSurface vertices
     * @param F_env: #F_env by 3 EnvelopeSurface faces
     */
    void init_from_image(
        const MatrixXd& V,
        const MatrixXi& T,
        const MatrixSi& T_tags,
        const MatrixXd& V_env,
        const MatrixXi F_env,
        const std::vector<std::string>& tag_names);

    void init_surfaces_and_boundaries();

    bool is_edge_on_surface(const Tuple& loc);
    bool is_edge_on_offset(const Tuple& loc);

    /**
     * @brief check that the ambient tag does not overlap with any other tags
     */
    bool ambient_assert();

    /**
     * @brief label input simplicial complex simplices, as defined in m_params.offset_selection
     */
    void label_input_complex();

    /**
     * @brief check if the input complex is empty. Only valid after calling init_from_image(...).
     * Checks if any vertices (therefore any simplices) are labelled 1, if not returns true
     */
    bool empty_input_complex();

    /**
     * @brief initialize BVH for input complex. Must be called after init_from_image(...)
     */
    void init_input_complex_bvh();

    /**
     * @deprecated
     * @brief split edge at point by minimizing m_params.target_distance - d() (where d() is
     * distance to input complex via BVH) along the edge. Uses binary search, so implicitly assumes
     * distance field is monotonic along edge. May give weird results if not monotonic
     */
    void edge_split_binary_search(const size_t v1, const size_t v2, Vector3d& p_new) const;
    void edge_split_binary_search(const Vector3d& v1_pos, const Vector3d& v2_pos, Vector3d& p_new)
        const;

    /**
     * @deprecated
     * @brief uses custom root finding routine, attempting to find first root (nearest to v1)
     * to split edge at
     */
    void edge_split_log_root_find(const size_t v1, const size_t v2, Vector3d& p_new) const;

    /**
     * @brief split edge at first root of d(l) - d*, where d(l) is distance to input complex,
     * using sphere tracing method. This is the best method and should be used instead of
     * binary or log root finding methods
     */
    void edge_split_sphere_tracing(const size_t v1, const size_t v2, Vector3d& p_new) const;

    /**
     * @brief label connected simplicial complex components (simplices labelled 1 or 2)
     */
    size_t flood_fill();

    std::vector<std::array<size_t, 3>> get_faces_by_condition(
        std::function<bool(const FaceAttributes&)> cond) const;

    //// overriden splits/invariants
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    bool split_tet_before(const Tuple& t) override;
    bool split_tet_after(const Tuple& t) override;
    bool invariants(const std::vector<Tuple>& tets) override;
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;
    //// overriden splits/invariants

    /**
     * @brief main function from which all others are called
     */
    void execute_offset(const std::filesystem::path& output_file);

    /**
     * @brief optimize the offset
     */
    void optimize_offset(const std::filesystem::path& output_file);

    //// smoothing
    /**
     * @brief AMIPS quality of a tet, given its (unordered) vertex ids
     * @note skeleton: plain double AMIPS, no rational fallback for near-degenerate tets
     */
    double get_quality(const std::array<size_t, 4>& vids) const;
    double get_quality(const Tuple& t) const;

    /**
     * @brief true if the tet given by vids (or by tuple, via oriented_tet_vids()) is inverted,
     * following SimWildMesh::is_inverted().
     * @note skeleton: SimWild falls back to a rational orient3d when a vertex isn't rounded yet
     * (m_is_rounded); this class has no rational position tracking, so it always uses the
     * plain double predicate, same as invariants().
     */
    bool is_inverted(const std::array<size_t, 4>& vids) const;
    bool is_inverted(const Tuple& t) const;

    /**
     * @brief run vertex smoothing over the whole mesh for n_iters passes
     * @note skeleton: single-threaded. Input-complex vertices (label==1) never move. Offset
     * surface vertices are optimized with quadrics toward target_distance from the input
     * complex, see smooth_after_offset_surface(). All other vertices get plain AMIPS
     * smoothing, see smooth_after_interior().
     */
    void smooth_all_vertices(size_t n_iters = 1);

    /**
     * @brief true if face f has exactly one incident tet labelled 2 (offset), i.e. it lies on
     * the boundary between the offset region and the rest of the mesh
     */
    bool is_offset_face(const Tuple& f) const;
    bool is_offset_face(const size_t fid) const;

    /**
     * @brief offset-surface faces (see is_offset_face()) incident to vertex t
     */
    std::vector<Tuple> get_offset_surface_faces_for_vertex(const Tuple& t) const;

    /**
     * @brief the 4 offset-field samples for face f (centroid + one near each corner), following
     * Quadrics.cpp's get_triangle_samples_and_area(). Shared by the Quadrics construction below
     * and by the feature-preserving checks in Collapse.cpp -- a single centroid sample cannot
     * tell a locally-flat patch from one straddling a sharp feature of the input complex, since
     * a single BVH nearest-point query just picks whichever side of the feature is closer.
     */
    std::array<OffsetSurfaceSample, 4> offset_surface_samples(const Tuple& f) const;

    /**
     * @brief AMIPS-only smoothing step, for vertices not on the offset surface
     */
    bool smooth_after_interior(const Tuple& t);

    /**
     * @brief quadrics-based smoothing step for offset surface vertices: blends a Laplacian
     * step with a projection onto quadrics built from target_distance-offset samples of the
     * input complex, following
     * https://github.com/wildmeshing/topological-offsets/blob/main/components/topological_offsets/wmtk/components/topological_offsets/internal/OffsetOptimization.cpp#L2226
     * @note skeleton: no bisection fallback toward p0 on rejection.
     */
    bool smooth_after_offset_surface(const Tuple& t);
    //// smoothing

    //// collapse
    /**
     * @brief collapse edges shorter than min_edge_len_ratio * target_distance once, over the
     * whole mesh
     * @note skeleton: single-threaded, single pass (no repeated sweeps until convergence, no
     * priority queue -- just one pass over get_edges() in whatever order it returns)
     */
    void collapse_all_edges();

    /**
     * @brief max angle (degrees, 0-90, orientation independent) allowed between an
     * offset-surface face's own normal and the input-complex normal it is supposed to
     * approximate, checked by collapse_edge_before/after. See face_normal_deviation() and
     * https://github.com/wildmeshing/topological-offsets/blob/main/components/topological_offsets/wmtk/components/topological_offsets/internal/invariants/NormalDeviationAfterInvariant.cpp
     * and .../invariants/OffsetCollapseBeforeInvariant.cpp
     */
    double m_max_normal_deviation_deg = 15.0;

    /**
     * @brief max angle (degrees, 0-90, orientation independent) between offset-surface face
     * f's own normal and any of its offset_surface_samples() normals. Taking the max over all
     * 4 samples (rather than a single centroid sample) is what makes this sensitive to
     * features: a face straddling a sharp fold of the input complex will have samples on
     * either side of the fold, so at least one of them disagrees strongly with the face's own
     * (necessarily flat) normal, even though the face may look locally fine from any single
     * sample alone.
     */
    double face_normal_deviation(const Tuple& f) const;

    /**
     * @brief max face_normal_deviation() over the offset-surface faces incident to vertex vid
     */
    double max_offset_surface_normal_deviation_at_vertex(size_t vid) const;

    /**
     * @brief OffsetCollapseBeforeInvariant analogue: pools offset_surface_samples() normals
     * from every offset-surface face incident to remove_vid and returns the spread
     * (max - min, degrees, orientation dependent via normal_angle_180-style acos) of their
     * angles to the collapsing edge's direction. A small spread means the samples around the
     * survivor agree with each other relative to the edge, i.e. the edge runs along a locally
     * flat/consistent part of the offset surface; a large spread means the true target-normal
     * field disagrees with itself nearby -- a sign of a feature edge -- and collapsing there
     * should be rejected. See
     * https://github.com/wildmeshing/topological-offsets/blob/main/components/topological_offsets/wmtk/components/topological_offsets/internal/invariants/OffsetCollapseBeforeInvariant.cpp
     */
    double collapse_normal_deviation(const Tuple& edge, size_t remove_vid) const;

    /**
     * @brief link condition for edge collapse, restricted to the offset region (tets labelled
     * 2), using the coning trick: the offset sub-complex's own boundary (its offset-surface
     * faces) is treated as attached to one shared virtual vertex, closing it into a complex
     * without boundary so the standard link condition applies to it directly.
     *
     * TetMesh::collapse_edge already runs the *whole-mesh* link condition (via
     * m_collapse_check_link_condition), which only guarantees the full tet complex (all
     * labels) stays manifold -- it says nothing about whether the offset region's own boundary
     * stays manifold, since there is generally enough surrounding label-0 material to keep the
     * whole mesh's link condition satisfied even when the label-2 sub-complex's boundary would
     * be pinched or split by the collapse. This is the analogue of offset_tet_consistent_topology()
     * (used when *growing* the region) for the collapse direction.
     *
     * A no-op (always returns true) for edges that don't touch any label-2 tet.
     */
    bool offset_link_condition(const Tuple& edge) const;
    //// collapse

    //// split (optimization-phase; separate from the marching-tets split machinery below)
    /**
     * @brief split edges longer than splitting_l2 (set in Parameters::init() from
     * length/length_rel) once, over the whole mesh, longest first. Reuses the existing
     * split_edge_before/after (EdgeSplitMode::Midpoint) already used by simplicial_embedding(),
     * so label/tag inheritance for the new vertex and split simplices is unchanged from that
     * path -- this just decides *which* edges to split and drives them through ExecutePass,
     * mirroring collapse_all_edges()/smooth_all_vertices().
     * @note skeleton: no quality- or feature-based gating beyond the length threshold.
     */
    void split_all_edges();
    //// split

    //// swap (2-3 / 3-2 edge-to-face flip only; no 4-4, 5-6, or face swap yet)
    /**
     * @brief swap_edge, restricted to edges whose 3 incident tets already share the same
     * label and tag: the new tets/faces then unambiguously inherit that label/tag, and the
     * offset/input region boundaries -- which live entirely in *which* tets carry which label
     * -- cannot move, since a swap confined to a single-label neighborhood never changes any
     * tet's label. Also stays away from the input and offset surfaces entirely (see
     * m_is_on_surface / get_offset_surface_faces_for_vertex), and is only accepted if it
     * strictly improves the worst AMIPS quality among the affected tets, matching
     * SimWildMesh::swap_edge_before/after.
     */
    void swap_all_edges();
    //// swap

    /**
     * @brief execute simplistic marching tets. All edges with one vertex labelled 0 and the other 1/2
     * are split. If m_edge_split_mode=BinarySearch, edges are split according to BVH distance field
     * and the offset target distance (m_params.target_distance). If m_edge_split_mode=Midpoint,
     * edges are split at the midpoint
     */
    void marching_tets();


    //// simplicial embedding stuff
    /**
     * @brief check if the input complex (simplices labelled 1) are simplicially embedded w.r.t. the
     * entire mesh
     */
    bool is_simplicially_embedded() const;

    /**
     * @brief check if a tet satisfies simpicial embedding criteria w.r.t. input complex
     * (simplices labelled 1)
     */
    bool tet_is_simp_emb(const Tuple& t) const;

    /**
     * @brief make mesh a simplicial embedding of the input complex (simplices labelled 1)
     */
    void simplicial_embedding();
    //// simplicial embedding stuff

    //// variable offset stuff
    /**
     * @brief check if removing the tet would change the topology of any label
     */
    bool tag_tet_consistent_topology(size_t t_id, int64_t tag) const;

    /**
     * @brief check if adding a tet to the offset region does not change the topology of the
     * offset. Returns true if topology would not be changed
     */
    bool offset_tet_consistent_topology(const size_t t_id) const;

    /**
     * @brief check if a tet is inside the offset (implicitly defined via BVH distance field to
     * input complex) via conservative sphere subdivision estimation
     */
    bool tet_is_in_offset_conservative(const size_t t_id, const double threshold_r) const;

    /**
     * @brief check if a tet is inside the offset (implicitly defined via BVH distance field to
     * input complex) by check if all its vertices are inside the offset region.
     */
    bool tet_is_in_offset_aggressive(const size_t t_id) const;

    /**
     * @brief grow offset region conservatively using conservative checks while ensuring consistent
     * topology
     */
    void grow_offset_conservative();

    /**
     * @brief Grow offset region aggressively. A tet is considered in the offset if all its vertices
     * are in the offset.
     */
    void grow_offset_aggressive();
    //// variable offset stuff

    /**
     * @brief update 'tags' data for tets in the offset region (tets labelled 2) based on
     * the given offset tag values in m_params.offset_tag_value
     */
    void set_offset_tet_tags();

    /**
     * @brief verify that the closed offset region (simplices labelled 1 or 2) form a manifold
     * region. This should be true for any offset. This function is for verification.
     * @note We first collect the tets labelled 1 or 2, then extract the boundary of this region
     * and check if it is manifold.
     */
    bool offset_is_manifold();

    //// output stuff
    void write_input_complex(const std::string& path); // write components labeled to be offset
    void write_vtu(const std::string& path);
    // void write_msh(const std::string& file);
    void write_msh_groups(const std::string& file);
    //// output stuff

private:
    /**
     * @note for all split caches, simplex attributes are inherited from the simplex (of same or
     * higher order) they are 'borne' out of
     */

    struct EdgeSplitCache
    {
        size_t v1_id;
        size_t v2_id;
        VertexAttributes new_v;

        bool is_edge_on_surface = false;
        bool is_edge_on_offset = false;
        bool is_edge_open_boundary = false;

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;

        // cache edge attributes
        EdgeAttributes split_e;
        std::map<size_t, EdgeAttributes> internal_e;
        std::map<simplex::Edge, EdgeAttributes> external_e; // edge is boundary edge (not link)
        std::map<simplex::Edge, EdgeAttributes> link_e; // link edge around splitted edge

        // cache face attributes
        std::map<size_t, FaceAttributes> split_f; // splitted faces
        std::map<simplex::Edge, FaceAttributes> internal_f; // new faces created by split
        std::map<std::pair<simplex::Edge, size_t>, FaceAttributes>
            external_f; // closed star boundary faces of splitted edge

        // cache tet attributes
        std::map<simplex::Edge, TetAttributes> tets;
    };
    wmtk::threading::enumerable_thread_specific<EdgeSplitCache> edge_split_cache;

    struct FaceSplitCache
    {
        size_t v1_id;
        size_t v2_id;
        size_t v3_id;

        // cache edge attributes
        std::map<simplex::Edge, EdgeAttributes> existing_e;

        // cache face attributes
        std::map<simplex::Face, FaceAttributes> existing_f;
        int splitf_label;

        // cache tet attributes
        std::map<size_t, TetAttributes> tets;
    };
    wmtk::threading::enumerable_thread_specific<FaceSplitCache> face_split_cache;

    struct TetSplitCache
    {
        std::array<size_t, 4> v_ids;

        // cache retained edge attributes
        std::map<simplex::Edge, EdgeAttributes> existing_e;

        // cache retained face attributes
        std::map<simplex::Face, FaceAttributes> existing_f;

        // cache tet attribute
        TetAttributes tet;
    };
    wmtk::threading::enumerable_thread_specific<TetSplitCache> tet_split_cache;

    /**
     * @brief snapshot of edge/face labels around the collapsed vertex, keyed by vertex ids
     * rather than eid()/fid() -- those are derived from the lowest-id incident tet, which can
     * change once the tets touching the collapsed edge are removed, so a label read back
     * through eid()/fid() after the collapse is not reliably the one that was there before
     */
    struct EdgeCollapseCache
    {
        size_t v1_id; // removed by the collapse
        size_t v2_id;
        double max_energy;
        double edge_length;
        bool is_limit_length;

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
        // all faces incident to the delete vertex (v1) that are on the tracked surface
        std::vector<std::array<size_t, 3>> surface_faces;
        // all faces incident to the delete vertex (v1) that are on the tracked surface
        std::vector<std::array<size_t, 3>> offset_faces;
        // all edges incident to the deleted vertex(v1) that are on the open boundary
        std::vector<std::array<size_t, 2>> boundary_edges;
        std::vector<size_t> changed_tids;
        std::vector<double> changed_energies;

        // NormalDeviationAfterInvariant analogue: the worse of the two endpoints' offset
        // surface deviation before the collapse, so collapse_edge_after() can tell whether a
        // deviation over the threshold afterward is a regression or was already there
        double nd_before = 0.;
    };
    wmtk::threading::enumerable_thread_specific<EdgeCollapseCache> edge_collapse_cache;

    /**
     * @brief snapshot for swap_edge, keyed by vertex ids for the same reason as
     * EdgeCollapseCache: the 3 incident tets get replaced by 2 new ones, so eid()/fid() of the
     * edges/faces that survive the swap can point at a different (unwritten) slot afterward.
     * Unlike collapse, no vertex disappears here -- only the swapped edge itself and its 3
     * incident faces vanish, and exactly one new face (the flipped-in triangle) appears -- so
     * every other edge/face of the 3 old tets is cached and later restored verbatim.
     */
    struct SwapEdgeCache
    {
        int common_label = 0; // label shared by all 3 incident tets before the swap
        CellTag common_tag; // tag shared by all 3 incident tets before the swap
        double max_quality_before = 0.;
        std::map<simplex::Edge, int> edge_labels; // all edges except the swapped edge itself
        std::map<simplex::Face, int> face_labels; // all faces except the 3 that vanish with it
    };
    wmtk::threading::enumerable_thread_specific<SwapEdgeCache> swap_edge_cache;

public:
    // substructure functions

    bool is_order_2_edge(const Tuple& e) const;
    bool is_order_2_edge(const std::array<size_t, 2>& e) const;

    bool vertex_is_on_surface(const size_t vid) const override;

    bool face_is_on_surface(const size_t fid) const override;

    size_t get_order_of_vertex(const size_t vid) const override;
    /**
     * @brief Compute the vertex order for every vertex.
     */
    void init_vertex_order();

private: // helpers
    /**
     * @brief determine if any tag from tag1 is also present in tag2.
     * @note if tag2 is empty (ambient), return true if tag1 is empty, otherwise false (tag2 is
     * ambient, so only 'element' is ambient)
     */
    bool any_tag_present(const CellTag& tag1, const CellTag& tag2)
    {
        if (tag2.empty()) {
            return tag1.empty();
        }
        if (tag1.empty()) { // tag1 is ambient, tag2 is not
            return false;
        }

        for (const int64_t& i : tag1) {
            if (tag2.find(i) != tag2.end()) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief sort edge simplices in place by decreasing edge length
     */
    void sort_edges_by_length(std::vector<simplex::Edge>& edges)
    {
        std::sort(
            edges.begin(),
            edges.end(),
            [this](const simplex::Edge& e1, const simplex::Edge& e2) {
                double len1 = (m_vertex_attribute[e1.vertices()[0]].m_posf -
                               m_vertex_attribute[e1.vertices()[1]].m_posf)
                                  .norm();
                double len2 = (m_vertex_attribute[e2.vertices()[0]].m_posf -
                               m_vertex_attribute[e2.vertices()[1]].m_posf)
                                  .norm();
                return len1 > len2;
            });
    }

public: // helpers
    /**
     * @brief assign each vertex a partition id (by spatial Morton order) for the parallel
     * ExecutePass policies (kPartition/kColor) used by smooth_all_vertices()/
     * collapse_all_edges(). A no-op if NUM_THREADS == 0.
     */
    void compute_vertex_partition();

    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }

    /**
     * @brief get tets (as Tuples) that are face-adjacent to the given tet (as Tuple)
     */
    std::vector<Tuple> get_face_adjacent_tets(const Tuple& t) const
    {
        std::vector<Tuple> adj_tets;
        auto tet_1 = t.switch_tetrahedron(*this);
        if (tet_1) {
            adj_tets.push_back(tet_1.value());
        }
        auto tet_2 = t.switch_face(*this).switch_tetrahedron(*this);
        if (tet_2) {
            adj_tets.push_back(tet_2.value());
        }
        auto tet_3 = t.switch_edge(*this).switch_face(*this).switch_tetrahedron(*this);
        if (tet_3) {
            adj_tets.push_back(tet_3.value());
        }
        auto tet_4 =
            t.switch_vertex(*this).switch_edge(*this).switch_face(*this).switch_tetrahedron(*this);
        if (tet_4) {
            adj_tets.push_back(tet_4.value());
        }
        return adj_tets;
    }

    /**
     * @brief get all one-ring vertices through input simplices (labelled 1)
     */
    std::vector<size_t> connected_components_helper(const size_t& v_id)
    {
        auto onering_v_ids = get_one_ring_vids_for_vertex(v_id);
        std::vector<size_t> ret_v_ids;
        for (const size_t& other_v_id : onering_v_ids) {
            size_t e_id = tuple_from_edge({{v_id, other_v_id}}).eid(*this);
            if (m_edge_attribute[e_id].label != 0) { // edge labelled 1 or 2
                ret_v_ids.push_back(other_v_id);
            }
        }
        return ret_v_ids;
    }

    /**
     * @brief reset connected component assignments.
     */
    void reset_connected_components()
    {
        auto verts = get_vertices();
        for (const Tuple& v : verts) {
            size_t v_id = v.vid(*this);
            m_vertex_attribute[v_id].component_id = 0;
        }
    }
};


} // namespace wmtk::components::topological_offset
