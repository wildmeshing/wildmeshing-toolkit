#pragma once

#include <igl/Timer.h>
#include <wmtk/SurfaceTagAttributes.h>
#include <wmtk/TetMesh.h>
#include <wmtk/TetOptimizerMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include <algorithm>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/threading/concurrent_map.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/threading/parallel_for.hpp>

#include "Parameters.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <VolumeRemesher/embed.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <igl/remove_unreferenced.h>
#include <memory>
#include <set>
#include <unordered_set>
#include <utility>
#include <wmtk/utils/SurfaceTopology.hpp>
#include <wmtk/utils/partition_utils.hpp>

namespace wmtk::components::tetwild {

/// The shared attribute types live on the base; keep the unqualified names working.
using VertexAttributes = wmtk::TetOptimizerMesh::VertexAttributes;
using FaceAttributes = wmtk::TetOptimizerMesh::FaceAttributes;

// TODO: missing comments on what these attributes are
class TetAttributes
{
public:
    double m_quality;
    double m_winding_number_input = 0; // winding number w.r.t. the input
    double m_winding_number_tracked = 0; // winding number w.r.t. the tracked surface
    std::vector<double> m_winding_number_per_input;
    int part_id = -1; // flood fill ID
};

class TetWildMesh : public wmtk::TetOptimizerMesh
{
public:
    /**
     * @brief tetwild's per-vertex additions to the shared VertexAttributes.
     *
     * Registered with the base's m_vertex_attr_group, so it is resized, protected and rolled
     * back exactly like the shared collection -- which matters, because collapse_edge_after and
     * split_edge_after both write it.
     */
    struct VertexExtras
    {
        /// Whether the vertex lies on an open boundary of the input surface. The 2D
        /// counterpart is TriWildMesh::VertexExtras::m_feature_id, which differs deliberately
        /// -- see the comment there.
        bool m_is_on_open_boundary = false;
    };
    wmtk::AttributeCollection<VertexExtras> m_vertex_extra;

    /// The base holds only wmtk::OptimizerParameters; this is the same object, typed, for the
    /// tetwild-only fields.
    Parameters& m_tet_params;

    /// Envelope for order-2 vertices, i.e. those on a surface boundary or a non-manifold
    /// edge. Named for the order rather than for "open boundary" because that is what
    /// TetMesh::compute_vertex_order actually reports, and it is the broader set.
    std::shared_ptr<SampleEnvelope> m_order2_envelope;

    /// Optional per-input names (JSON "input_names"), used to label the per-input
    /// winding-number output fields. Empty => the fields are numbered.
    std::vector<std::string> m_input_names;

    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    TetAttCol m_tet_attribute;

    double cell_quality(const size_t tid) const override { return m_tet_attribute[tid].m_quality; }
    void set_cell_quality(const size_t tid, const double q) override
    {
        m_tet_attribute[tid].m_quality = q;
    }

    /// Iterations mesh_improvement actually used. Reported so a run that needs the whole
    /// budget is visible as such, and asserted against in the integration tests.
    int m_iterations_used = 0;

    /// Envelope a vertex is pulled toward while smoothing.
    std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(const size_t vid) const;
    TetWildMesh(
        Parameters& _m_params,
        std::shared_ptr<SampleEnvelope> _m_envelope,
        int _num_threads = 1)
        : wmtk::TetOptimizerMesh(_m_params, std::move(_m_envelope))
        , m_tet_params(_m_params)
    {
        m_vertex_attr_group.add(&m_vertex_extra);
        NUM_THREADS = _num_threads;
        p_tet_attrs = &m_tet_attribute;
        m_collapse_check_link_condition = false;
        m_collapse_check_manifold = false;

        optimization::deactivate_opt_logger();
        m_s_envelope = 1. / (m_params.diag_l * m_params.eps * m_params.eps);
        m_params.w_envelope = 1. - m_params.w_amips;
    }

    ~TetWildMesh() {}

    // only used with unit tests
    void create_mesh_attributes(
        const std::vector<VertexAttributes>& _vertex_attribute,
        const std::vector<TetAttributes>& _tet_attribute)
    {
        const size_t n_tet = _tet_attribute.size();
        m_vertex_attribute.resize(_vertex_attribute.size());
        // The extras carry no data a caller supplies -- every field defaults -- so they only
        // have to be sized alongside the shared collection.
        m_vertex_extra.resize(_vertex_attribute.size());
        m_face_attribute.resize(4 * n_tet);
        m_tet_attribute.resize(n_tet);

        // new for edge
        // m_edge_attribute.resize(6 * n_tet);

        for (size_t i = 0; i < _vertex_attribute.size(); i++)
            m_vertex_attribute[i] = _vertex_attribute[i];

        // Keep whatever init() reserved. AttributeCollection::resize only ever grows, so the
        // calls above cannot shrink a collection -- but assigning m_attributes directly does,
        // and it used to drop the tet attributes to exactly n_tet. The attributes have to stay
        // at least as large as the connectivity: an operation that creates a new element
        // indexes them by its id, and a 5->6 swap creates one. That left m_tet_attribute one
        // short of the tet the swap adds, and AttributeCollection::operator[] read past the end
        // of it -- ASan reports a heap-buffer-overflow reached from swap_edge_56_after, and
        // libstdc++ then aborts on the corrupted heap a few allocations later, while libc++
        // carries on and the tests pass.
        const size_t tcap = std::max(n_tet, m_tet_attribute.size());
        m_tet_attribute.m_attributes = std::vector<TetAttributes>(tcap);
        for (size_t i = 0; i < n_tet; i++) m_tet_attribute[i] = _tet_attribute[i];
        for (size_t i = 0; i < n_tet; i++)
            m_tet_attribute[i].m_quality = get_quality(tuple_from_tet(i));
    }

    ////// Attributes related

    void output_mesh(std::string file);
    void init_from_delaunay_box_mesh(const std::vector<Eigen::Vector3d>& vertices);

public:
    void split_all_edges();
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& loc) override;

    bool smooth_after(const Tuple& t) override;

    void smooth_all_vertices();
    void collapse_all_edges(bool is_limit_length = true);
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    size_t swap_all_edges_44();
    bool swap_edge_44_before(const Tuple& t) override;
    bool swap_edge_44_accept_case(const std::array<size_t, 2>& new_edge) override;
    bool swap_edge_44_after(const Tuple& t) override;

    size_t swap_all_edges_56();
    bool swap_edge_56_before(const Tuple& t) override;
    bool swap_edge_56_accept_case(const std::array<size_t, 3>& new_face) override;
    bool swap_edge_56_after(const Tuple& t) override;

    size_t swap_all_edges_32();
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    /**
     * @brief Prepare a surface edge swap (a surface diagonal flip), any ring size.
     *
     * Called from swap_edge_before / swap_edge_44_before / swap_edge_56_before when the swapped
     * edge (a,b) is on the surface. Regardless of how many tets share (a,b), the surface change is
     * always the 2D diagonal flip of the two incident surface faces (a,b,c),(a,b,d) into
     * (a,c,d),(b,c,d). This verifies the ring-size-independent guards that guarantee the flip
     * preserves surface manifoldness / topology and fills the surface-flip fields of swap_cache.
     * The specific retetrahedralization that realizes (c,d) is selected later by
     * swap_edge_44_accept_case / swap_edge_56_accept_case (the 3->2 path always realizes it).
     * Returns false (rejecting the swap) if any guard fails: open-boundary edge, non-manifold edge
     * (!= 2 surface faces incident to (a,b)), the target edge (c,d) already carries a surface face,
     * or one of the two would-be new surface faces (a,c,d),(b,c,d) is already tagged surface. The
     * tets sharing (a,b) are passed in to avoid recomputation.
     */
    bool prepare_surface_flip(const Tuple& t, const std::vector<size_t>& incident_tets);

    /// A topological fingerprint of the tracked surface (m_is_surface_fs). See
    /// wmtk/utils/SurfaceTopology.hpp.
    using SurfaceTopoSignature = wmtk::utils::SurfaceTopoSignature;

    SurfaceTopoSignature surface_topology_signature() const
    {
        return wmtk::utils::surface_topology_signature(*this, [this](size_t fid) {
            return m_face_attribute[fid].m_is_surface_fs;
        });
    }

    /**
     * @brief Compare a surface signature against the current one and log an
     * error if it changed. Used (when m_tet_params.check_surface_topology is set) to
     * guard swap passes that can flip surface edges.
     */
    void warn_if_surface_topology_changed(const SurfaceTopoSignature& before, const char* where)
        const
    {
        wmtk::utils::warn_if_surface_topology_changed(before, surface_topology_signature(), where);
    }

    size_t swap_all_faces();
    bool swap_face_before(const Tuple& t) override;
    bool swap_face_after(const Tuple& t) override;

    size_t swap_all_edges_all();

    //
    /**
     * brief Check if the vertex has an incident boundary edge.
     * This performs a topological check.
     */
    bool is_vertex_on_boundary(const size_t vid);
    //
    void mesh_improvement(int max_its = 80);
    /**
     * @brief Call the original TetWild code.
     */
    void mesh_improvement_legacy(int max_its = 80);
    std::tuple<double, double> local_operations(
        const std::array<int, 4>& ops,
        bool collapse_limit_length = true);

    /**
     * @brief Compute the winding number.
     *
     * If `vertices` and `faces` are empty, compute the winding number for the tracked surface.
     * Otherwise, compute the winding number for the input surface given by `vertices` and `faces`.
     */
    /**
     * @brief Barycenter (row per tet) of each tet in `tets`. Computed once and
     * passed to the winding-number passes so they do not each rebuild it.
     */
    Eigen::MatrixXd tet_barycenters(const std::vector<Tuple>& tets) const;

    void compute_winding_number(
        const std::vector<Tuple>& tets,
        const Eigen::MatrixXd& barycenters,
        const std::vector<Vector3d>& vertices = {},
        const std::vector<std::array<size_t, 3>>& faces = {});

    // `in_vertices`/`in_faces` let the single-input case reuse the already-loaded
    // surface instead of re-reading it from disk.
    void compute_winding_numbers(
        const std::vector<std::string>& input_paths,
        const std::vector<Tuple>& tets,
        const Eigen::MatrixXd& barycenters,
        const std::vector<Vector3d>& in_vertices = {},
        const std::vector<std::array<size_t, 3>>& in_faces = {});

    void filter_with_input_surface_winding_number();
    void filter_with_tracked_surface_winding_number();
    void filter_with_flood_fill();


    double get_length2(const Tuple& loc) const;
    // debug use
    std::atomic<int> cnt_split = 0, cnt_collapse = 0, cnt_swap = 0;
    // Successful surface diagonal flips (subset of cnt_swap). Diagnostic.
    // cnt_surface_swap is the grand total; the per-type counters break it down by the swap that
    // realized the flip (3->2, 4-4, 5-6).
    std::atomic<int> cnt_surface_swap = 0;
    std::atomic<int> cnt_surface_swap_32 = 0, cnt_surface_swap_44 = 0, cnt_surface_swap_56 = 0;

private:
    // tags: correspondence map from new tet-face node indices to in-triangle ids.
    // built up while triangles are inserted.
    wmtk::threading::concurrent_map<std::array<size_t, 3>, std::vector<int>> tet_face_tags;

    struct TriangleInsertionLocalInfoCache
    {
        // local info: for each face insertion
        int face_id;
        std::vector<std::array<size_t, 3>> old_face_vids;
    };
    wmtk::threading::enumerable_thread_specific<TriangleInsertionLocalInfoCache>
        triangle_insertion_local_cache;

    ////// Operations

    struct SplitInfoCache
    {
        //        VertexAttributes vertex_info;
        size_t v1_id;
        size_t v2_id;
        bool is_edge_on_surface = false;
        bool is_edge_open_boundary = false;
        size_t edge_order = 0;
        /// Worst quality among the tets incident to the edge BEFORE the split, so
        /// split_edge_after can tell "this split created a degenerate tet" from "this split
        /// subdivided a region that was already degenerate".
        double max_quality_before = 0.;

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
    };
    wmtk::threading::enumerable_thread_specific<SplitInfoCache> split_cache;

    struct CollapseInfoCache
    {
        size_t v1_id;
        size_t v2_id;
        double max_energy;
        double edge_length;

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
        // all faces incident to the delete vertex (v1) that are on the tracked surface
        std::vector<std::array<size_t, 3>> surface_faces;
        // all edges incident to the deleted vertex(v1) that are on the open boundary
        std::vector<std::array<size_t, 2>> boundary_edges;
        std::vector<size_t> changed_tids;
        std::vector<double> changed_energies;
    };
    wmtk::threading::enumerable_thread_specific<CollapseInfoCache> collapse_cache;


    struct SwapInfoCache
    {
        double max_energy;
        std::map<std::array<size_t, 3>, FaceAttributes> changed_faces;

        // Surface diagonal-flip bookkeeping (filled by prepare_surface_flip from
        // swap_edge_before / swap_edge_44_before / swap_edge_56_before when the swapped edge (a,b)
        // lies on the surface). a,b are the removed-edge endpoints, c,d are the new surface-edge
        // endpoints (the apexes of the two incident surface faces). sf_face_attr is copied onto the
        // two new surface faces (a,c,d),(b,c,d). is_surface_flip gates the accept-case case-forcing
        // and the extra retag/envelope handling in the swap *_after callbacks.
        bool is_surface_flip = false;
        size_t sf_a = 0, sf_b = 0, sf_c = 0, sf_d = 0;
        FaceAttributes sf_face_attr;
    };
    wmtk::threading::enumerable_thread_specific<SwapInfoCache> swap_cache;


    // for incremental tetwild
public:
    /**
     * @brief Conformally insert the input surface into a background tet mesh,
     * via the exact arrangement (vol_rem::embed_tri_in_poly_mesh).
     *
     * This is the insertion path. See the banner in VolumemesherInsertion.cpp.
     */
    void insertion_by_volumeremesher(
        const std::vector<Vector3d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces,
        std::vector<Vector3r>& v_rational,
        std::vector<std::array<size_t, 3>>& facets_after,
        std::vector<bool>& is_v_on_input,
        std::vector<std::array<size_t, 4>>& tets_after,
        std::vector<bool>& tet_face_on_input_surface);

    void init_from_Volumeremesher(
        const std::vector<Vector3r>& v_rational,
        const std::vector<std::array<size_t, 3>>& facets,
        const std::vector<bool>& is_v_on_input,
        const std::vector<std::array<size_t, 4>>& tets,
        const std::vector<bool>& tet_face_on_input_surface);

    void init_from_file(std::string input_dir);


    /**
     * @brief Escape a stuck max energy by refining the sizing field around the
     * worst elements.
     *
     * Finds the m_params.stuck_refine_num_worst tets with the highest energy,
     * gathers all vertices within m_params.stuck_refine_rings graph rings of
     * them, and multiplies each such vertex's m_sizing_scalar by
     * m_params.stuck_refine_factor (clamped at m_params.stuck_refine_min_scalar).
     * Then runs gradation_smooth_sizing so the refined region blends smoothly
     * into the surrounding resolution. Replaces the old global
     * adjust_sizing_field mechanism. Returns the number of vertices refined.
     */
    size_t refine_sizing_around_worst(double max_energy);

    /// Per-pass claim for the high-valence split gate: one slot per vertex, reset at the
    /// start of every split pass. A high-valence vertex accepts the first
    /// valence-increasing split of the pass and refuses the rest, so refinement spreads
    /// instead of piling onto the same vertex. Atomic because splits run in parallel; a
    /// plain array of unique_ptr rather than a vector because std::atomic is not movable.
    std::unique_ptr<std::atomic<int>[]> m_high_valence_claim;
    size_t m_high_valence_claim_size = 0;
    /// Splits refused by that gate in the current pass, reported once per pass.
    std::atomic<size_t> m_high_valence_rejects = 0;

    /// True iff edge (v1,v2) is a worst tet's longest edge queued for force-split.
    bool is_force_split_edge(size_t v1, size_t v2) const
    {
        return m_force_split_edges.find(simplex::Edge(v1, v2)) != m_force_split_edges.end();
    }

    // for open boundary
    void find_open_boundary();
    bool is_open_boundary_edge(const Tuple& e);
    bool is_open_boundary_edge(const std::array<size_t, 2>& e);

public:
    // substructure functions

    /**
     * @brief Compute the vertex order for every vertex.
     */
    void init_vertex_order();

public:
    // for boolean operations
    int flood_fill();

    void save_paraview(const std::string& path, const bool use_hdf5);

    // initialize sizing field (for topology preservation)
    void init_sizing_field();

public:
    struct ExportStruct
    {
        // tet mesh
        MatrixXd V;
        MatrixXi T;
        // tracked surface
        MatrixXi F;
        // attributes
        VectorXd t_amips;
        VectorXd t_winding_number_input;
        VectorXd t_winding_number_tracked;
        MatrixXd t_winding_number_per_input;
        VectorXi t_part;
    };
    // export functionality
    ExportStruct export_mesh_data() const;
};


} // namespace wmtk::components::tetwild
