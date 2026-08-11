#pragma once

#include <igl/Timer.h>
#include <wmtk/SurfaceTagAttributes.h>
#include <wmtk/TetMesh.h>
#include <wmtk/TetOptimizerMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/threading/parallel_for.hpp>

#include "ConnectedComponent.hpp"
#include "Parameters.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <VolumeRemesher/embed.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <igl/remove_unreferenced.h>
#include <atomic>
#include <memory>

#include <wmtk/utils/SurfaceTopology.hpp>
#include <wmtk/utils/partition_utils.hpp>
#include "expression_parser/Expression.hpp"

namespace wmtk::components::simwild {

/// The shared attribute types live on the base; keep the unqualified names working. The
/// simwild copies were identical apart from tetwild's m_is_on_open_boundary, which simwild
/// never sets.
using VertexAttributes = wmtk::TetOptimizerMesh::VertexAttributes;
using FaceAttributes = wmtk::TetOptimizerMesh::FaceAttributes;

class TetAttributes
{
public:
    /**
     * cubed (!) AMIPS quality
     */
    double m_quality;
    /**
     * All image labels. Stored as pairs of image ID and the tag within the image. Using a sparse
     * vector, so 0 entries are ommitted.
     */
    CellTag tags;
};

class SimWildMesh : public wmtk::TetOptimizerMesh
{
public:
    using ExprPtr = expression_parser::ExpressionPtr;

    int m_debug_print_counter = 0;
    size_t m_tags_count = 0;
    std::map<int64_t, std::string> m_tag_id_to_name;
    std::map<std::string, int64_t> m_tag_name_to_id;

    /// The base holds only wmtk::OptimizerParameters; this is the same object, typed, for the
    /// simwild-only fields (tags, quality/sizing fields, the operation name).
    Parameters& m_sim_params;
    std::vector<Vector3d> m_V_envelope;
    std::vector<Vector3i> m_F_envelope;
    std::shared_ptr<SampleEnvelope> m_envelope_orig;
    double m_envelope_eps = -1;

    std::vector<std::tuple<ExprPtr, double>> m_sizing_field;
    std::vector<std::tuple<ExprPtr, double>> m_quality_field;

    bool m_collapse_check_quality = true;

    // for open boundary
    /// Follows m_envelope's use_exact; see where it is built in VolumemesherInsertion.cpp.
    std::shared_ptr<SampleEnvelope> m_order_2_edge_envelope;

    /// Envelope a vertex is pulled toward while smoothing.
    std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(const size_t vid) const;

    /// No 0-dimensional features here, so smoothing is never positionally constrained beyond
    /// the envelope. See TriWildMesh::smoothing_position_is_allowed for the case that is.
    bool smoothing_position_is_allowed(const size_t, const Vector2d&) const { return true; }

    // When set, split_edge_after binary-searches vmid onto the zero-crossing of this function.
    // Negative = stays on v1 side, positive = stays on v2 side.
    std::function<double(const Vector3d&)> m_voronoi_split_fn = nullptr;

    SimWildMesh(Parameters& _m_params, double envelope_eps, int _num_threads = 0)
        : wmtk::TetOptimizerMesh(_m_params, nullptr)
        , m_sim_params(_m_params)
    {
        m_envelope_eps = envelope_eps;
        NUM_THREADS = _num_threads;
        p_tet_attrs = &m_tet_attribute;
        m_collapse_check_link_condition = false;
        m_collapse_check_manifold = false;

        // solver is lazily created on first use

        optimization::deactivate_opt_logger();

        m_s_envelope = 1. / (m_params.diag_l * m_params.eps * m_params.eps);

        double& wa = m_params.w_amips;
        double& we = m_params.w_envelope;
        we = 1 - wa;
        logger().info("w_envelope = {}", we);
    }

    ~SimWildMesh() {}
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    TetAttCol m_tet_attribute;

    double cell_quality(const size_t tid) const override { return m_tet_attribute[tid].m_quality; }
    void set_cell_quality(const size_t tid, const double q) override
    {
        m_tet_attribute[tid].m_quality = q;
    }

    // only used with unit tests
    void create_mesh_attributes(
        const std::vector<VertexAttributes>& _vertex_attribute,
        const std::vector<TetAttributes>& _tet_attribute)
    {
        const size_t n_tet = _tet_attribute.size();
        m_vertex_attribute.resize(_vertex_attribute.size());
        m_face_attribute.resize(4 * n_tet);
        m_tet_attribute.resize(n_tet);

        for (size_t i = 0; i < _vertex_attribute.size(); i++)
            m_vertex_attribute[i] = _vertex_attribute[i];

        // Keep whatever init() reserved. AttributeCollection::resize only ever grows, so the
        // calls above cannot shrink a collection -- but assigning m_attributes directly does,
        // and it used to drop the tet attributes to exactly n_tet. The attributes have to stay
        // at least as large as the connectivity: an operation that creates a new element
        // indexes them by its id, and a 5->6 swap creates one. tetwild hit the same thing (see
        // the note on its copy); there the overrun wrote a double past the end and libc++ let
        // it pass, here it assigns a std::set and segfaults outright.
        const size_t tcap = std::max(n_tet, m_tet_attribute.size());
        m_tet_attribute.m_attributes = std::vector<TetAttributes>(tcap);
        for (size_t i = 0; i < n_tet; i++) m_tet_attribute[i] = _tet_attribute[i];
        for (size_t i = 0; i < n_tet; i++)
            m_tet_attribute[i].m_quality = get_quality(tuple_from_tet(i));
    }

    // TODO This should not be here but inside wmtk
    // TODO This should not be here but inside wmtk
    void init_envelope(const MatrixXd& V, const MatrixXi& F, const bool use_exact);

    CellTag string_set_to_cell_tag(const std::set<std::string>& str_set);

    void set_sizing_field(const nlohmann::json& sizing_field_json);

    void set_quality_field(const nlohmann::json& quality_field_json);

    double target_quality(const size_t tid) const;
    double target_quality(const Tuple& t) const;
    double quality_rel(const size_t tid) const;
    double quality_rel(const Tuple& t) const;
    bool check_mesh_quality(double& max_rel_quality, const bool verbose = false) const;


    ////// Attributes related

    void write_msh(std::string file, const bool write_envelope = true);

public:
    void split_all_edges();
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& loc) override;

    void smooth_all_vertices(const size_t n_iters);
    bool smooth_after(const Tuple& t) override;

    void collapse_all_edges(bool is_limit_length = true);
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    void simplify();

    size_t swap_all_edges_44();
    bool swap_edge_44_before(const Tuple& t) override;
    bool swap_edge_44_after(const Tuple& t) override;
    /// Steers the 4-4 swap to the diagonal that realizes a surface flip. See
    /// prepare_surface_flip; identical to tetwild's.
    bool swap_edge_44_accept_case(const std::array<size_t, 2>& new_edge) override;

    size_t swap_all_edges_56();
    bool swap_edge_56_before(const Tuple& t) override;
    bool swap_edge_56_after(const Tuple& t) override;
    /// Steers the 5-6 swap to the fan that realizes a surface flip. See prepare_surface_flip;
    /// identical to tetwild's.
    bool swap_edge_56_accept_case(const std::array<size_t, 3>& new_face) override;

    size_t swap_all_edges_32();
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    /**
     * @brief Prepare a surface edge swap (a surface diagonal flip).
     *
     * Called from swap_edge_before / swap_edge_44_before / swap_edge_56_before when the swapped
     * edge (a,b) is on the surface. Verifies the local guards that guarantee the flip preserves
     * surface manifoldness / topology, and fills the surface-flip fields of swap_cache. Returns
     * false (rejecting the swap) if any guard fails: non-manifold edge (!= 2 surface faces), a
     * surface face already incident to the new edge (c,d), or one of the two would-be new
     * surface faces already tagged surface. The tets sharing (a,b) are passed in to avoid
     * recomputation.
     *
     * This is tetwild's, generalized to any ring size (3->2, 4-4, 5-6); the specific
     * retetrahedralization that realizes the flip is picked by the accept_case hooks above.
     * On top of tetwild's it records which tag each side of the interface carries -- see
     * SwapInfoCache::ring_tags.
     */
    bool prepare_surface_flip(const Tuple& t, const std::vector<size_t>& incident_tets);

    /**
     * @brief Record the one tag every tet this swap produces must carry, for an interior swap.
     *
     * A face is a surface face exactly when it separates differently tagged tets, so a swap
     * with no incident surface face acts entirely inside one tagged region and there is nothing
     * to decide. Returns false if `tids` disagree anyway -- that means the tag/surface invariant
     * is already broken here, and re-tagging would silently move tagged volume.
     */
    bool cache_interior_swap_tag(const std::vector<size_t>& tids);

    /**
     * @brief Give every tet the swap just created its tag.
     *
     * Interior swap: the single tag cache_interior_swap_tag recorded. Surface flip: the tag of
     * the side of the interface the tet ended up on, read off the ring vertices it contains
     * (see SwapInfoCache::ring_tags). Returns false if any new tet cannot be assigned a tag,
     * which rejects the swap and rolls the writes back.
     */
    bool propagate_swap_tags(const std::vector<size_t>& tids);
    bool propagate_swap_tags(const std::vector<Tuple>& tets);

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
     * error if it changed. Used (when m_sim_params.check_surface_topology is set) to
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


    /**
     * @brief Check if all vertices of the mesh are rounded.
     *
     */
    bool all_rounded() const;

    //
    //
    void mesh_improvement(int max_its = 80);
    double local_operations(const std::array<int, 4>& ops, bool collapse_limit_length = true);
    // debug use
    std::atomic<int> cnt_split = 0, cnt_collapse = 0, cnt_swap = 0;
    // Successful surface diagonal flips (subset of cnt_swap). Diagnostic.
    // cnt_surface_swap is the grand total; the per-type counters break it down by the swap that
    // realized the flip.
    std::atomic<int> cnt_surface_swap = 0;
    std::atomic<int> cnt_surface_swap_32 = 0, cnt_surface_swap_44 = 0, cnt_surface_swap_56 = 0;

private:
    ////// Operations

    struct SplitInfoCache
    {
        //        VertexAttributes vertex_info;
        size_t v_new;
        size_t v1_id;
        size_t v2_id;
        bool is_edge_on_surface = false;
        bool is_edge_open_boundary = false;
        /// Worst quality among the elements incident to the edge BEFORE the split, so
        /// split_edge_after can tell "this split created a degenerate element" from "this
        /// split subdivided a region that was already degenerate".
        double max_quality_before = 0.;

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;

        /**
         * All tets incident to the splitted edge, identified by the link edge (the edge opposite to
         * the splitted one).
         */
        std::map<simplex::Edge, TetAttributes> tets;
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

        /// The tag every new tet takes, for an interior swap. See cache_interior_swap_tag.
        CellTag tet_tags;
        /**
         * The tag on each side of the interface, for a surface flip, keyed by a ring vertex
         * that identifies the side. Filled by prepare_surface_flip, read by
         * propagate_swap_tags. c and d are deliberately absent: they sit ON the interface and
         * so belong to both sides.
         */
        std::map<size_t, CellTag> ring_tags;

        // Surface diagonal-flip bookkeeping (filled by prepare_surface_flip from
        // swap_edge_before / swap_edge_44_before / swap_edge_56_before when the swapped edge
        // (a,b) lies on the surface). a,b are the removed-edge endpoints, c,d are the new
        // surface-edge endpoints (the apexes of the two incident surface faces). sf_face_attr
        // is copied onto the two new surface faces (a,c,d),(b,c,d). is_surface_flip gates the
        // accept-case case-forcing and the extra retag/envelope handling in the swap *_after
        // callbacks.
        bool is_surface_flip = false;
        size_t sf_a = 0, sf_b = 0, sf_c = 0, sf_d = 0;
        FaceAttributes sf_face_attr;
    };
    wmtk::threading::enumerable_thread_specific<SwapInfoCache> swap_cache;

public:
    /**
     * @brief Init from meshes image.
     *
     * @param V #Vx3 vertices of the tet mesh
     * @param T #Tx4 vertex IDs for all tets
     * @param T_tags #Tx1 image data represented by the individual tets
     */
    void init_from_image(
        const MatrixXr& V,
        const MatrixXi& T,
        const MatrixSi& T_tags,
        const std::vector<std::string>& tag_names);
    void init_from_image(
        const MatrixXd& V,
        const MatrixXi& T,
        const MatrixSi& T_tags,
        const std::vector<std::string>& tag_names);

    void init_surfaces_and_boundaries();


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
    size_t refine_sizing_around_worst();


    /**
     * @brief Splits in the last pass that fell back to the exact rational midpoint.
     *
     * A split is the only operation that can un-round a vertex, so this is exactly how often
     * the mesh acquired exact coordinates during optimization -- the counterpart of the
     * "rounded n/m" line, which says how many it still carries. Non-zero on real inputs: on
     * the challenging-model set it ranges from 1 to 44 per run.
     */
    size_t m_exact_split_count = 0;

    /// True iff edge (v1,v2) is a worst tet's longest edge queued for force-split.
    bool is_force_split_edge(size_t v1, size_t v2) const
    {
        return m_force_split_edges.find(simplex::Edge(v1, v2)) != m_force_split_edges.end();
    }

    /**
     * @brief Find open boundary edges of the embedded surface and initialize a BVH for the open
     * boundary.
     *
     * The envelope for the open boundary uses a hack: A boundary edge is represented as a
     * degenerate triangle, e.g., (v0,v1,v0). That way, the standard triangle envelope code can be
     * used.
     *
     */
    void find_order_2_edges();
    /**
     * @brief Checks if an edge COULD be an open boundary edge.
     *
     * The method performs two checks. First, it checks if the two vertices are marked as on the
     * open boundary. Second, it checks if the edge is within the open boundary envelope.
     * Note that these checks are not sufficient to guarantee that an edge is actually on the open
     * boundary! For example, an almost degenerate triangle with two edges on the open boundary
     * could cause a false positive result.
     */
    bool is_order_2_edge(const Tuple& e) const;
    bool is_order_2_edge(const std::array<size_t, 2>& e) const;

    void write_vtu(const std::string& path);

    void write_surface(const std::string& path) const;

public:
    // substructure functions

    /**
     * @brief Compute the vertex order for every vertex.
     */
    void init_vertex_order();

public:
    // Annotations

    double tet_volume(const size_t tid) const;

    /**
     * @brief Find all connected components that contain the `tag_in` tags.
     */
    std::vector<ConnectedComponent> compute_connected_components(const CellTag& tag_in) const;
    std::vector<ConnectedComponent> compute_connected_components(const ExprPtr& expr) const;

    /**
     * @brief Find all regions that do not contain the tags from `tag_in`.
     *
     * The `tag_in` vector represents a list of tag intersections.
     * Example: tag_in = {{1,2},{3}}
     * A face will be considered as a hole if its tags neither include {1,2} or {3}.
     * The following would be holes:
     * {}
     * {1,4}
     * The following would be NOT holes:
     * {1,2,4} <- contains 1 and 2
     * {3} <- contains 3
     * {1,3} <- contains 3
     *
     * The returned vector also contains "holes" that touch the boundary. They should be
     * ommitted in hole filling.
     */
    std::vector<ConnectedComponent> find_holes(const std::vector<CellTag>& tag_in) const;

    /**
     * @brief Compute the boundary of a tag.
     *
     * @param tag A set of tags that must be present in a tet for being considered as
     * tagged.
     * @param V Vertices of the tag boundary.
     * @param F Faces of the tag boundary.
     */
    void compute_tag_boundary(const CellTag& tag, MatrixXd& V, MatrixXi& F) const;

    /**
     * @brief Keep only the largest connected component for each of the distinct tag_0 values,
     * and engulf all other components.
     *
     * @param lcc_tags
     * @param n_lcc The number of largest components that should be kept.
     */
    void keep_largest_connected_component(
        const std::vector<CellTag>& lcc_tags,
        const size_t n_lcc = 1);

    void fill_holes_topo(
        const std::vector<CellTag>& fill_holes_tags,
        double threshold = std::numeric_limits<double>::infinity());

    void seal_connected_components(
        const std::vector<CellTag>& tag_sets,
        const std::vector<ConnectedComponent>& components);

    void tight_seal_topo(
        const std::vector<std::vector<CellTag>>& tight_seal_tag_sets,
        double threshold = std::numeric_limits<double>::infinity());

    void resolve_overlaps(const std::vector<std::array<ExprPtr, 2>>& intersecting_tags);

    void replace_tags(const std::vector<CellTag>& tags_in, const std::vector<CellTag>& tags_out);

    /**
     * @brief Gives tags priority over others.
     *
     * If a tet has multiple tags, only the one with the
     * highest priority will be kept. The priority is determined by the order of the tags in the
     * input vector, e.g., if tag A is before tag B in the vector, then A has higher priority than
     * B.
     *
     * @param tags A vector of tags, where the order determines the priority.
     */
    void tag_priority(const std::vector<int64_t>& tags);
};

} // namespace wmtk::components::simwild
