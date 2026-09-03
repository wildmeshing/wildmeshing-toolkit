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
#include <wmtk/utils/EmbedTriangles.hpp>
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
        /// Whether the vertex lies on an input feature curve (see EdgeAttributes).
        bool m_is_on_feature_curve = false;
        /**
         * Index into TetWildMesh::m_feature_points, or NO_FEATURE. The port of
         * TriWildMesh::VertexExtras::m_feature_id, and for the same reason: preserving a
         * 0-dimensional feature -- an input point, or a feature curve's endpoint -- is a
         * COVERAGE property, and a containment test passes trivially when one anchor
         * vertex is collapsed onto another while the first quietly stops being
         * represented. See the discussion there.
         */
        size_t m_feature_point_id = std::numeric_limits<size_t>::max();
    };
    wmtk::AttributeCollection<VertexExtras> m_vertex_extra;


    /// A vertex that stands for no input feature point. See VertexExtras::m_feature_point_id.
    static constexpr size_t NO_FEATURE = std::numeric_limits<size_t>::max();

    /**
     * Anchor positions of the 0-dimensional features, indexed by
     * VertexExtras::m_feature_point_id: the input feature points, and the feature-curve
     * endpoints (junctions too, when allow_junction_cleanup is off). A vertex carrying
     * feature f may never end up further than m_feature_eps from m_feature_points[f]; the
     * anchor never moves, so the id is a direct lookup.
     */
    std::vector<Vector3d> m_feature_points;
    /// Radius of the anchor ball. Set by the driver alongside m_feature_envelope; kept a
    /// separate name from the tube eps for the same reason triwild keeps m_feature_eps
    /// separate from m_envelope_eps.
    double m_feature_eps = -1;

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
    bool allow_surface_swap() const override { return m_tet_params.allow_surface_swap; }
    bool check_surface_topology() const override { return m_tet_params.check_surface_topology; }
    void split_after_vertex(const size_t vid, const bool is_open_boundary) override
    {
        m_vertex_extra[vid].m_is_on_open_boundary = is_open_boundary;
    }
    bool boundary_edges_at_vertex_inside(size_t vid) const override
    {
        if (!m_vertex_extra[vid].m_is_on_open_boundary || !m_order2_envelope) {
            return true;
        }
        for (const size_t u : get_one_ring_vids_for_vertex(vid)) {
            if (!m_vertex_extra[u].m_is_on_open_boundary) {
                continue;
            }
            // Exact membership: an open boundary edge has exactly one incident tracked
            // face. Flags alone would also catch interior chords between two boundary
            // vertices (e.g. across a corner), which must not be tube-tested.
            if (get_num_surface_faces_for_edge({{vid, u}}) != 1) {
                continue;
            }
            if (m_order2_envelope->is_outside(
                    std::array<Eigen::Vector3d, 2>{
                        {m_vertex_attribute[vid].m_posf, m_vertex_attribute[u].m_posf}})) {
                return false;
            }
        }
        return true;
    }

    bool collapse_before_vertex(size_t v1, size_t v2, double edge_length) override
    {
        if (collapse_breaks_feature_point(v1, v2)) return false;
        if (edge_length <= 0 || !m_vertex_extra[v1].m_is_on_open_boundary) return true;
        return m_vertex_extra[v2].m_is_on_open_boundary ||
               !m_order2_envelope->is_outside(m_vertex_attribute[v2].m_posf);
    }

    /**
     * True iff collapsing v1 into v2 would drop or displace a feature-point anchor.
     * Port of TriWildMesh::collapse_breaks_feature, distance test included: two anchors
     * within m_feature_eps of each other are allowed to merge -- the survivor covers both
     * -- and forbidding that deadlocks the mesh (measured in 2D, see the triwild comment).
     */
    bool collapse_breaks_feature_point(const size_t v1_id, const size_t v2_id) const
    {
        if (!m_tet_params.preserve_feature_points) return false;
        const size_t f1 = m_vertex_extra[v1_id].m_feature_point_id;
        if (f1 == NO_FEATURE) return false;
        return (m_vertex_attribute[v2_id].m_posf - m_feature_points[f1]).squaredNorm() >
               m_feature_eps * m_feature_eps;
    }

    bool smoothing_position_is_allowed(const size_t vid, const Vector3d& p) const override
    {
        const size_t f = m_vertex_extra[vid].m_feature_point_id;
        if (f == NO_FEATURE || !m_tet_params.preserve_feature_points) return true;
        // A ball, not a pin -- the vertex may move anywhere within m_feature_eps of its
        // anchor, exactly the freedom the envelope gives everywhere else.
        return (p - m_feature_points[f]).squaredNorm() <= m_feature_eps * m_feature_eps;
    }

    /// {anchors still represented within m_feature_eps, total}. Geometric, like triwild's:
    /// "some vertex is within eps of this anchor", whether or not it carries the id.
    std::pair<size_t, size_t> feature_retention(double* worst_ratio = nullptr) const;
    bool collapse_is_order_2_edge(const std::array<size_t, 2>& e) override
    {
        return is_open_boundary_edge(e);
    }
    bool collapse_after_connectivity(
        size_t v1,
        size_t v2,
        const std::vector<std::array<size_t, 2>>&) override
    {
        m_vertex_extra[v2].m_is_on_open_boundary =
            m_vertex_extra[v1].m_is_on_open_boundary || m_vertex_extra[v2].m_is_on_open_boundary;
        m_vertex_extra[v2].m_is_on_feature_curve =
            m_vertex_extra[v1].m_is_on_feature_curve || m_vertex_extra[v2].m_is_on_feature_curve;
        return true;
    }
    void collapse_after_vertex(size_t v1, size_t v2) override
    {
        // The survivor picks up the anchor the collapsed vertex carried, if it has none.
        if (m_vertex_extra[v2].m_feature_point_id == NO_FEATURE) {
            m_vertex_extra[v2].m_feature_point_id = m_vertex_extra[v1].m_feature_point_id;
        }
        if (m_vertex_extra[v2].m_is_on_open_boundary && !is_vertex_on_boundary(v2)) {
            m_vertex_extra[v2].m_is_on_open_boundary = false;
        }
        // Same cleanup for the feature flag: v1's flag was OR-ed in (see
        // collapse_after_connectivity), but if the survivor has no incident tagged edge the
        // curve rerouted around it and the flag is stale.
        if (m_track_feature_edges && m_vertex_extra[v2].m_is_on_feature_curve &&
            !vertex_has_feature_edge(v2)) {
            m_vertex_extra[v2].m_is_on_feature_curve = false;
        }
    }

    void split_after_vertex_feature(size_t vid, bool on_feature) override
    {
        m_vertex_extra[vid].m_is_on_feature_curve = on_feature;
        // A fresh split vertex stands for no anchor; its slot may be reused and stale.
        m_vertex_extra[vid].m_feature_point_id = NO_FEATURE;
    }


    /// Envelope a vertex is pulled toward while smoothing.
    std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(const size_t vid) const override;
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
        m_s_envelope = 1. / (m_params.eps * m_params.eps);
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
    //
    /**
     * brief Check if the vertex has an incident boundary edge.
     * This performs a topological check.
     */
    bool is_vertex_on_boundary(const size_t vid);
    //
    /**
     * @brief Call the original TetWild code.
     */
    void mesh_improvement_legacy(int max_its = 80);

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

    /**
     * The 'hybrid' filter: keep only tets inside (winding > 0.5) at least one of the
     * inputs listed in `volume_inputs` (indices into the input list, matching
     * m_winding_number_per_input). Everything else is scaffolding; the driver collects
     * the surface-role faces and the feature curves BEFORE calling this, because they
     * live on the tets this deletes.
     */
    void filter_with_roles(const std::vector<size_t>& volume_inputs);

    /**
     * Write the mixed-dimensional output of the hybrid filter: the (already filtered)
     * tet mesh, plus the collected surface-role triangles as a face block and the
     * feature curves as an edge block and the anchor points as 0D point elements.
     */
    void output_hybrid_mesh(
        const std::string& file,
        const std::vector<std::array<Vector3d, 3>>& sheet_triangles,
        const std::vector<std::array<Vector3d, 2>>& curve_segments,
        const std::vector<Vector3d>& anchor_points);


    // debug use
    std::atomic<int> cnt_split = 0, cnt_collapse = 0;

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

    // for incremental tetwild
public:
    /**
     * @brief Conformally insert the input surface into a background tet mesh,
     * via the exact arrangement (vol_rem::embed_tri_in_poly_mesh).
     *
     * This is the insertion path. See the banner in VolumemesherInsertion.cpp.
     */
    /**
     * Optional feature inputs: `feature_edge_vertices`/`feature_edges` is an edge mesh to
     * force into the tetrahedralization as tet edges, `feature_points` are points to force
     * in as tet vertices. Where they ended up comes back in `features_out` (compacted
     * output ids, see utils::EmbedFeaturesResult), which is required when features are
     * passed. Features and their forcing-triangle apexes must lie inside the background
     * box; the caller grows the input bounding box over them (tetwild.cpp does).
     */
    void insertion_by_volumeremesher(
        const std::vector<Vector3d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces,
        std::vector<Vector3r>& v_rational,
        std::vector<std::array<size_t, 3>>& facets_after,
        std::vector<bool>& is_v_on_input,
        std::vector<std::array<size_t, 4>>& tets_after,
        std::vector<bool>& tet_face_on_input_surface,
        const std::vector<Vector3d>& feature_edge_vertices = {},
        const std::vector<std::array<size_t, 2>>& feature_edges = {},
        const std::vector<Vector3d>& feature_points = {},
        utils::EmbedFeaturesResult* features_out = nullptr);

    /**
     * `features` tags the feature-edge tilings on the initial mesh; passing it registers
     * m_edge_attribute with p_edge_attrs. Null => no feature tracking, nothing allocated.
     */
    void init_from_Volumeremesher(
        const std::vector<Vector3r>& v_rational,
        const std::vector<std::array<size_t, 3>>& facets,
        const std::vector<bool>& is_v_on_input,
        const std::vector<std::array<size_t, 4>>& tets,
        const std::vector<bool>& tet_face_on_input_surface,
        const utils::EmbedFeaturesResult* features = nullptr,
        const std::vector<Vector3d>* feature_anchors = nullptr);

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
    size_t refine_sizing_around_worst(double max_energy) override;

    // for open boundary
    void find_open_boundary();
    /// tetwild's surface is the input mesh, which may be non-watertight, so the base's
    /// "no open boundary" default does not hold here. See TetOptimizerMesh.
    bool is_open_boundary_edge(const Tuple& e) override;
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
    void write_optimization_debug_output(const std::string& path) override
    {
        save_paraview(path, false);
    }
    void optimization_sanity_checks_extra() override;

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
