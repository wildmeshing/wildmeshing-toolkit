#pragma once
#include <wmtk/TriMesh.h>
#include <wmtk/TriOptimizerMesh.h>
#include <algorithm>
#include <set>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include "Parameters.h"
#include "SimplicialComplexBVH.hpp"

using CellTag = std::set<int64_t>;


namespace wmtk::components::topological_offset {


const int64_t TEMP_OFFSET_TRI_TAG = -1;
const CellTag TEMP_OFFSET_TRI_TAG_SET{TEMP_OFFSET_TRI_TAG};

/**
 * @brief Per-vertex data the shared 2D optimizer knows nothing about.
 *
 * Position, rounding, bbox membership, sizing and partition all live on
 * wmtk::TriOptimizerMesh::VertexAttributes. m_is_on_input / m_is_on_offset say WHICH of the two
 * tracked surfaces a vertex belongs to; the base's m_is_on_surface is their union.
 */
class VertexExtra2d
{
public:
    int label = 0;
    bool m_is_on_input = false; // on the input complex
    bool m_is_on_offset = false; // on the offset boundary
};


/// Per-edge construction label; the surface tags themselves are the base's
/// wmtk::SurfaceTagAttributes. Registered with m_edge_attr_group.
class EdgeExtra2d
{
public:
    int label = 0;
};


/// Per-face construction label. The region tag lives in the base's FaceAttributes::tags.
/// Registered with m_face_attr_group.
class FaceExtra2d
{
public:
    int label = 0;
};


/**
 * @brief The offset's 2D mesh, on the shared 2D optimizer.
 *
 * Mirrors TopoOffsetTetMesh: the construction phase is entirely its own, and the optimization
 * phase that follows is wmtk::TriOptimizerMesh's.
 *
 * Two surfaces are tracked. The input complex is class 0 and must stay inside m_envelope; the
 * offset boundary is OFFSET_SURFACE_CLASS and is free to move. In 2D the offset boundary is
 * exactly the set of edges across which the incident FACE LABELS differ -- there is no stored
 * definition of it, it falls out of the labelling, which is why label_offset_boundary()
 * recomputes it once at the top of the optimization.
 */
class TopoOffsetTriMesh : public wmtk::TriOptimizerMesh
{
public: // mode for splitting in marching tets
    enum class EdgeSplitMode {
        Midpoint = 0, // used for simplicial embedding steps
        Initial = 1, // this is used to initialize the complex. Its a little hacky

        // only one of these is used, hard coded in execute_offset()
        BinarySearch = 2, // bisection root finding algo
        LogRootFind = 3, // 'custom' root finding, using the fact that d(x) - d* < 0 at first vertex
        SphereTracing = 4, // use sphere tracing to compute the zero of the distance field

        Optimization = 5 // the optimization phase; the shared engine places the vertex
    };

public:
    int m_vtu_counter = 0;
    std::array<size_t, 3> m_init_counts = {{0, 0, 0}};
    size_t m_tags_count;
    SimplicialComplexBVH m_input_complex_bvh;
    EdgeSplitMode m_edge_split_mode = EdgeSplitMode::Midpoint;

    // tag name maps
    std::map<std::string, int64_t> m_tag_name_to_id;
    std::map<int64_t, std::string> m_tag_id_to_name;
    CellTag m_offset_output_tag_ids;

    // if in 'singlebody' mode
    bool m_singlebody = false;
    int64_t m_single_tag;

    // just for retaining in output. dont actually use
    bool m_has_envelope = false;
    MatrixXd m_V_envelope;
    MatrixXi m_F_envelope;

    /// SurfaceTagAttributes::m_surface_class for the offset boundary; the input complex keeps
    /// the primary class 0 and is the only one checked against the envelope.
    static constexpr int OFFSET_SURFACE_CLASS = 1;

    /// The base holds only wmtk::OptimizerParameters; this is the same object, typed.
    Parameters& m_offset_params;

    using VertexExtraCol = wmtk::AttributeCollection<VertexExtra2d>;
    using EdgeExtraCol = wmtk::AttributeCollection<EdgeExtra2d>;
    using FaceExtraCol = wmtk::AttributeCollection<FaceExtra2d>;
    // m_vertex_attribute, m_edge_attribute and m_face_attribute are the base's; these three
    // are registered alongside them in its attribute groups.
    VertexExtraCol m_vertex_extra;
    EdgeExtraCol m_edge_extra;
    FaceExtraCol m_face_extra;

    TopoOffsetTriMesh(Parameters& _m_offset_params, int _num_threads = 0)
        : wmtk::TriOptimizerMesh(_m_offset_params)
        , m_offset_params(_m_offset_params)
    {
        NUM_THREADS = _num_threads;
        m_vertex_attr_group.add(&m_vertex_extra);
        m_edge_attr_group.add(&m_edge_extra);
        m_face_attr_group.add(&m_face_extra);
    }

    ~TopoOffsetTriMesh() override = default;

    /**
     * @brief Place a vertex, keeping its exact and rounded coordinates in step.
     *
     * As in 3D: the offset works in doubles (its construction is driven by a BVH distance
     * field), so every vertex it places is rounded, but m_pos must still be filled because the
     * shared split's exact-midpoint fallback reads it.
     */
    void set_vertex_position(const size_t vid, const Vector2d& p)
    {
        m_vertex_attribute[vid].m_posf = p;
        m_vertex_attribute[vid].m_pos = to_rational(p);
        m_vertex_attribute[vid].m_is_rounded = true;
    }

    /// Whether edge `eid` is on the offset boundary / carries input geometry.
    bool edge_is_offset(const size_t eid) const
    {
        return m_edge_attribute[eid].m_is_surface_fs &&
               m_edge_attribute[eid].m_surface_class == OFFSET_SURFACE_CLASS;
    }
    /**
     * @brief An edge's / face's shared attributes together with the offset's own label.
     *
     * The marching-triangles splits snapshot a simplex and write it back onto the pieces it
     * became, and both halves have to travel together.
     */
    struct EdgeSnapshot2d
    {
        EdgeAttributes tags;
        EdgeExtra2d extra;
    };
    struct FaceSnapshot2d
    {
        FaceAttributes attrs;
        FaceExtra2d extra;
    };
    EdgeSnapshot2d edge_snapshot(const size_t eid) const
    {
        return EdgeSnapshot2d{m_edge_attribute[eid], m_edge_extra[eid]};
    }
    void restore_edge(const size_t eid, const EdgeSnapshot2d& s)
    {
        m_edge_attribute[eid] = s.tags;
        m_edge_extra[eid] = s.extra;
    }
    FaceSnapshot2d face_snapshot(const size_t fid) const
    {
        return FaceSnapshot2d{m_face_attribute[fid], m_face_extra[fid]};
    }
    void restore_face(const size_t fid, const FaceSnapshot2d& s)
    {
        m_face_attribute[fid] = s.attrs;
        m_face_extra[fid] = s.extra;
    }

    /**
     * @brief Tag the two tracked surfaces for the optimization phase.
     *
     * The offset boundary in 2D has no stored definition -- it is exactly the set of edges
     * across which the incident FACE LABELS differ, so it falls out of the labelling and is
     * recomputed here once. An edge with only one incident face is on the domain boundary and
     * is tagged bbox instead, which is what stops the box from collapsing.
     */
    void label_offset_boundary();

    /**
     * @brief Re-derive each face's construction label from its region tags.
     *
     * Tags are the source of truth, as in simwild: every shared operation propagates
     * FaceAttributes -- and therefore the tags -- but none of them knows about the label, which
     * offset_is_manifold() and the output are built from. Rather than teach every operation to
     * carry a second quantity, the label is recomputed from the tags it was originally derived
     * from, so the two cannot drift apart no matter which operation ran.
     */
    /**
     * @brief Whether face `fid` belongs to the closed offset region, read from its TAGS.
     *
     * The region is the offset band plus the input complex it wraps. Both are named by tags --
     * the band by the output tag, the input complex by the user's selection expression -- and
     * tags are what every shared operation propagates. The construction label says the same
     * thing, but it is derived state maintained alongside them, and derived state drifts: two
     * children of one split ended up on opposite sides of it, which is what let a refinement
     * tear the region.
     *
     * Reading the tags directly removes that failure mode rather than guarding against it.
     */
    bool face_in_region(const size_t fid) const;

    /// Whether face `fid` is part of the INPUT complex (as opposed to the offset band), read
    /// from its tags via the user's selection expression -- the same test that defined it.
    bool face_is_input_complex(const size_t fid) const;

    void relabel_faces_from_tags();
    void relabel_face_from_tags(const size_t fid);

    /**
     * @brief The substructure the link condition is evaluated against, DERIVED not cached.
     *
     * substructure_link_condition() asks these, and it is only as good as the answers. The
     * cached edge tags are refreshed once per iteration, which is too coarse: the split pass
     * runs first and creates edges the tagging never classified, so the collapse pass that
     * follows evaluates the condition against a substructure that no longer describes the
     * mesh -- which is exactly why split and collapse tear the region together while each is
     * safe alone.
     *
     * Computing them from the face labels on demand costs a face lookup and cannot go stale.
     */
    bool vertex_is_on_surface(const size_t vid) const override;
    bool edge_is_on_surface(const std::array<size_t, 2>& vids) const override;

    /// Move an offset-boundary vertex back onto the target distance from the input complex,
    /// blended with the Laplacian of its offset-boundary neighbours.
    bool project_offset_vertex(const Tuple& t);

    /// The 2D optimization phase: split / collapse / swap / smooth on the shared driver.
    void optimize_offset(const std::filesystem::path& output_file);

    /// Dispatch: the optimization phase runs the shared split, everything else the
    /// marching-triangles one.
    /// Parent face labels for an optimization split, keyed by the apex vertex opposite the
    /// split edge -- that vertex is shared by both children of the same parent, so it names
    /// them afterwards.
    struct OptSplitCache2d
    {
        std::map<size_t, int> face_label;
    };
    wmtk::threading::enumerable_thread_specific<OptSplitCache2d> m_opt_split_cache;

    bool marching_split_edge_before(const Tuple& t);
    bool marching_split_edge_after(const Tuple& t);

    /**
     * @brief Reject any collapse that violates the substructure link condition.
     *
     * The base applies it only when BOTH endpoints already sit on a tracked surface or the
     * bbox, which is the right rule for tetwild and simwild. It is not enough here: the offset
     * region is a thin band, and a collapse with only one endpoint on the boundary can still
     * pinch the two sides of that band together, which is precisely what makes the region stop
     * being manifold. The offset asks unconditionally.
     */
    bool collapse_edge_before(const Tuple& t) override;

    /**
     * @brief Reject a flip whose new edge already exists.
     *
     * Flipping (a,b) to (c,d) when c and d are already joined creates a second edge between the
     * same pair of vertices. Across a thin offset band that is exactly how the two sides of the
     * band get stitched together, and the region stops being manifold. The base refuses
     * tracked-surface edges but has no reason to check this.
     */
    bool swap_edge_before(const Tuple& t) override;
    bool collapse_before_vertex(size_t v1, size_t v2) override;
    void collapse_after_vertex(size_t v1, size_t v2) override;
    void split_after_vertex(size_t v_new) override;
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    bool edge_is_input(const size_t eid) const
    {
        return m_edge_attribute[eid].m_is_surface_fs &&
               m_edge_attribute[eid].m_surface_class != OFFSET_SURFACE_CLASS;
    }

    ////// wmtk::TriOptimizerMesh hooks

    /// Only the input complex is envelope-constrained; the offset boundary is defined by the
    /// face labels, not by input geometry, so there is nothing for it to stay inside.
    std::shared_ptr<SampleEnvelope> surface_envelope_for_edge(
        const std::array<size_t, 2>& vids) const override
    {
        for (const size_t v : vids) {
            if (!m_vertex_extra[v].m_is_on_input) return nullptr;
        }
        return m_envelope;
    }

    /// An offset-boundary vertex is placed by project_offset_vertex(); the base's smoother is
    /// only allowed to move genuinely interior ones, and the input complex never moves.
    bool smoothing_position_is_allowed(size_t vid, const Vector2d&) const override
    {
        return !m_vertex_extra[vid].m_is_on_input;
    }

    /**
     * @brief Sizing refinement over the offset polyline.
     *
     * The 3D field is driven by the mean ratio of the offset TRIANGULATION; a polyline has no
     * such metric, so this uses the quantity that actually matters in 2D -- how far each
     * offset-boundary vertex sits from the target distance. A vertex whose error exceeds
     * sizing_mrm_threshold of the target gets a shorter target edge length, so the next split
     * pass resolves that stretch more finely.
     */
    size_t refine_sizing_around_worst(double) override { return update_sizing_field(); }
    size_t update_sizing_field();
    void write_smoothing_debug_output(const std::string& path) const override
    {
        const_cast<TopoOffsetTriMesh*>(this)->write_vtu(path);
    }


    /**
     * @brief initialize TriMesh from vertex, face, tag data
     * @param V: #V by 2 vertex matrix
     * @param F: #F by 3 face matrix
     * @param F_tags: #F by #physical groups tag matrix
     * @param V_env: #V_env by 2 EnvelopeSurface vertex matrix
     * @param F_env: #F_env by 2 EnvelopeSurface edge matrix
     */
    void init_from_image(
        const MatrixXd& V,
        const MatrixXi& F,
        const MatrixSi& F_tags,
        const MatrixXd& V_env,
        const MatrixXi& F_env,
        const std::vector<std::string>& tag_names);

    /**
     * @brief ensure ambient tag does not overlap any other tags in mesh.
     */
    bool ambient_assert();

    /**
     * @brief label input complex simplices as per boolean expression (or single body mode)
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
     * @brief split edge at point by minimizing m_offset_params.target_distance - d() (where d() is
     * distance to input complex via BVH) along the edge. Uses binary search, so implicitly assumes
     * distance field is monotonic along edge. May give weird results if not monotonic
     */
    void edge_split_binary_search(const size_t v1, const size_t v2, Vector2d& p_new) const;
    void edge_split_binary_search(const Vector2d& v1_pos, const Vector2d& v2_pos, Vector2d& p_new)
        const;

    /**
     * @deprecated
     * @brief split edge at root of d() - target_distance, using fact that this is negative at
     * v1.
     */
    void edge_split_log_root_find(const size_t v1, const size_t v2, Vector2d& p_new) const;

    /**
     * @brief split edge at first root of d(l) - d*, where d(l) is distance to input complex,
     * using sphere tracing method. This is the best method and should be used instead of
     * binary or log root finding methods
     */
    void edge_split_sphere_tracing(const size_t v1, const size_t v2, Vector2d& p_new) const;

    //// overriden splits/invariants
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    bool invariants(const std::vector<Tuple>& tris) override;
    //// overriden splits/invariants

    /**
     * @brief entry point for offset procedure
     */
    void execute_offset(const std::filesystem::path& output_file);

    /**
     * @brief execute simplistic marching tets. All edges with one vertex labelled 0 and the other 1/2
     * are split. If m_edge_split_mode=BinarySearch, edges are split according to BVH distance field
     * and the offset target distance (m_offset_params.target_distance). If
     * m_edge_split_mode=Midpoint, edges are split at the midpoint
     */
    void marching_tris();


    //// simplicial embedding stuff
    /**
     * @brief check if the input complex (simplices labelled 1) are simplicially embedded w.r.t. the
     * entire mesh
     */
    bool is_simplicially_embedded() const;

    /**
     * @brief check if a triangle satisfies simpicial embedding criteria w.r.t. input complex
     * (simplices labelled 1)
     */
    bool tri_is_simp_emb(const Tuple& t) const;

    /**
     * @brief make mesh a simplicial embedding of the input complex (simplices labelled 1)
     */
    void simplicial_embedding();
    //// simplicial embedding stuff

    // variable offset stuff

    /**
     * @brief check if removing the given face from the given tag set would retain its topology
     */
    bool tag_tri_consistent_topology(size_t f_id, int64_t tag) const;

    /**
     * @brief check if adding a triangle to the offset region does not change the topology of the
     * offset. Returns true if topology would not be changed
     */
    bool offset_tri_consistent_topology(const size_t f_id) const;

    /**
     * @brief check if a triangle is inside the offset (implicitly defined via BVH distance field to
     * input complex) via conservative circle subdivision estimation
     */
    bool tri_is_in_offset_conservative(const size_t f_id, const double threshold_r) const;

    /**
     * @brief grow offset region conservatively using conservative checks while ensuring consistent
     * topology
     */
    void grow_offset_conservative();
    //// variable offset stuff

    /**
     * @brief update 'tags' data for triangles in the offset region (tris labelled 2) based on
     * the given offset tag values in m_offset_params.offset_tag_value
     */
    void set_offset_tri_tags();

    /**
     * @brief verify that the closed offset region (simplices labelled 1 or 2) form a manifold
     * region. This should be true for any offset. This function is for verification
     */
    bool offset_is_manifold();

    //// output stuff
    void write_input_complex(const std::string& path);
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
        Vector2d new_v_pos;
        VertexExtra2d new_v_extra;

        // cache edge attributes
        EdgeSnapshot2d split_eattr;
        std::map<simplex::Edge, EdgeSnapshot2d> existing_eattr;

        // cache face attributes
        std::map<size_t, FaceSnapshot2d> opp_v_fattr;
    };
    wmtk::threading::enumerable_thread_specific<EdgeSplitCache> edge_split_cache;

    struct FaceSplitCache
    {
        size_t v1_id;
        size_t v2_id;
        size_t v3_id;
        Vector2d new_v_pos;
        VertexExtra2d new_v_extra;

        std::map<simplex::Edge, EdgeSnapshot2d> existing_eattr; // 3 orig edges
        FaceSnapshot2d split_fattr; // split face attributes
    };
    wmtk::threading::enumerable_thread_specific<FaceSplitCache> face_split_cache;

private: // helpers
    /**
     * @brief determine if any tag from tag1 is also present in tag2.
     */
    bool any_tag_present(const CellTag& tag1, const CellTag& tag2) const
    {
        for (const int64_t& i : tag1) {
            if (tag2.find(i) != tag2.end()) {
                return true;
            }
        }
        return false;

        // if (tag2.empty()) {
        //     return tag1.empty();
        // }
        // if (tag1.empty()) { // tag1 is ambient and tag2 is not
        //     return false;
        // }

        // for (const int64_t& i : tag1) {
        //     if (tag2.find(i) != tag2.end()) {
        //         return true;
        //     }
        // }
        // return false;
    }

    /**
     * @brief sort vector of edge simplices in place by decreasing length
     */
    void sort_edges_by_length(std::vector<simplex::Edge>& edges)
    {
        std::sort(
            edges.begin(),
            edges.end(),
            [this](const simplex::Edge& e1, const simplex::Edge& e2) {
                double len1 = (m_vertex_attribute[e1.vertices()[0]].m_posf -
                               m_vertex_attribute[e1.vertices()[1]].m_posf)
                                  .squaredNorm();
                double len2 = (m_vertex_attribute[e2.vertices()[0]].m_posf -
                               m_vertex_attribute[e2.vertices()[1]].m_posf)
                                  .squaredNorm();
                return len1 > len2;
            });
    }

public: // helpers
    /**
     * @brief get global id of edge from simplex::Edge object
     */
    size_t edge_id_from_simplex(const simplex::Edge& e) const
    {
        const auto& verts = e.vertices();
        const auto incident = simplex_incident_triangles(e);
        const auto& faces = incident.faces();

        assert(!faces.empty()); // throw error here otherwise

        const size_t f_id = tuple_from_simplex(faces.front()).fid(*this);
        const Tuple t_edge = tuple_from_edge(verts[0], verts[1], f_id);
        return t_edge.eid(*this);
    }

    /**
     * @brief get Tuple simplex::Edge object
     */
    Tuple get_tuple_from_edge(const simplex::Edge& e) const
    {
        const auto& v = e.vertices();
        const auto faces = simplex_incident_triangles(e).faces();
        assert(!faces.empty());
        const size_t fid = tuple_from_simplex(faces.front()).fid(*this);
        return tuple_from_edge(v[0], v[1], fid);
    }

    /**
     * @brief get faces (as Tuples) that are edge-adjacent to the given face (as Tuple)
     */
    std::vector<Tuple> get_edge_adjacent_faces(const Tuple& f) const
    {
        std::vector<Tuple> adj_tris;
        auto tri_1 = f.switch_face(*this);
        if (tri_1) {
            adj_tris.push_back(tri_1.value());
        }
        auto tri_2 = f.switch_edge(*this).switch_face(*this);
        if (tri_2) {
            adj_tris.push_back(tri_2.value());
        }
        auto tri_3 = f.switch_vertex(*this).switch_edge(*this).switch_face(*this);
        if (tri_3) {
            adj_tris.push_back(tri_3.value());
        }
        return adj_tris;
    }
};


} // namespace wmtk::components::topological_offset
