#pragma once

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>

// clang-format off
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <memory>

namespace wmtk::components::c1_simplification {

/*
attributes
*/
struct SurfaceMeshVertexAttribute
{
    int64_t vid_in_tet = -1;

    Vector3d pos;

    // gradient at vertex
    Vector3d grad;

    bool is_cone = false;

    SurfaceMeshVertexAttribute() {}
    SurfaceMeshVertexAttribute(const Vector3d& p, const Vector3d& g, const int64_t& tvid)
        : pos(p)
        , grad(g)
        , vid_in_tet(tvid)
    {}
};

struct SurfaceMeshEdgeAttribute
{
    Vector3d midgrad;

    SurfaceMeshEdgeAttribute() {}
    SurfaceMeshEdgeAttribute(const Vector3d& g)
        : midgrad(g)
    {}
};

struct SurfaceMeshFaceAttribute
{
    // midpoint gradients for local edges
    std::array<Vector3d, 3> midgrad;
    Eigen::Matrix<double, 12, 3> dofs;

    SurfaceMeshFaceAttribute() {}
    SurfaceMeshFaceAttribute(const std::array<Vector3d, 3>& g)
        : midgrad(g)
    {}
    SurfaceMeshFaceAttribute(const Vector3d& g0, const Vector3d& g1, const Vector3d& g2)
    {
        midgrad = {{g0, g1, g2}};
    }
};

struct TetMeshVertexAttribute
{
    int64_t vid_in_surface = -1;

    Vector3d pos;

    TetMeshVertexAttribute() {}
    TetMeshVertexAttribute(const Vector3d& p, const int64_t& svid)
        : pos(p)
        , vid_in_surface(svid)
    {}
};

struct UVMeshVertexAttribute
{
    Vector2d pos;

    bool is_cone = false;

    UVMeshVertexAttribute() {}
    UVMeshVertexAttribute(const Vector2d& p)
        : pos(p)
    {}
};

/*
multimesh meshes
*/

class MMTetMesh : public wmtk::TetMesh
{
public:
    MMTetMesh() { p_vertex_attrs = &v_attrs; }
    ~MMTetMesh() {}

    void init_from_eigen(const MatrixXd& V, const MatrixXi& T);

    bool is_inverted(const Tuple& t) const;

    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    struct TetCollpaseInfoCache
    {
        size_t v1_id;
        size_t v2_id;
        double max_energy;
        double edge_length;
        bool is_limit_length;

        // std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
        std::vector<std::array<size_t, 3>> surface_faces;
        std::vector<size_t> changed_tids;

        std::vector<std::array<size_t, 2>> failed_edges;

        std::map<std::pair<size_t, size_t>, int> edge_link;
        std::map<size_t, int> vertex_link;
        size_t global_nonmani_ver_cnt;
    };

    TetCollpaseInfoCache t_cache;

public:
    using Tvattrs = wmtk::AttributeCollection<TetMeshVertexAttribute>;
    Tvattrs v_attrs;
};

class MMUVMesh : public wmtk::TriMesh
{
public:
    MMUVMesh() { p_vertex_attrs = &v_attrs; }
    ~MMUVMesh() {}

    void init_from_eigen(const MatrixXd& V, const MatrixXi& F);

    bool is_inverted(const Tuple& t) const;

    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    void output_uv_mesh(const std::string& file, bool consolidate = false);


    struct UVCollpaseInfoCache
    {
        Vector2d old_uv_pos;
        Vector2d v2_pos;
    };

    UVCollpaseInfoCache uv_cache;

public:
    using UVvattrs = wmtk::AttributeCollection<UVMeshVertexAttribute>;
    UVvattrs v_attrs;
};

class MMSurfaceMesh : public wmtk::TriMesh
{
public:
    MMSurfaceMesh() {}
    MMSurfaceMesh(
        std::shared_ptr<MMTetMesh>& t_ptr,
        std::shared_ptr<MMUVMesh>& uv_ptr,
        const double threshold)
        : tetmesh_ptr(t_ptr)
        , uvmesh_ptr(uv_ptr)
        , deviation_threshold(threshold)
    {
        p_vertex_attrs = &v_attrs;
        p_edge_attrs = &e_attrs;
        // p_face_attrs = f_attrs;
    }

    void init_from_eigen_with_map_and_dofs(
        const MatrixXd& V,
        const MatrixXi& F,
        const std::map<int64_t, int64_t>& s2t_vid_map,
        const std::vector<Eigen::Matrix<double, 12, 3>>& dofs,
        const std::vector<size_t>& cone_vids);
    // const std::vector<Vector3d>& vgrads,
    // const std::vector<std::array<Vector3d, 3>>& egrads);

    // mapping functions
    TetMesh::Tuple map_to_tet_edge_tuple(const Tuple& e);
    TetMesh::Tuple map_to_tet_vertex_tuple(const Tuple& v);
    std::vector<Tuple> map_to_uv_edge_tuples(const Tuple& e);
    std::vector<Tuple> map_to_uv_vertex_tuples(const Tuple& v);
    Tuple map_to_equivalent_uv_tuple(const Tuple& v); // return uv tuple with exact same fid, leid
                                                      // and same vertex (vid may be different)

    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;
    bool multimesh_collapse_edge(const Tuple& t);

    Eigen::Matrix<double, 12, 3> assemble_dofs(const size_t& fid);

    void output_surface_mesh(const std::string& file);

    void clear_info_cache();

public:
    double deviation_threshold;

    using Svattrs = wmtk::AttributeCollection<SurfaceMeshVertexAttribute>;
    using Seattrs = wmtk::AttributeCollection<SurfaceMeshEdgeAttribute>;
    using Sfattrs = wmtk::AttributeCollection<SurfaceMeshFaceAttribute>;
    Svattrs v_attrs;
    Seattrs e_attrs;
    Sfattrs f_attrs;

    struct TrackedVertexInfo
    {
        size_t vid;
        size_t ref_uv_vid; // vid for the collapse-from vertex (v1) in the face containing
                           // the tracked vertex

        Vector3d surface_pos;
        Vector2d uv_pos; // uv position for the tracked vertex
        Vector2d ref_uv_pos; // uv position for the collapse-from vertex (v1) in the face containing
                             // the tracked vertex

        Vector2d translation; // ref_pos to origin_pos
        double rotation;
    };

    struct LayoutPartInfo // info for vert chart parts
    {
        size_t ref_uv_vid;

        double rotation;
        Vector2d translation;

        Vector2d new_pos;


        LayoutPartInfo() {}

        LayoutPartInfo(
            const size_t& vid,
            const Vector2d& new_p,
            const Vector2d& trans,
            const double rot)
            : ref_uv_vid(vid)
            , new_pos(new_p)
            , translation(trans)
            , rotation(rot)
        {}
    };

    struct SurfaceCollpaseInfoCache
    {
        Vector3d old_pos;
        Vector3d v2_pos;

        Vector2d v1_uv_pos;
        Vector2d v2_uv_pos;

        size_t ref_uv_v1_id;
        size_t ref_uv_v2_id;

        size_t deleted_fid_1;
        size_t deleted_fid_2 = -1;

        size_t deleted_vid_1; // uv
        size_t deleted_vid_2 = -1; // uv

        size_t uv_collapse_to_v1_id; // uv
        size_t uv_collapse_to_v2_id = -1; // uv

        std::map<size_t, LayoutPartInfo> layout_parts_map;
        std::vector<TrackedVertexInfo> tracked_vertex_info_cache;
        std::map<size_t, bool> collapse_area_fid_involved;
    };

    SurfaceCollpaseInfoCache s_cache;

    std::shared_ptr<MMTetMesh> tetmesh_ptr;
    std::shared_ptr<MMUVMesh> uvmesh_ptr;

    // for uv tracking
    struct tracked_vertex
    {
        Vector2d uv_pos;
        Vector3d surface_pos;

        size_t vid;
        size_t in_tri_id = -1;

        tracked_vertex() {}

        tracked_vertex(const Vector2d& p, const Vector3d& sp, const size_t& id, const size_t& fid)
            : uv_pos(p)
            , surface_pos(sp)
            , vid(id)
            , in_tri_id(fid)
        {}
    };

    std::vector<tracked_vertex> tracked_vertices;

    std::map<size_t, std::vector<size_t>> tracked_fid_to_vids_map;
    std::map<size_t, size_t> tracked_vid_to_fid_map;

    void output_tracked_vertices(const std::string& filename);
};


} // namespace wmtk::components::c1_simplification