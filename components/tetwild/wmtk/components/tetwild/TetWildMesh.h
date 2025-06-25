#pragma once

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Morton.h>
#include <wmtk/utils/PartitionMesh.h>
#include "Parameters.h"
#include "common.h"
#include "sec/envelope/SampleEnvelope.hpp"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/concurrent_map.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#include <tbb/parallel_sort.h>
#include <wmtk/utils/EnableWarnings.hpp>
#include <VolumeRemesher/embed.h>
// clang-format on

#include <igl/remove_unreferenced.h>
#include <memory>

namespace wmtk::components::tetwild {

// TODO: missing comments on what these attributes are
class VertexAttributes
{
public:
    Vector3r m_pos;
    Vector3d m_posf;
    bool m_is_rounded = false;

    bool m_is_on_surface = false;
    std::vector<int> on_bbox_faces; // same as is_bbox_fs?

    Scalar m_sizing_scalar = 1;

    size_t partition_id = 0;

    // for open boundary
    bool m_is_on_open_boundary = false;

    VertexAttributes(){};
    VertexAttributes(const Vector3r& p);
};


// class EdgeAttributes
// {
// public:
//     bool m_is_on_open_boundary = false;
// };

// TODO: missing comments on what these attributes are
class FaceAttributes
{
public:
    Scalar tag;

    bool m_is_surface_fs = false; // 0; 1
    int m_is_bbox_fs = -1; //-1; 0~5

    void reset()
    {
        m_is_surface_fs = false;
        m_is_bbox_fs = -1;
    }

    void merge(const FaceAttributes& attr)
    {
        m_is_surface_fs = m_is_surface_fs || attr.m_is_surface_fs;
        if (attr.m_is_bbox_fs >= 0) m_is_bbox_fs = attr.m_is_bbox_fs;
    }
};

// TODO: missing comments on what these attributes are
class TetAttributes
{
public:
    Scalar m_quality;
    Scalar m_winding_number = 0;
    int part_id = -1;
};

class TetWildMesh : public wmtk::TetMesh
{
public:
    double time_env = 0.0;
    igl::Timer isout_timer;
    const double MAX_ENERGY = 1e50;

    Parameters& m_params;
    wmtk::Envelope& m_envelope;
    // for surface projection
    sample_envelope::SampleEnvelope& triangles_tree;

    // for open boundary
    wmtk::ExactEnvelope m_open_boundary_envelope; // todo: add sample envelope option
    sample_envelope::SampleEnvelope boundaries_tree;

    TetWildMesh(
        Parameters& _m_params,
        wmtk::Envelope& _m_envelope,
        sample_envelope::SampleEnvelope& _triangles_tree,
        int _num_threads = 1)
        : m_params(_m_params)
        , m_envelope(_m_envelope)
        , triangles_tree(_triangles_tree)
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_face_attrs = &m_face_attribute;
        p_tet_attrs = &m_tet_attribute;
        m_collapse_check_link_condition = false;
    }

    ~TetWildMesh() {}
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    // using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    VertAttCol m_vertex_attribute;
    FaceAttCol m_face_attribute;
    TetAttCol m_tet_attribute;
    // EdgeAttCol m_edge_attribute;

    // only used with unit tests
    void create_mesh_attributes(
        const std::vector<VertexAttributes>& _vertex_attribute,
        const std::vector<TetAttributes>& _tet_attribute)
    {
        auto n_tet = _tet_attribute.size();
        m_vertex_attribute.resize(_vertex_attribute.size());
        m_face_attribute.resize(4 * n_tet);
        m_tet_attribute.resize(n_tet);

        // new for edge
        // m_edge_attribute.resize(6 * n_tet);

        for (auto i = 0; i < _vertex_attribute.size(); i++)
            m_vertex_attribute[i] = _vertex_attribute[i];
        m_tet_attribute.m_attributes = tbb::concurrent_vector<TetAttributes>(_tet_attribute.size());
        for (auto i = 0; i < _tet_attribute.size(); i++) m_tet_attribute[i] = _tet_attribute[i];
        for (auto i = 0; i < _tet_attribute.size(); i++)
            m_tet_attribute[i].m_quality = get_quality(tuple_from_tet(i));
    }

    // TODO This should not be here but inside wmtk
    void compute_vertex_partition()
    {
        auto partition_id = partition_TetMesh(*this, NUM_THREADS);
        for (auto i = 0; i < vert_capacity(); i++)
            m_vertex_attribute[i].partition_id = partition_id[i];
    }

    // TODO This should not be here but inside wmtk
    void compute_vertex_partition_morton()
    {
        if (NUM_THREADS == 0) return;

        wmtk::logger().info("Number of parts: {} by morton", NUM_THREADS);

        tbb::task_arena arena(NUM_THREADS);

        arena.execute([&] {
            std::vector<Eigen::Vector3d> V_v(vert_capacity());

            tbb::parallel_for(
                tbb::blocked_range<int>(0, V_v.size()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        V_v[i] = m_vertex_attribute[i].m_posf;
                    }
                });


            struct sortstruct
            {
                int order;
                Resorting::MortonCode64 morton;
            };

            std::vector<sortstruct> list_v;
            list_v.resize(V_v.size());
            // since the morton code requires a correct scale of input vertices,
            //  we need to scale the vertices if their coordinates are out of range
            std::vector<Eigen::Vector3d> V = V_v; // this is for rescaling vertices
            Eigen::Vector3d vmin, vmax;
            vmin = V.front();
            vmax = V.front();

            for (size_t j = 0; j < V.size(); j++) {
                for (int i = 0; i < 3; i++) {
                    vmin(i) = std::min(vmin(i), V[j](i));
                    vmax(i) = std::max(vmax(i), V[j](i));
                }
            }

            // get_bb_corners(V, vmin, vmax);
            Eigen::Vector3d center = (vmin + vmax) / 2;

            tbb::parallel_for(tbb::blocked_range<int>(0, V.size()), [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    V[i] = V[i] - center;
                }
            });

            Eigen::Vector3d scale_point =
                vmax - center; // after placing box at origin, vmax and vmin are symetric.

            double xscale, yscale, zscale;
            xscale = fabs(scale_point[0]);
            yscale = fabs(scale_point[1]);
            zscale = fabs(scale_point[2]);
            double scale = std::max(std::max(xscale, yscale), zscale);
            if (scale > 300) {
                tbb::parallel_for(
                    tbb::blocked_range<int>(0, V.size()),
                    [&](tbb::blocked_range<int> r) {
                        for (int i = r.begin(); i < r.end(); i++) {
                            V[i] = V[i] / scale;
                        }
                    });
            }

            constexpr int multi = 1000;
            tbb::parallel_for(tbb::blocked_range<int>(0, V.size()), [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    list_v[i].morton = Resorting::MortonCode64(
                        int(V[i][0] * multi),
                        int(V[i][1] * multi),
                        int(V[i][2] * multi));
                    list_v[i].order = i;
                }
            });

            const auto morton_compare = [](const sortstruct& a, const sortstruct& b) {
                return (a.morton < b.morton);
            };

            tbb::parallel_sort(list_v.begin(), list_v.end(), morton_compare);

            int interval = list_v.size() / NUM_THREADS + 1;

            tbb::parallel_for(
                tbb::blocked_range<int>(0, list_v.size()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        m_vertex_attribute[list_v[i].order].partition_id = i / interval;
                    }
                });
        });
    }

    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }

    ////// Attributes related

    void output_mesh(std::string file);
    void output_faces(std::string file, std::function<bool(const FaceAttributes&)> cond);

    void init_from_delaunay_box_mesh(const std::vector<Eigen::Vector3d>& vertices);

    void finalize_triangle_insertion(const std::vector<std::array<size_t, 3>>& faces);

    void init_from_input_surface(
        const std::vector<Vector3d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces,
        const std::vector<size_t>& partition_id);
    bool triangle_insertion_before(const std::vector<Tuple>& faces) override;
    bool triangle_insertion_after(const std::vector<std::vector<Tuple>>& new_faces) override;


public:
    void split_all_edges();
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& loc) override;

    void smooth_all_vertices();
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    void collapse_all_edges(bool is_limit_length = true);
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    void swap_all_edges_44();
    bool swap_edge_44_before(const Tuple& t) override;
    bool swap_edge_44_after(const Tuple& t) override;

    void swap_all_edges();
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    void swap_all_faces();
    bool swap_face_before(const Tuple& t) override;
    bool swap_face_after(const Tuple& t) override;

    bool is_inverted(const Tuple& loc) const;
    double get_quality(const Tuple& loc) const;
    bool round(const Tuple& loc);
    //
    bool is_edge_on_surface(const Tuple& loc);
    bool is_edge_on_bbox(const Tuple& loc);
    //
    bool adjust_sizing_field(double max_energy);
    void mesh_improvement(int max_its = 80);
    std::tuple<double, double> local_operations(
        const std::array<int, 4>& ops,
        bool collapse_limit_length = true);
    std::tuple<double, double> get_max_avg_energy();
    void filter_outside(
        const std::vector<Vector3d>& vertices = {},
        const std::vector<std::array<size_t, 3>>& faces = {},
        bool remove_ouside = true);

    bool check_attributes();

    std::vector<std::array<size_t, 3>> get_faces_by_condition(
        std::function<bool(const FaceAttributes&)> cond);

    bool invariants(const std::vector<Tuple>& t) override; // this is now automatically checked

    double get_length2(const Tuple& loc) const;
    // debug use
    std::atomic<int> cnt_split = 0, cnt_collapse = 0, cnt_swap = 0;

private:
    // tags: correspondence map from new tet-face node indices to in-triangle ids.
    // built up while triangles are inserted.
    tbb::concurrent_map<std::array<size_t, 3>, std::vector<int>> tet_face_tags;

    struct TriangleInsertionLocalInfoCache
    {
        // local info: for each face insertion
        int face_id;
        std::vector<std::array<size_t, 3>> old_face_vids;
    };
    tbb::enumerable_thread_specific<TriangleInsertionLocalInfoCache> triangle_insertion_local_cache;

    ////// Operations

    struct SplitInfoCache
    {
        //        VertexAttributes vertex_info;
        size_t v1_id;
        size_t v2_id;
        bool is_edge_on_surface = false;
        bool is_edge_open_boundary = false;
        std::vector<size_t> v1_param_type;
        std::vector<size_t> v2_param_type;

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
    };
    tbb::enumerable_thread_specific<SplitInfoCache> split_cache;

    struct CollapseInfoCache
    {
        size_t v1_id;
        size_t v2_id;
        double max_energy;
        double edge_length;
        bool is_limit_length;

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
        std::vector<std::array<size_t, 3>> surface_faces;
        std::vector<size_t> changed_tids;

        std::vector<std::array<size_t, 2>> failed_edges;

        std::map<std::pair<size_t, size_t>, int> edge_link;
        std::map<size_t, int> vertex_link;
        size_t global_nonmani_ver_cnt;

        // debug use
        std::vector<size_t> one_ring_surface_vertices;
        std::vector<std::pair<size_t, size_t>> one_ring_surface_edges;
        std::vector<std::array<size_t, 3>> one_ring_surface;

        // for geometry preservation
        std::vector<size_t> edge_incident_param_type;
    };
    tbb::enumerable_thread_specific<CollapseInfoCache> collapse_cache;


    struct SwapInfoCache
    {
        double max_energy;
        std::map<std::array<size_t, 3>, FaceAttributes> changed_faces;
    };
    tbb::enumerable_thread_specific<SwapInfoCache> swap_cache;


    // for incremental tetwild
public:
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

    std::vector<std::array<size_t, 3>> triangulate_polygon_face(std::vector<Vector3r> points);
    bool check_polygon_face_validity(std::vector<tetwild::Vector3r> points);

    bool check_nondegenerate_tets();
    void output_embedded_polygon_mesh(
        std::string output_dir,
        const std::vector<Vector3r>& v_rational,
        const std::vector<std::vector<size_t>>& polygon_faces,
        const std::vector<std::vector<size_t>>& polygon_cells,
        const std::vector<bool>& polygon_faces_on_input_surface);

    void output_embedded_polygon_surface_mesh(
        std::string output_dir,
        const std::vector<Vector3r>& v_rational,
        const std::vector<std::vector<size_t>>& polygon_faces,
        const std::vector<bool>& polygon_faces_on_input_surface);

    void output_tetrahedralized_embedded_mesh(
        std::string output_dir,
        const std::vector<Vector3r>& v_rational,
        const std::vector<std::array<size_t, 3>>& facets,
        const std::vector<std::array<size_t, 4>>& tets,
        const std::vector<bool>& tet_face_on_input_surface);

    void output_init_tetmesh(std::string output_dir);

    long long checksum_vidx();
    wmtk::Rational checksum_vpos();
    long long checksum_tidx();
    Scalar checksum_tquality();

    bool adjust_sizing_field_serial(double max_energy);

    // for open boundary
    void find_open_boundary();
    bool is_open_boundary_edge(const Tuple& e);

    // for topology preservation
    int count_vertex_links(const Tuple& v);
    int count_edge_links(const Tuple& e);

    // for geometry preservation
    struct coplanar_triangle_collection
    {
        bool effective = false;
        std::vector<size_t> face_ids;
        std::vector<size_t> tracked_face_ids;
        // std::map<std::pair<size_t, size_t>, bool> presented_edges;
        Vector3r normal;
        Vector3r a_pos; // a is the origin of the uv plane
        Vector3r b_pos;
        Vector3r c_pos;
        Vector3r param_u; // b-a // can be vector3d then can be normalized
        Vector3r param_v; // u.cross(b-a)
        // for nearly
        Vector3d normal_f = Vector3d(0, 0, 0);
        Vector3d a_pos_f = Vector3d(0, 0, 0);
        Vector3d param_u_f = Vector3d(0, 0, 0);
        Vector3d param_v_f = Vector3d(0, 0, 0);
    };

    bool is_triangle_coplanar_collection(
        const Vector3r& v1,
        const Vector3r& v2,
        const Vector3r& v3,
        const coplanar_triangle_collection& collection);

    bool is_triangle_nearly_coplanar_collection(
        const Vector3r& v1,
        const Vector3r& v2,
        const Vector3r& v3,
        const coplanar_triangle_collection& collection,
        double theta);

    std::vector<std::vector<size_t>> transfer_vf_to_face_face_connectivity(
        size_t num_v,
        std::vector<std::array<size_t, 3>> faces);


    struct triangle_collections
    {
        std::vector<coplanar_triangle_collection> collections;
        std::vector<coplanar_triangle_collection> nearly_coplanar_collections;
        std::vector<size_t> exact_to_nearly_map;
        std::vector<Vector3r> input_vertices_rational;
        std::vector<std::array<size_t, 3>> input_faces;
        // can add parameterizations here
    };
    triangle_collections triangle_collections_from_input_surface;

    struct edge_parametrization
    {
        Vector3d direction;
        Vector3d origin;
    };
    std::vector<edge_parametrization> edge_params;

    void detect_coplanar_triangle_collections(
        const std::vector<Vector3d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces);

    int find_collection_for_tracked_surface(const Tuple& t);

    bool is_point_in_triangle(
        const Vector3r& p,
        const Vector3r& a,
        const Vector3r& b,
        const Vector3r& c);
    bool is_point_in_collection(const Vector3r& p, size_t collection_id);


    // debug functions
    int orient3D(
        vol_rem::bigrational px,
        vol_rem::bigrational py,
        vol_rem::bigrational pz,
        vol_rem::bigrational qx,
        vol_rem::bigrational qy,
        vol_rem::bigrational qz,
        vol_rem::bigrational rx,
        vol_rem::bigrational ry,
        vol_rem::bigrational rz,
        vol_rem::bigrational sx,
        vol_rem::bigrational sy,
        vol_rem::bigrational sz);

    bool checkTrackedFaces(
        std::vector<vol_rem::bigrational>& vol_coords,
        const std::vector<double>& surf_coords,
        std::vector<uint32_t>& facets,
        std::vector<uint32_t>& facets_on_input,
        const std::vector<uint32_t>& surf_tris);

    int orient3D_wmtk_rational(
        wmtk::Rational px,
        wmtk::Rational py,
        wmtk::Rational pz,
        wmtk::Rational qx,
        wmtk::Rational qy,
        wmtk::Rational qz,
        wmtk::Rational rx,
        wmtk::Rational ry,
        wmtk::Rational rz,
        wmtk::Rational sx,
        wmtk::Rational sy,
        wmtk::Rational sz);

    bool checkTrackedFaces_wmtk_rational(
        std::vector<wmtk::Rational>& vol_coords,
        const std::vector<double>& surf_coords,
        std::vector<uint32_t>& facets,
        std::vector<uint32_t>& facets_on_input,
        const std::vector<uint32_t>& surf_tris);

    bool check_vertex_param_type();

    // for boolean operations
    int flood_fill();

    void save_paraview(const std::string& path, const bool use_hdf5);

    // initialize sizing field (for topology preservation)
    void init_sizing_field();
};


} // namespace wmtk::components::tetwild
