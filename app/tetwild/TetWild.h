#pragma once

#include <tbb/concurrent_map.h>
#include <wmtk/ConcurrentTetMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include "Parameters.h"
#include "common.h"

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
// clang-format on

#include <igl/remove_unreferenced.h>
#include <wmtk/utils/Morton.h>
#include <wmtk/utils/PartitionMesh.h>
#include <memory>

namespace tetwild {

class VertexAttributes
{
public:
    Vector3r m_pos;
    Vector3d m_posf;
    bool m_is_rounded = false;

    bool m_is_on_surface = false;
    std::vector<int> on_bbox_faces;
    bool m_is_outside = false;

    Scalar m_sizing_scalar = 1;
    Scalar m_scalar = 1;
    bool m_is_freezed = false;

    size_t partition_id = 0;

    VertexAttributes(){};
    VertexAttributes(const Vector3r& p);
};


class FaceAttributes
{
public:
    Scalar tag;

    bool m_is_surface_fs = false; // 0; 1
    int m_is_bbox_fs = -1; //-1; 0~5

    int m_surface_tags = -1;

    void reset()
    {
        m_is_surface_fs = false;
        m_is_bbox_fs = -1;
        m_surface_tags = -1;
    }

    void merge(const FaceAttributes& attr)
    {
        m_is_surface_fs = m_is_surface_fs || attr.m_is_surface_fs;
        if (attr.m_is_bbox_fs >= 0) m_is_bbox_fs = attr.m_is_bbox_fs;
        m_surface_tags = std::max(m_surface_tags, attr.m_surface_tags);
    }
};

class TetAttributes
{
public:
    Scalar m_quality;
    Scalar m_scalar;
    bool m_is_outside;
};

class TetWild : public wmtk::ConcurrentTetMesh
{
public:
    const double MAX_ENERGY = 1e50;

    Parameters& m_params;
    fastEnvelope::FastEnvelope& m_envelope;

    TetWild(Parameters& _m_params, fastEnvelope::FastEnvelope& _m_envelope, int _num_threads = 1)
        : m_params(_m_params)
        , m_envelope(_m_envelope)
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_face_attrs = &m_face_attribute;
        p_tet_attrs = &m_tet_attribute;
        m_collapse_check_link_condition = false;
    }

    ~TetWild() {}
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    VertAttCol m_vertex_attribute;
    FaceAttCol m_face_attribute;
    TetAttCol m_tet_attribute;

    void create_mesh_attributes(
        const std::vector<VertexAttributes>& _vertex_attribute,
        const std::vector<TetAttributes>& _tet_attribute)
    {
        auto n_tet = _tet_attribute.size();
        m_vertex_attribute.resize(_vertex_attribute.size());
        m_face_attribute.resize(4 * n_tet);
        m_tet_attribute.resize(n_tet);

        for (auto i = 0; i < _vertex_attribute.size(); i++)
            m_vertex_attribute[i] = _vertex_attribute[i];
        m_tet_attribute.m_attributes = tbb::concurrent_vector<TetAttributes>(_tet_attribute.size());
        for (auto i = 0; i < _tet_attribute.size(); i++) m_tet_attribute[i] = _tet_attribute[i];
    }

    void compute_vertex_partition()
    {
        auto partition_id = partition_TetMesh(*this, NUM_THREADS);
        for (auto i = 0; i < m_vertex_attribute.size(); i++)
            m_vertex_attribute[i].partition_id = partition_id[i];
    }

    void compute_vertex_partition_morton()
    {
        auto f_tuples = get_faces();
        Eigen::MatrixXi F(f_tuples.size(), 3);

        tbb::task_arena arena(NUM_THREADS);

        arena.execute([&] {
            tbb::parallel_for(
                tbb::blocked_range<int>(0, f_tuples.size()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        F(i, 0) = f_tuples[i].vid(*this);
                        auto e1 = f_tuples[i].switch_vertex(*this);
                        F(i, 1) = e1.vid(*this);
                        F(i, 2) = e1.switch_edge(*this).switch_vertex(*this).vid(*this);
                    }
                });

            // for (int i = 0; i < f_tuples.size(); i++) {
            //     F(i, 0) = f_tuples[i].vid(*this);
            //     auto e1 = f_tuples[i].switch_vertex(*this);
            //     F(i, 1) = e1.vid(*this);
            //     F(i, 2) = e1.switch_edge(*this).switch_vertex(*this).vid(*this);
            // }

            Eigen::VectorXi I, J;
            igl::remove_unreferenced(vert_capacity(), F, I, J);

            std::vector<Eigen::Vector3i> F_v(F.rows());

            tbb::parallel_for(tbb::blocked_range<int>(0, F.rows()), [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    for (auto j = 0; j < 3; j++) {
                        F_v[i][j] = I(F(i, j));
                    }
                }
            });

            // for (auto i = 0; i < F.rows(); i++) {
            //     for (auto j = 0; j < 3; j++) {
            //         F_v[i][j] = I(F(i, j));
            //     }
            // }

            std::vector<Eigen::Vector3d> V_v(m_vertex_attribute.size());

            tbb::parallel_for(
                tbb::blocked_range<int>(0, V_v.size()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        V_v[i] = m_vertex_attribute[i].m_posf;
                    }
                });

            // for (int i = 0; i < V_v.size(); i++) {
            //     V_v[i] = m_vertex_attribute[i].m_posf;
            // }

            std::vector<Eigen::Vector3i> fnew;
            std::vector<std::array<int, 3>> ct;
            struct sortstruct
            {
                int order;
                Resorting::MortonCode64 morton;
            };
            std::vector<sortstruct> list;
            const int multi = 1000;
            ct.resize(F_v.size());
            list.resize(F_v.size());

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

            // for (int i = 0; i < V.size(); i++) {
            //     V[i] = V[i] - center; // make box centered at origin
            // }

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

                // for (int i = 0; i < V.size(); i++) {
                //     V[i] = V[i] / scale; // if the box is too big, resize it
                // }
            }

            tbb::parallel_for(
                tbb::blocked_range<int>(0, F_v.size()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        ct[i][0] = int(((V[F_v[i][0]] + V[F_v[i][1]] + V[F_v[i][2]]) * multi)[0]);
                        ct[i][1] = int(((V[F_v[i][0]] + V[F_v[i][1]] + V[F_v[i][2]]) * multi)[1]);
                        ct[i][2] = int(((V[F_v[i][0]] + V[F_v[i][1]] + V[F_v[i][2]]) * multi)[2]);
                        list[i].morton = Resorting::MortonCode64(ct[i][0], ct[i][1], ct[i][2]);
                        list[i].order = i;
                    }
                });

            // for (int i = 0; i < F_v.size(); i++) {
            //     ct[i][0] = int(((V[F_v[i][0]] + V[F_v[i][1]] + V[F_v[i][2]]) * multi)[0]);
            //     ct[i][1] = int(((V[F_v[i][0]] + V[F_v[i][1]] + V[F_v[i][2]]) * multi)[1]);
            //     ct[i][2] = int(((V[F_v[i][0]] + V[F_v[i][1]] + V[F_v[i][2]]) * multi)[2]);
            //     list[i].morton = Resorting::MortonCode64(ct[i][0], ct[i][1], ct[i][2]);
            //     list[i].order = i;
            // }
            const auto morton_compare = [](const sortstruct& a, const sortstruct& b) {
                return (a.morton < b.morton);
            };
            tbb::parallel_sort(list.begin(), list.end(), morton_compare);
            // std::sort(list.begin(), list.end(), morton_compare);

            fnew.resize(F_v.size());

            tbb::parallel_for(
                tbb::blocked_range<int>(0, F_v.size()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        fnew[i] = F_v[list[i].order];
                    }
                });

            // for (int i = 0; i < F_v.size(); i++) {
            //     fnew[i] = F_v[list[i].order];

            int interval = fnew.size() / NUM_THREADS + 1;

            tbb::parallel_for(
                tbb::blocked_range<int>(0, fnew.size()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        m_vertex_attribute[fnew[i][0]].partition_id = i / interval;
                    }
                });

            // for (int i = 0; i < fnew.size(); i++) {
            //     m_vertex_attribute[fnew[i][0]].partition_id = i / interval;
            // }
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

    void finalize_triangle_insertion(
        const std::vector<std::array<size_t, 3>>& faces);

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
        bool collapse_limite_length = true);
    std::tuple<double, double> get_max_avg_energy();
    void filter_outside(
        const std::vector<Vector3d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces,
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
    };
    tbb::enumerable_thread_specific<CollapseInfoCache> collapse_cache;


    struct SwapInfoCache
    {
        double max_energy;
        std::map<std::array<size_t, 3>, FaceAttributes> changed_faces;
    };
    tbb::enumerable_thread_specific<SwapInfoCache> swap_cache;
};

} // namespace tetwild
