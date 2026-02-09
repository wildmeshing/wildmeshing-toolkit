#pragma once

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Morton.h>
#include <wmtk/utils/PartitionMesh.h>
#include <SimpleBVH/BVH.hpp>
#include <functional>
#include <wmtk/simplex/RawSimplex.hpp>

#include "Parameters.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/concurrent_map.h>
#include <tbb/parallel_sort.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <igl/remove_unreferenced.h>
#include <memory>

namespace wmtk::components::tet_implicits {

class VertexAttributes
{
public:
    Vector3d m_posf;

    size_t partition_id = 0;

    /**
     * Tags on the incident tets. A pair represents [image_id, tag_id].
     * On the vector inside a tet, that would mean: tet.tags[pair.first] == pair.second
     */
    // std::vector<std::pair<int64_t, int64_t>> tags;
    bool m_is_inside = false;
    double m_sq_sdf = std::numeric_limits<double>::max();

    VertexAttributes() {};
    VertexAttributes(const Vector3d& p);
};


// class EdgeAttributes
// {
// public:
//     bool m_is_on_open_boundary = false;
// };

// class FaceAttributes
//{
// public:
//     void reset() {}
//
//     void merge(const FaceAttributes& attr) {}
// };

class TetAttributes
{
public:
    double m_quality;
    std::vector<int64_t> tags;
};

class TetImplicitsMesh : public wmtk::TetMesh
{
public:
    int m_debug_print_counter = 0;
    size_t m_tags_count = 0; // length of tags vector in TetAttributes

    double time_env = 0.0;
    igl::Timer isout_timer;
    const double MAX_ENERGY = std::numeric_limits<double>::max();

    Parameters& m_params;

    /**
     * Signed Distance Function
     * inside: < 0
     * outside: > 0
     */
    std::function<double(const Vector3d&)> m_sq_sdf = nullptr;
    /**
     * Decide, based on the position (e.g., center of tet) what tag to assign
     */
    std::function<std::pair<int64_t, int64_t>(const Vector3d&)> m_new_tag = nullptr;

    std::map<
        std::pair<int64_t, int64_t>,
        std::shared_ptr<SimpleBVH::BVH>>
        m_bvh; // one BVH for each input tag

    TetImplicitsMesh(Parameters& params, int _num_threads = 0)
        : m_params(params)
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        // p_face_attrs = &m_face_attribute;
        p_tet_attrs = &m_tet_attribute;
    }

    ~TetImplicitsMesh() {}
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    // using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    // using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    VertAttCol m_vertex_attribute;
    // EdgeAttCol m_edge_attribute;
    // FaceAttCol m_face_attribute;
    TetAttCol m_tet_attribute;

    // only used with unit tests
    void create_mesh_attributes(
        const std::vector<VertexAttributes>& _vertex_attribute,
        const std::vector<TetAttributes>& _tet_attribute)
    {
        auto n_tet = _tet_attribute.size();
        m_vertex_attribute.resize(_vertex_attribute.size());
        // m_face_attribute.resize(4 * n_tet);
        m_tet_attribute.resize(n_tet);

        for (auto i = 0; i < _vertex_attribute.size(); i++) {
            m_vertex_attribute[i] = _vertex_attribute[i];
        }
        m_tet_attribute.m_attributes = tbb::concurrent_vector<TetAttributes>(_tet_attribute.size());
        for (auto i = 0; i < _tet_attribute.size(); i++) {
            m_tet_attribute[i] = _tet_attribute[i];
        }
        for (auto i = 0; i < _tet_attribute.size(); i++) {
            m_tet_attribute[i].m_quality = get_quality(tuple_from_tet(i));
        }
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
                tbb::blocked_range<size_t>(0, V_v.size()),
                [&](tbb::blocked_range<size_t> r) {
                    for (size_t i = r.begin(); i < r.end(); i++) {
                        V_v[i] = m_vertex_attribute[i].m_posf;
                    }
                });


            struct sortstruct
            {
                size_t order;
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

            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, V.size()),
                [&](tbb::blocked_range<size_t> r) {
                    for (size_t i = r.begin(); i < r.end(); i++) {
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
                    tbb::blocked_range<size_t>(0, V.size()),
                    [&](tbb::blocked_range<size_t> r) {
                        for (size_t i = r.begin(); i < r.end(); i++) {
                            V[i] = V[i] / scale;
                        }
                    });
            }

            constexpr int multi = 1000;
            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, V.size()),
                [&](tbb::blocked_range<size_t> r) {
                    for (size_t i = r.begin(); i < r.end(); i++) {
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

            size_t interval = list_v.size() / NUM_THREADS + 1;

            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, list_v.size()),
                [&](tbb::blocked_range<size_t> r) {
                    for (size_t i = r.begin(); i < r.end(); i++) {
                        m_vertex_attribute[list_v[i].order].partition_id = i / interval;
                    }
                });
        });
    }

    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }

    double get_length2(const Tuple& l) const;

    Vector3d tet_center(const size_t tid) const;

    bool tet_is_inside(const size_t tid) const;

    ////// Attributes related

    void write_msh(std::string file);

public:
    void split_all_edges();
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& loc) override;

    bool is_inverted(const std::array<size_t, 4>& vs) const;
    bool is_inverted(const Tuple& loc) const;
    double get_quality(const std::array<size_t, 4>& vs) const;
    double get_quality(const Tuple& loc) const;

    bool is_vertex_on_boundary(const size_t vid);

    std::tuple<double, double> get_max_avg_energy();
    /**
     * @brief Compute the mean and standard deviation of the edge lengths.
     */
    std::tuple<double, double> get_mean_dev_edge_length();

    bool check_attributes();

    // std::vector<std::array<size_t, 3>> get_faces_by_condition(
    //     std::function<bool(const FaceAttributes&)> cond);

    // debug use
    std::atomic<int> cnt_split = 0, cnt_collapse = 0, cnt_swap = 0;

public:
    ////// Operations

    /**
     * @brief Separation operation.
     *
     * Requires:
     * 2 input tags (in the future, more may be supported)
     * 1 output tag
     * d = distance to both objects
     *
     * Considers everything that is within "d" distance of both objects as inside.
     */
    void op_separate();

    /**
     * @brief Tight seal operation.
     *
     * Requires:
     * 2 input tags (in the future, more may be supported)
     * 0 output tags
     * d = distance within the tight seal should be computed
     *
     * Considers everything that is within "d" distance of both objects as inside.
     */
    void op_tight_seal();

private:
    struct SplitInfoCache
    {
        //        VertexAttributes vertex_info;
        size_t v0_id;
        size_t v1_id;

        // std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;

        /**
         * All tets incident to the splitted edge, identified by the link edge (the edge opposite to
         * the splitted one).
         */
        std::map<simplex::Edge, TetAttributes> tets;
    };
    tbb::enumerable_thread_specific<SplitInfoCache> split_cache;

public:
    /**
     * @brief Init from meshes image.
     *
     * @param V #Vx3 vertices of the tet mesh
     * @param T #Tx4 vertex IDs for all tets
     * @param F #Fx3 vertex IDs of all embedded faces
     * @param T_tags #Tx1 image data represented by the individual tets
     */
    void init_from_image(const MatrixXd& V, const MatrixXi& T, const MatrixXi& T_tags);

    void init_bvhs();

    std::vector<std::array<size_t, 3>> triangulate_polygon_face(std::vector<Vector3r> points);

    void write_vtu(const std::string& path);

    void write_surface(const std::string& path) const;
};

} // namespace wmtk::components::tet_implicits
