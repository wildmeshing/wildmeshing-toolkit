#include <wmtk/utils/PartitionMesh.h>

#include <igl/remove_unreferenced.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>
#include <wmtk/utils/Morton.h>
#include <wmtk/utils/Partitioning.h>

namespace wmtk {

constexpr auto _partition_faces = [](auto& m, auto num_partition) {
    auto f_tuples = m.get_faces();
    Eigen::MatrixXi F(f_tuples.size(), 3);
    for (int i = 0; i < f_tuples.size(); i++) {
        F(i, 0) = (int)f_tuples[i].vid(m);
        auto e1 = f_tuples[i].switch_vertex(m);
        F(i, 1) = (int)e1.vid(m);
        F(i, 2) = (int)e1.switch_edge(m).switch_vertex(m).vid(m);
    }

    Eigen::VectorXi I, J;
    igl::remove_unreferenced(m.vert_capacity(), F, I, J);
    Eigen::MatrixXi NF = F;
    for (auto i = 0; i < NF.rows(); i++) {
        for (auto j = 0; j < 3; j++) {
            NF(i, j) = I(NF(i, j));
        }
    }

    auto partitioned_v = partition_mesh_vertices(NF, num_partition);
    assert(partitioned_v.size() == J.rows());
    std::vector<size_t> v_pid(m.vert_capacity(), 0);
    for (int i = 0; i < partitioned_v.rows(); i++) {
        v_pid[J[i]] = partitioned_v(i, 0);
    }
    return v_pid;
};

std::vector<size_t> partition_TriMesh(const wmtk::TriMesh& m, int num_partition)
{
    return _partition_faces(m, num_partition);
}

std::vector<size_t> partition_TetMesh(wmtk::TetMesh& m, int num_partition)
{
    return _partition_faces(m, num_partition);
}

std::vector<size_t> partition_morton(std::vector<Eigen::Vector3d> vertex_position, int NUM_THREADS)
{
    std::vector<size_t> partition_id(vertex_position.size());
    tbb::task_arena arena(NUM_THREADS);

    arena.execute([&] {
        std::vector<Eigen::Vector3d> V_v = vertex_position;
        struct sortstruct
        {
            int order;
            Resorting::MortonCode64 morton;
        };
        std::vector<sortstruct> list_v;
        list_v.resize(V_v.size());
        const int multi = 1000;
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
        int interval = list_v.size() / NUM_THREADS + 1;

        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, list_v.size()),
            [&](tbb::blocked_range<size_t> r) {
                for (size_t i = r.begin(); i < r.end(); i++) {
                    partition_id[list_v[i].order] = i / interval;
                }
            });
    });

    return partition_id;
}

} // namespace wmtk
