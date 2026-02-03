#include "partition_utils.hpp"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <wmtk/utils/Morton.h>

void wmtk::partition_vertex_morton(
    int vert_size,
    const std::function<Eigen::Vector3d(size_t)>& pos,
    int num_partition,
    std::vector<size_t>& result)
{
    tbb::task_arena arena(num_partition);

    std::vector<Eigen::Vector3d> V_v(vert_size);
    arena.execute([&] {
        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, V_v.size()),
            [&](tbb::blocked_range<size_t> r) {
                for (size_t i = r.begin(); i < r.end(); i++) {
                    V_v[i] = pos(i);
                }
            });


        struct sortstruct
        {
            size_t order;
            Resorting::MortonCode64 morton;
        };

        std::vector<sortstruct> list_v;
        list_v.resize(V_v.size());
        // std::vector<sortstruct> list;
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

        int interval = list_v.size() / num_partition + 1;

        result.clear();
        result.resize(vert_size);
        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, list_v.size()),
            [&](tbb::blocked_range<size_t> r) {
                for (size_t i = r.begin(); i < r.end(); i++) {
                    result[list_v[i].order] = i / interval;
                }
            });
    });
}