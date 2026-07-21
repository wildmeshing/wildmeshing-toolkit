#include "partition_utils.hpp"

#include <wmtk/utils/Morton.h>
#include <wmtk/threading/parallel_for.hpp>

void wmtk::partition_vertex_morton(
    size_t vert_size,
    const std::function<Eigen::Vector3d(size_t)>& pos,
    int num_partition,
    std::vector<size_t>& result)
{
    std::vector<Eigen::Vector3d> V_v(vert_size);

    wmtk::threading::parallel_for(
        wmtk::threading::blocked_range<size_t>(0, V_v.size()),
        [&](wmtk::threading::blocked_range<size_t> r) {
            for (size_t i = r.begin(); i < r.end(); i++) {
                V_v[i] = pos(i);
            }
        },
        num_partition);


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

    wmtk::threading::parallel_for(
        wmtk::threading::blocked_range<size_t>(0, V.size()),
        [&](wmtk::threading::blocked_range<size_t> r) {
            for (size_t i = r.begin(); i < r.end(); i++) {
                V[i] = V[i] - center;
            }
        },
        num_partition);

    Eigen::Vector3d scale_point =
        vmax - center; // after placing box at origin, vmax and vmin are symetric.

    double xscale, yscale, zscale;
    xscale = fabs(scale_point[0]);
    yscale = fabs(scale_point[1]);
    zscale = fabs(scale_point[2]);
    double scale = std::max(std::max(xscale, yscale), zscale);
    if (scale > 300) {
        wmtk::threading::parallel_for(
            wmtk::threading::blocked_range<size_t>(0, V.size()),
            [&](wmtk::threading::blocked_range<size_t> r) {
                for (size_t i = r.begin(); i < r.end(); i++) {
                    V[i] = V[i] / scale;
                }
            },
            num_partition);
    }

    wmtk::threading::parallel_for(
        wmtk::threading::blocked_range<size_t>(0, V.size()),
        [&](wmtk::threading::blocked_range<size_t> r) {
            for (size_t i = r.begin(); i < r.end(); i++) {
                list_v[i].morton = Resorting::MortonCode64(
                    int(V[i][0] * multi),
                    int(V[i][1] * multi),
                    int(V[i][2] * multi));
                list_v[i].order = i;
            }
        },
        num_partition);

    const auto morton_compare = [](const sortstruct& a, const sortstruct& b) {
        return (a.morton < b.morton);
    };

    std::sort(list_v.begin(), list_v.end(), morton_compare);

    size_t interval = list_v.size() / num_partition + 1;

    result.clear();
    result.resize(vert_size);
    wmtk::threading::parallel_for(
        wmtk::threading::blocked_range<size_t>(0, list_v.size()),
        [&](wmtk::threading::blocked_range<size_t> r) {
            for (size_t i = r.begin(); i < r.end(); i++) {
                result[list_v[i].order] = i / interval;
            }
        },
        num_partition);
}