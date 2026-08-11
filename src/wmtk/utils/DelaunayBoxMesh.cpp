#include <limits>
#include <wmtk/utils/DelaunayBoxMesh.hpp>

#include <wmtk/utils/Logger.hpp>

#include <bitset>

namespace wmtk::utils {

void delaunay_box_mesh(
    const SampleEnvelope& envelope,
    const Vector3d& bbox_min,
    const Vector3d& bbox_max,
    const double diag,
    std::vector<delaunay::Point3D>& points,
    std::vector<delaunay::Tetrahedron>& tets,
    Vector3d& box_min,
    Vector3d& box_max)
{
    // bbox
    const double delta = diag / 15.0;
    box_min = Vector3d(bbox_min[0] - delta, bbox_min[1] - delta, bbox_min[2] - delta);
    box_max = Vector3d(bbox_max[0] + delta, bbox_max[1] + delta, bbox_max[2] + delta);

    // add corners of domain
    for (int i = 0; i < 8; i++) {
        Vector3d p;
        std::bitset<sizeof(int) * 8> a(i);
        for (int j = 0; j < 3; j++) {
            if (a.test(j)) {
                p[j] = box_max[j];
            } else {
                p[j] = box_min[j];
            }
        }
        points.push_back({{p[0], p[1], p[2]}});
    }

    const double voxel_resolution = diag / 20.0;
    std::array<int, 3> N; // number of grid points per dimension
    std::array<double, 3> h; // distance between grid points per dimension
    for (int i = 0; i < 3; i++) {
        const double D = box_max[i] - box_min[i];
        N[i] = (D / voxel_resolution) + 1;
        h[i] = D / N[i];
    }

    std::array<std::vector<double>, 3> ds;
    for (int i = 0; i < 3; i++) {
        ds[i].push_back(box_min[i]);
        for (int j = 0; j < N[i] - 1; j++) {
            ds[i].push_back(box_min[i] + h[i] * (j + 1));
        }
        ds[i].push_back(box_max[i]);
    }

    const double min_dis = voxel_resolution * voxel_resolution / 4;
    //    double min_dis = state.target_edge_len * state.target_edge_len;//epsilon*2
    for (int i = 0; i < ds[0].size(); i++) {
        for (int j = 0; j < ds[1].size(); j++) {
            for (int k = 0; k < ds[2].size(); k++) {
                if ((i == 0 || i == ds[0].size() - 1) && (j == 0 || j == ds[1].size() - 1) &&
                    (k == 0 || k == ds[2].size() - 1)) {
                    continue;
                }
                const Vector3d p(ds[0][i], ds[1][j], ds[2][k]);

                Eigen::Vector3d n;
                const double sqd = envelope.nearest_point(p, n);

                if (sqd < min_dis) {
                    continue;
                }
                points.push_back({{ds[0][i], ds[1][j], ds[2][k]}});
            }
        }
    }


    std::vector<delaunay::Point3D> unused_points;
    std::tie(unused_points, tets) = delaunay::delaunay3D(points);
    logger().info(
        "after delaunay tets.size() = {} | points.size() = {}",
        tets.size(),
        points.size());

    // Drop the points the tetrahedrization does not use, and renumber the tets onto what is
    // left. delaunay3D dedups its input (unique_points) and maps the tets it returns back to
    // ORIGINAL indices, so when two input points coincide exactly only one of the two indices
    // is ever referenced -- the other belongs to no tet. Left in, the caller builds a vertex
    // for it with an empty m_conn_tets that TetMesh::init() does not mark removed, and the
    // first get_vertices() calls tuple_from_vertex on it, which reads m_conn_tets[0] on an
    // empty vector. Release builds segfault; the assert that would name it is compiled out.
    // (TetMesh::init_with_isolated_vertices has always marked tet-less vertices removed;
    // plain init() never has.)
    //
    // Compacting rather than marking them removed, because the callers rely on vertex ids
    // being contiguous: they size their attribute vectors from points.size() and index them
    // by position.
    //
    // Found on Thingi10K 117682, which died 20 s in with SIGSEGV. Its simplified surface
    // carries exactly two coincident vertex pairs.
    {
        std::vector<size_t> remap(points.size(), std::numeric_limits<size_t>::max());
        for (const auto& t : tets) {
            for (const size_t v : t) remap[v] = 0;
        }
        size_t live = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            if (remap[i] == 0) {
                points[live] = points[i];
                remap[i] = live++;
            }
        }
        if (live != points.size()) {
            logger().info(
                "dropped {} delaunay point(s) that no tet uses (exactly coincident input)",
                points.size() - live);
            points.resize(live);
            for (auto& t : tets) {
                for (size_t& v : t) v = remap[v];
            }
        }
    }
}

} // namespace wmtk::utils
