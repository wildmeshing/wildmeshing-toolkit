#include "Envelope.hpp"
#include "mesh_AABB.h"

#include <geogram/basic/attributes.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/geometry_nd.h>
#include <geogram/mesh/mesh_geometry.h>
#include <wmtk/utils/Logger.hpp>

namespace {

void to_geogram_mesh(
    const std::vector<Eigen::Vector3d>& V,
    const std::vector<Eigen::Vector3i>& F,
    GEO::Mesh& M)
{
    static std::once_flag once_flag;
    std::call_once(once_flag, []() { GEO::initialize(); });

    M.clear();
    // Setup vertices
    M.vertices.create_vertices((int)V.size());
    for (int i = 0; i < (int)M.vertices.nb(); ++i) {
        GEO::vec3& p = M.vertices.point(i);
        p[0] = V[i][0];
        p[1] = V[i][1];
        p[2] = V[i][2];
    }
    {
        GEO::Attribute<int> indices(M.vertices.attributes(), "vertex_id");
        for (int i = 0; i < V.size(); i++) indices[i] = i;
    }

    M.facets.create_triangles((int)F.size());
    // Setup faces
    for (int i = 0; i < F.size(); i++)
        for (int j = 0; j < 3; j++) M.facets.set_vertex(i, j, F[i][j]);
    M.facets.connect();
    GEO::Attribute<int> indices(M.facets.attributes(), "facet_id");
    for (int i = 0; i < F.size(); i++) indices[i] = i;
}

} // namespace

namespace wmtk {

// From TetWild
void sampleTriangle(
    const std::array<GEO::vec3, 3>& vs,
    std::vector<GEO::vec3>& ps,
    const double sampling_dist)
{
    double sqrt3_2 = std::sqrt(3) / 2;

    std::array<double, 3> ls;
    for (int i = 0; i < 3; i++) {
        ls[i] = GEO::length2(vs[i] - vs[(i + 1) % 3]);
    }
    auto min_max = std::minmax_element(ls.begin(), ls.end());
    int min_i = min_max.first - ls.begin();
    int max_i = min_max.second - ls.begin();
    double N = sqrt(ls[max_i]) / sampling_dist;
    if (N <= 1) {
        for (int i = 0; i < 3; i++) ps.push_back(vs[i]);
        return;
    }
    if (N == int(N)) N -= 1;

    GEO::vec3 v0 = vs[max_i];
    GEO::vec3 v1 = vs[(max_i + 1) % 3];
    GEO::vec3 v2 = vs[(max_i + 2) % 3];

    GEO::vec3 n_v0v1 = GEO::normalize(v1 - v0);
    for (int n = 0; n <= N; n++) {
        ps.push_back(v0 + n_v0v1 * sampling_dist * n);
    }
    ps.push_back(v1);

    double h = GEO::distance(GEO::dot((v2 - v0), (v1 - v0)) * (v1 - v0) / ls[max_i] + v0, v2);
    int M = h / (sqrt3_2 * sampling_dist);
    if (M < 1) {
        ps.push_back(v2);
        return;
    }

    GEO::vec3 n_v0v2 = GEO::normalize(v2 - v0);
    GEO::vec3 n_v1v2 = GEO::normalize(v2 - v1);
    auto sin_v0 = GEO::length(GEO::cross((v2 - v0), (v1 - v0))) /
                  (GEO::distance(v0, v2) * GEO::distance(v0, v1));
    auto tan_v0 = GEO::length(GEO::cross((v2 - v0), (v1 - v0))) / GEO::dot((v2 - v0), (v1 - v0));
    auto tan_v1 = GEO::length(GEO::cross((v2 - v1), (v0 - v1))) / GEO::dot((v2 - v1), (v0 - v1));
    auto sin_v1 = GEO::length(GEO::cross((v2 - v1), (v0 - v1))) /
                  (GEO::distance(v1, v2) * GEO::distance(v0, v1));

    for (int m = 1; m <= M; m++) {
        int n = sqrt3_2 / tan_v0 * m + 0.5;
        int n1 = sqrt3_2 / tan_v0 * m;
        if (m % 2 == 0 && n == n1) {
            n += 1;
        }
        GEO::vec3 v0_m = v0 + m * sqrt3_2 * sampling_dist / sin_v0 * n_v0v2;
        GEO::vec3 v1_m = v1 + m * sqrt3_2 * sampling_dist / sin_v1 * n_v1v2;
        if (GEO::distance(v0_m, v1_m) <= sampling_dist) break;

        double delta_d = ((n + (m % 2) / 2.0) - m * sqrt3_2 / tan_v0) * sampling_dist;
        GEO::vec3 v = v0_m + delta_d * n_v0v1;
        int N1 = GEO::distance(v, v1_m) / sampling_dist;
        for (int i = 0; i <= N1; i++) {
            ps.push_back(v + i * n_v0v1 * sampling_dist);
        }
    }
    ps.push_back(v2);

    // sample edges
    N = sqrt(ls[(max_i + 1) % 3]) / sampling_dist;
    if (N > 1) {
        if (N == int(N)) N -= 1;
        GEO::vec3 n_v1v2 = GEO::normalize(v2 - v1);
        for (int n = 1; n <= N; n++) {
            ps.push_back(v1 + n_v1v2 * sampling_dist * n);
        }
    }

    N = sqrt(ls[(max_i + 2) % 3]) / sampling_dist;
    if (N > 1) {
        if (N == int(N)) N -= 1;
        GEO::vec3 n_v2v0 = GEO::normalize(v0 - v2);
        for (int n = 1; n <= N; n++) {
            ps.push_back(v2 + n_v2v0 * sampling_dist * n);
        }
    }
}


inline void get_point_facet_nearest_point(
    const GEO::Mesh& M,
    const GEO::vec3& p,
    GEO::index_t f,
    GEO::vec3& nearest_p,
    double& squared_dist)
{
    using namespace GEO;
    geo_debug_assert(M.facets.nb_vertices(f) == 3);
    index_t c = M.facets.corners_begin(f);
    const vec3& p1 = Geom::mesh_vertex(M, M.facet_corners.vertex(c));
    ++c;
    const vec3& p2 = Geom::mesh_vertex(M, M.facet_corners.vertex(c));
    ++c;
    const vec3& p3 = Geom::mesh_vertex(M, M.facet_corners.vertex(c));
    double lambda1, lambda2, lambda3; // barycentric coords, not used.
    squared_dist =
        Geom::point_triangle_squared_distance(p, p1, p2, p3, nearest_p, lambda1, lambda2, lambda3);
}

void SampleEnvelope::init(
    const std::vector<Eigen::Vector3d>& V,
    const std::vector<Eigen::Vector3i>& F,
    const double _eps)
{
    if (use_exact) return exact_envelope.init(V, F, _eps);

    eps2 = _eps * _eps;
    sampling_dist = std::sqrt(eps2);

    geo_polyhedron_ptr_ = std::make_unique<GEO::Mesh>();
    to_geogram_mesh(V, F, *geo_polyhedron_ptr_);
    geo_tree_ptr_ = std::make_unique<GEO::MeshFacetsAABBWithEps>(*geo_polyhedron_ptr_, true);

    geo_vertex_ind.resize(V.size());
    GEO::Attribute<int> original_indices(geo_polyhedron_ptr_->vertices.attributes(), "vertex_id");
    for (int i = 0; i < original_indices.size(); i++) geo_vertex_ind[i] = original_indices[i];

    geo_face_ind.resize(F.size());
    GEO::Attribute<int> face_indices(geo_polyhedron_ptr_->facets.attributes(), "facet_id");
    for (int i = 0; i < face_indices.size(); i++) geo_face_ind[i] = face_indices[i];
}

bool SampleEnvelope::is_outside(const Eigen::Vector3d& pts) const
{
    if (use_exact) return exact_envelope.is_outside(pts);
    auto dist2 = geo_tree_ptr_->squared_distance(GEO::vec3(pts[0], pts[1], pts[2]));
    return (dist2 > eps2);
}

bool SampleEnvelope::is_outside(const std::array<Eigen::Vector3d, 3>& tri) const
{
    if (use_exact) return exact_envelope.is_outside(tri);
    std::array<GEO::vec3, 3> vs = {
        {GEO::vec3(tri[0][0], tri[0][1], tri[0][2]),
         GEO::vec3(tri[1][0], tri[1][1], tri[1][2]),
         GEO::vec3(tri[2][0], tri[2][1], tri[2][2])}};
    static thread_local std::vector<GEO::vec3> ps;
    ps.clear();


    sampleTriangle(vs, ps, sampling_dist);

    size_t num_queries = 0;
    size_t num_samples = ps.size();

    GEO::vec3 nearest_point;
    double sq_dist = std::numeric_limits<double>::max();
    GEO::index_t prev_facet = GEO::NO_FACET;
    int cnt = 0;
    const unsigned int ps_size = ps.size();
    for (unsigned int i = ps_size / 2; i < ps.size();
         i = (i + 1) % ps_size) { // check from the middle
        GEO::vec3& current_point = ps[i];
        if (prev_facet != GEO::NO_FACET) {
            get_point_facet_nearest_point(
                *geo_polyhedron_ptr_,
                current_point,
                prev_facet,
                nearest_point,
                sq_dist);
        }
        if (sq_dist > eps2) {
            geo_tree_ptr_->facet_in_envelope_with_hint(
                current_point,
                eps2,
                prev_facet,
                nearest_point,
                sq_dist);
        }
        ++num_queries;
        if (sq_dist > eps2) {
            wmtk::logger().trace("num_queries {} / {}", num_queries, num_samples);
            return true;
        }
        cnt++;
        if (cnt >= ps_size) break;
    }


    wmtk::logger().trace("num_queries {} / {}", num_queries, num_samples);
    return false;
}

double SampleEnvelope::nearest_point(const Eigen::Vector3d& pts, Eigen::Vector3d& result) const
{
    auto dist = 0.;
    GEO::vec3 current_point(pts[0], pts[1], pts[2]);
    GEO::vec3 r;
    geo_tree_ptr_->nearest_facet(current_point, r, dist);
    result << r.x, r.y, r.z;
    return dist;
}


} // namespace wmtk