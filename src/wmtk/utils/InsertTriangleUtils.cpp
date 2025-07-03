#include "InsertTriangleUtils.hpp"

#include <igl/remove_duplicate_vertices.h>
#include <map>

namespace wmtk {

void match_tet_faces_to_triangles(
    const TetMesh& m,
    const std::vector<std::array<size_t, 3>>& faces,
    tbb::concurrent_vector<bool>& is_matched,
    tbb::concurrent_map<std::array<size_t, 3>, std::vector<int>>& tet_face_tags)
{
    is_matched.resize(faces.size(), false);

    std::map<std::array<size_t, 3>, size_t> map_surface;
    for (size_t i = 0; i < faces.size(); i++) {
        auto f = faces[i];
        std::sort(f.begin(), f.end());
        map_surface[f] = i;
    }
    for (auto t : m.get_tets()) {
        auto vs = m.oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            std::array<size_t, 3> f = {
                {vs[(j + 1) % 4].vid(m), vs[(j + 2) % 4].vid(m), vs[(j + 3) % 4].vid(m)}};
            std::sort(f.begin(), f.end());
            auto it = map_surface.find(f);
            if (it != map_surface.end()) {
                auto fid = it->second;
                tet_face_tags[f].push_back(fid);
                is_matched[fid] = true;
            }
        }
    }
}


bool remove_duplicates(
    std::vector<Vector3d>& vertices,
    std::vector<std::array<size_t, 3>>& faces,
    const double epsilon)
{
    MatrixXd V_tmp(vertices.size(), 3), V_in;
    MatrixXi F_tmp(faces.size(), 3), F_in;
    for (int i = 0; i < vertices.size(); i++) {
        V_tmp.row(i) = vertices[i];
    }
    for (int i = 0; i < faces.size(); i++) {
        F_tmp.row(i) << faces[i][0], faces[i][1], faces[i][2]; // note: using int here
    }

    //
    VectorXi IV, _;
    igl::remove_duplicate_vertices(V_tmp, F_tmp, epsilon, V_in, IV, _, F_in);
    // rotate faces s.th. lowest ID comes first
    for (int i = 0; i < F_in.rows(); i++) {
        int j_min = 0;
        for (int j = 1; j < 3; j++) {
            if (F_in(i, j) < F_in(i, j_min)) {
                j_min = j;
            }
        }
        if (j_min == 0) {
            continue;
        }
        int v0_id = F_in(i, j_min);
        int v1_id = F_in(i, (j_min + 1) % 3);
        int v2_id = F_in(i, (j_min + 2) % 3);
        F_in.row(i) << v0_id, v1_id, v2_id;
    }
    F_tmp.resize(0, 0);
    VectorXi IF;
    igl::unique_rows(F_in, F_tmp, IF, _);
    F_in = F_tmp;

    if (V_in.rows() == 0 || F_in.rows() == 0) {
        return false;
    }

    logger().info("remove duplicates: ");
    logger().info("#v: {} -> {}", vertices.size(), V_in.rows());
    logger().info("#f: {} -> {}", faces.size(), F_in.rows());

    std::vector<Vector3d> out_vertices;
    std::vector<std::array<size_t, 3>> out_faces;
    out_vertices.resize(V_in.rows());
    out_faces.reserve(F_in.rows());
    //    old_input_tags = input_tags;
    //    input_tags.clear();
    for (int i = 0; i < V_in.rows(); i++) {
        out_vertices[i] = V_in.row(i);
    }

    for (int i = 0; i < F_in.rows(); i++) {
        if (F_in(i, 0) == F_in(i, 1) || F_in(i, 0) == F_in(i, 2) || F_in(i, 2) == F_in(i, 1)) {
            continue;
        }
        if (i > 0 && (F_in(i, 0) == F_in(i - 1, 0) && F_in(i, 1) == F_in(i - 1, 2) &&
                      F_in(i, 2) == F_in(i - 1, 1))) {
            continue;
        }
        // check area
        Vector3d u = V_in.row(F_in(i, 1)) - V_in.row(F_in(i, 0));
        Vector3d v = V_in.row(F_in(i, 2)) - V_in.row(F_in(i, 0));
        Vector3d area = u.cross(v);
        if (area.norm() / 2 <= epsilon) {
            continue;
        }
        out_faces.push_back({{(size_t)F_in(i, 0), (size_t)F_in(i, 1), (size_t)F_in(i, 2)}});
        //        input_tags.push_back(old_input_tags[i]);
    }

    vertices = out_vertices;
    faces = out_faces;

    return true;
}

} // namespace wmtk