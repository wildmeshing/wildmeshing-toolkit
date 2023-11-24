#include "HelperFunctions.hpp"

namespace wmtk::operations::utils {

Eigen::Vector3d nearest_point_to_edge(
    Mesh& mesh,
    const MeshAttributeHandle<double>& pos_handle,
    const Tuple& edge_tuple,
    const Tuple& vertex_tuple)
{
    Accessor<double> acc_pos = mesh.create_accessor(pos_handle);
    Eigen::Vector3d ev0, ev1, v, ret;
    ev0 = acc_pos.vector_attribute(edge_tuple);
    ev1 = acc_pos.vector_attribute(mesh.switch_vertex(edge_tuple));
    v = acc_pos.vector_attribute(vertex_tuple);
    if ((v - ev0).dot(ev1 - ev0) <= 0) {
        // case 1 nearest is ev0
        ret = ev0;
    } else if ((v - ev1).dot(ev0 - ev1) <= 0) {
        // case 2 nearest is ev1
        ret = ev1;
    } else {
        // case 3 nearset is on the edge
        double a0 = ev0.x() - ev1.x();
        double a1 = v.x() - ev0.x();
        double b0 = ev0.y() - ev1.y();
        double b1 = v.y() - ev0.y();
        double c0 = ev0.z() - ev1.z();
        double c1 = v.z() - ev0.z();
        double k = -(a0 * a1 + b0 * b1 + c0 * c1) / (a0 * a0 + b0 * b0 + c0 * c0);
        ret = ev0 + k * (ev1 - ev0);
    }
    return ret;
}

Eigen::Vector3d nearest_point_to_face(
    Mesh& mesh,
    const MeshAttributeHandle<double>& pos_handle,
    const Tuple& face_tuple,
    const Tuple& vertex_tuple)
{
    Accessor<double> acc_pos = mesh.create_accessor(pos_handle);
    Eigen::Vector3d fv0, fv1, fv2, v, ret;
    fv0 = acc_pos.vector_attribute(face_tuple);
    fv1 = acc_pos.vector_attribute(mesh.switch_vertex(face_tuple));
    fv2 = acc_pos.vector_attribute(mesh.switch_vertex(mesh.switch_edge(face_tuple)));
    v = acc_pos.vector_attribute(vertex_tuple);
    Eigen::Vector3d n = ((fv1 - fv0).cross(fv2 - fv0)).normalized();
    double len = (v - fv0).dot(n);
    ret = v - len * n;
    if (len < 0) {
        len *= -1.0;
    }

    // if this point is inside the triangle, then return it.
    int flag_sum = 0;
    if ((fv0 - ret).cross(fv1 - ret).dot(n) > 0) {
        flag_sum++;
    }
    if ((fv1 - ret).cross(fv2 - ret).dot(n) > 0) {
        flag_sum++;
    }
    if ((fv2 - ret).cross(fv0 - ret).dot(n) > 0) {
        flag_sum++;
    }
    if (flag_sum == 3) {
        return ret;
    } else {
        len = std::numeric_limits<double>::max();
    }


    Eigen::Vector3d candidate_v = nearest_point_to_edge(mesh, pos_handle, face_tuple, vertex_tuple);
    if ((v - candidate_v).norm() < len) {
        len = (v - candidate_v).norm();
        ret = candidate_v;
    }
    candidate_v =
        nearest_point_to_edge(mesh, pos_handle, mesh.switch_edge(face_tuple), vertex_tuple);
    if ((v - candidate_v).norm() < len) {
        len = (v - candidate_v).norm();
        ret = candidate_v;
    }
    candidate_v = nearest_point_to_edge(
        mesh,
        pos_handle,
        mesh.switch_edge(mesh.switch_vertex(face_tuple)),
        vertex_tuple);
    if ((v - candidate_v).norm() < len) {
        len = (v - candidate_v).norm();
        ret = candidate_v;
    }

    return ret;
}

bool is_invert(
    Mesh& mesh,
    const MeshAttributeHandle<double>& pos_handle,
    const Tuple& vertex_tuple,
    PrimitiveType type,
    Eigen::Vector3d original_pos)
{
    Accessor<double> acc_pos = mesh.create_accessor(pos_handle);
    switch (type) {
    case PrimitiveType::Face: {
        int sum = 0;
        const SimplicialComplex vertex_open_star =
            SimplicialComplex::open_star(mesh, Simplex::vertex(vertex_tuple));
        for (const Simplex& s : vertex_open_star.get_faces()) {
            const Tuple t = mesh.is_ccw(s.tuple()) ? s.tuple() : mesh.switch_vertex(s.tuple());
            const Simplex s_ccw(s.primitive_type(), t);

            std::vector<Tuple> face =
                simplex::faces_single_dimension(mesh, s_ccw, PrimitiveType::Vertex);
            Eigen::Vector3d p0 = acc_pos.vector_attribute(face[0]);
            Eigen::Vector3d p1 =
                acc_pos.vector_attribute(mesh.switch_vertex(mesh.switch_vertex(face[1])));
            Eigen::Vector3d p2 = acc_pos.vector_attribute(
                mesh.switch_vertex(mesh.switch_vertex(mesh.switch_edge(face[2]))));
            double sign = ((p1 - p0).cross(p2 - p0)).z();
            if (sign >= 0) {
                return true;
            }
        }
    } break;
    case PrimitiveType::Tetrahedron: {
        int sum = 0;
        const SimplicialComplex vertex_open_star =
            SimplicialComplex::open_star(mesh, Simplex::vertex(vertex_tuple));
        for (const Simplex& s : vertex_open_star.get_tetrahedra()) {
            // const Tuple t = mesh.is_ccw(s.tuple()) ? s.tuple() : mesh.switch_vertex(s.tuple());
            // const Simplex s_ccw(s.primitive_type(), t);
            // std::vector<Tuple> tet =
            //     simplex::faces_single_dimension(mesh, s_ccw, PrimitiveType::Vertex);
            std::vector<Tuple> tet =
                simplex::faces_single_dimension(mesh, s, PrimitiveType::Vertex);
            Eigen::Vector3d p0 = acc_pos.vector_attribute(tet[0]);
            Eigen::Vector3d p1 = acc_pos.vector_attribute(tet[1]);
            Eigen::Vector3d p2 = acc_pos.vector_attribute(tet[2]);
            Eigen::Vector3d p3 = acc_pos.vector_attribute(tet[3]);
            double sign_before =
                ((p1 - original_pos).cross((p2 - original_pos))).dot((p3 - original_pos));
            double sign_after = ((p1 - p0).cross((p2 - p0))).dot((p3 - p0));
            if (sign_before * sign_after <= 0) {
                return true;
            }
        }
    } break;
    default: break;
    }
    return false;
}

void optimize_position(
    Mesh& mesh,
    const MeshAttributeHandle<double> pos_handle,
    const Tuple& vertex_tuple,
    Eigen::Vector3d target_pos,
    Eigen::Vector3d last_best_pos,
    PrimitiveType type)
{
    Accessor<double> acc_pos = mesh.create_accessor(pos_handle);
    const Eigen::Vector3d original_p = last_best_pos;
    acc_pos.vector_attribute(vertex_tuple) = target_pos;
    auto final_p = acc_pos.vector_attribute(vertex_tuple);

    if (is_invert(mesh, pos_handle, vertex_tuple, type, original_p)) {
        for (int i = 0; i < 8; ++i) {
            if (is_invert(mesh, pos_handle, vertex_tuple, type, original_p)) {
                target_pos = final_p;
                final_p = (last_best_pos + target_pos) * 0.5;
            } else {
                last_best_pos = final_p;
                final_p = (last_best_pos + target_pos) * 0.5;
            }
        }
        final_p = last_best_pos;
        if (is_invert(mesh, pos_handle, vertex_tuple, type, original_p)) {
            final_p = original_p;
        }
    }
}

void push_offset(
    Mesh& mesh,
    const MeshAttributeHandle<double>& pos_handle,
    const Tuple& vertex_tuple,
    const Eigen::Vector3d& projection_pos,
    double len,
    PrimitiveType type,
    Eigen::Vector3d original_pos)
{
    Accessor<double> acc_pos = mesh.create_accessor(pos_handle);
    const Eigen::Vector3d original_p = acc_pos.const_vector_attribute(vertex_tuple);
    Eigen::Vector3d n = (acc_pos.vector_attribute(vertex_tuple) - projection_pos).normalized();
    Eigen::Vector3d target_p = projection_pos + n * len;
    Eigen::Vector3d last_best_p = original_p;

    optimize_position(mesh, pos_handle, vertex_tuple, target_p, last_best_p, type);
}

} // namespace wmtk::operations::utils
