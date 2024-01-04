#include "SeamlessConstraints.hpp"
#include <cassert>
#include <iostream>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
namespace wmtk::utils {

const std::array<Eigen::Matrix<double, 2, 2>, 4> rotation_matrix = {
    {(Eigen::Matrix<double, 2, 2>() << 1, 0, 0, 1).finished(),
     (Eigen::Matrix<double, 2, 2>() << 0, 1, -1, 0).finished(),
     (Eigen::Matrix<double, 2, 2>() << -1, 0, 0, -1).finished(),
     (Eigen::Matrix<double, 2, 2>() << 0, -1, 1, 0).finished()}};

simplex::Simplex get_pair_edge(
    const TriMesh& seamed_mesh,
    const TriMesh& cut_mesh,
    const simplex::Simplex& edge_simplex)
{
    assert(cut_mesh.is_boundary(edge_simplex));
    const simplex::Simplex edge_on_seamed_mesh = cut_mesh.map_to_parent(edge_simplex);
    auto edge_pair = seamed_mesh.map_to_child(cut_mesh, edge_on_seamed_mesh);
    assert(edge_pair.size() == 2);
    if (!wmtk::simplex::utils::SimplexComparisons::equal(cut_mesh, edge_simplex, edge_pair[0])) {
        return simplex::Simplex::edge(edge_pair[0].tuple());
    } else {
        return simplex::Simplex::edge(edge_pair[1].tuple());
    }
}

Eigen::Matrix<double, 2, 2> get_rotation_matrix(
    const TriMesh& cut_mesh,
    const TypedAttributeHandle<double>& uv_coordinate,
    const simplex::Simplex& edge_simplex,
    const simplex::Simplex& pair_edge_simplex)
{
    ConstAccessor<double> accessor = cut_mesh.create_accessor(uv_coordinate);
    Eigen::Vector2d e_ab =
        accessor.const_vector_attribute(cut_mesh.switch_vertex(edge_simplex.tuple())).head<2>() -
        accessor.const_vector_attribute(edge_simplex.tuple()).head<2>();
    Eigen::Vector2d e_dc =
        accessor.const_vector_attribute(pair_edge_simplex.tuple()).head<2>() -
        accessor.const_vector_attribute(cut_mesh.switch_vertex(pair_edge_simplex.tuple()))
            .head<2>();
    Eigen::Vector2d e_ab_perp;
    e_ab_perp(0) = -e_ab(1);
    e_ab_perp(1) = e_ab(0);
    double angle = atan2(-e_ab_perp.dot(e_dc), e_ab.dot(e_dc));
    int r = static_cast<int>(round(2 * angle / M_PI) + 2) % 4;

    return rotation_matrix[r];
}

bool check_constraints(
    const TriMesh& seamed_mesh,
    const TriMesh& cut_mesh,
    const TypedAttributeHandle<double>& uv_coordinate,
    double eps)
{
    ConstAccessor<double> accessor = cut_mesh.create_accessor(uv_coordinate);
    auto all_edges = cut_mesh.get_all(PrimitiveType::Edge);

    for (const Tuple& edge_tuple : all_edges) {
        simplex::Simplex edge_simplex = simplex::Simplex::edge(edge_tuple);
        if (!cut_mesh.is_boundary(edge_simplex)) {
            continue;
        }

        simplex::Simplex pair_edge_simplex = get_pair_edge(seamed_mesh, cut_mesh, edge_simplex);

        Eigen::Matrix<double, 2, 2> rot_mat =
            get_rotation_matrix(cut_mesh, uv_coordinate, edge_simplex, pair_edge_simplex);
        Eigen::Vector2d e_ab =
            accessor.const_vector_attribute(cut_mesh.switch_vertex(edge_simplex.tuple()))
                .head<2>() -
            accessor.const_vector_attribute(edge_simplex.tuple()).head<2>();
        Eigen::Vector2d e_cd =
            accessor.const_vector_attribute(cut_mesh.switch_vertex(pair_edge_simplex.tuple()))
                .head<2>() -
            accessor.const_vector_attribute(pair_edge_simplex.tuple()).head<2>();

        Eigen::Vector2d e_ab_rotated = rot_mat * e_ab;

        double error = (e_ab_rotated - e_cd).norm();

        if (error > eps) {
            std::cout << "error: " << error << std::endl;
            std::cout << "check constraints fails, error > eps(" << eps << ")" << std::endl;

            return false;
        }
    }
    std::cout << "check_constraints: all constraints are satisfied" << std::endl;
    return true;
}

} // namespace wmtk::utils
