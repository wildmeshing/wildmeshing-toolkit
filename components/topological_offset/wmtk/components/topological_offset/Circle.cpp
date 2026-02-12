#include "Circle.hpp"
// #include <igl/circumradius.h>
#include <igl/point_simplex_squared_distance.h>


namespace wmtk::components::topological_offset {


Circle::Circle(const TopoOffsetTriMesh& mesh, const size_t f_id)
{
    // // compute bounding circle for triangle
    // auto v_ids = mesh.oriented_tri_vids(f_id);
    // Vector2d p0 = mesh.m_vertex_attribute[v_ids[0]].m_posf;
    // Vector2d p1 = mesh.m_vertex_attribute[v_ids[1]].m_posf;
    // Vector2d p2 = mesh.m_vertex_attribute[v_ids[2]].m_posf;
    // Vector2d center;
    // circumcenter(p0, p1, p2, center);
    // double r = (center - p0).norm();

    // m_c(0) = center(0);
    // m_c(1) = center(1);
    // m_r = r;

    auto vs = mesh.oriented_tri_vids(f_id);
    double side_length;
    Vector2d center;
    fit_square(
        mesh.m_vertex_attribute[vs[0]].m_posf,
        mesh.m_vertex_attribute[vs[1]].m_posf,
        mesh.m_vertex_attribute[vs[2]].m_posf,
        center,
        side_length);
    m_c(0) = center(0);
    m_c(1) = center(1);
    m_r = side_length / sqrt(2.0);
}


void Circle::fit_square(
    const Vector2d& p1,
    const Vector2d& p2,
    const Vector2d& p3,
    Vector2d& c,
    double& l)
{
    MatrixXd V(3, 2);
    V.row(0) = p1;
    V.row(1) = p2;
    V.row(2) = p3;
    Eigen::RowVectorXd mins = V.colwise().minCoeff();
    Eigen::RowVectorXd maxs = V.colwise().maxCoeff();
    if ((maxs(0) - mins(0)) > (maxs(1) - mins(1))) { // x span bigger than y span
        l = maxs(0) - mins(0);
    } else {
        l = maxs(1) - mins(1);
    }
    c(0) = (maxs(0) + mins(0)) / 2.0;
    c(1) = (maxs(1) + mins(1)) / 2.0;
}


void Circle::refine(std::queue<Circle>& q) const
{
    double c_off_l = m_r / (2 * sqrt(2.0));
    double new_r = m_r / 2.0;
    q.emplace(m_c + (c_off_l * Vector2d(-1, -1)), new_r);
    q.emplace(m_c + (c_off_l * Vector2d(-1, 1)), new_r);
    q.emplace(m_c + (c_off_l * Vector2d(1, -1)), new_r);
    q.emplace(m_c + (c_off_l * Vector2d(1, 1)), new_r);
}


bool Circle::overlaps_tri(const TopoOffsetTriMesh& mesh, const size_t f_id) const
{
    auto vs = mesh.oriented_tri_vids(f_id);
    Vector2d p0 = mesh.m_vertex_attribute[vs[0]].m_posf;
    Vector2d p1 = mesh.m_vertex_attribute[vs[1]].m_posf;
    Vector2d p2 = mesh.m_vertex_attribute[vs[2]].m_posf;

    MatrixXd V(3, 2);
    V.row(0) = p0;
    V.row(1) = p1;
    V.row(2) = p2;
    MatrixXi F(1, 3);
    F << 0, 1, 2;
    double dist2;
    Vector2d closest_point;
    igl::point_simplex_squared_distance<2>(m_c, V, F, 0, dist2, closest_point);
    return (dist2 < (m_r * m_r));
}


} // namespace wmtk::components::topological_offset