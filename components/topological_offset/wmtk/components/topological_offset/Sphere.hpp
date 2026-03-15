#pragma once
#include <igl/barycentric_coordinates.h>
#include <igl/point_mesh_squared_distance.h>
#include <queue>
#include "TopoOffsetMesh.h"


namespace wmtk::components::topological_offset {


class Sphere
{
private:
    Vector3d m_c;
    double m_r;

public:
    /**
     * @brief given 3D tet, fit cube to tet (return center and side length)
     */
    static void fit_cube(
        const Vector3d& p1,
        const Vector3d& p2,
        const Vector3d& p3,
        const Vector3d& p4,
        Vector3d& c,
        double& l)
    {
        MatrixXd V(4, 3);
        V.row(0) = p1;
        V.row(1) = p2;
        V.row(2) = p3;
        V.row(3) = p4;
        Eigen::RowVectorXd mins = V.colwise().minCoeff();
        Eigen::RowVectorXd maxs = V.colwise().maxCoeff();
        double x_span = maxs(0) - mins(0);
        double y_span = maxs(1) - mins(1);
        double z_span = maxs(2) - mins(2);
        if (x_span > y_span && x_span > z_span) {
            l = x_span;
        } else if (y_span > z_span) {
            l = y_span;
        } else {
            l = z_span;
        }
        c(0) = (maxs(0) + mins(0)) / 2.0;
        c(1) = (maxs(1) + mins(1)) / 2.0;
        c(2) = (maxs(2) + mins(2)) / 2.0;
    }

    Sphere(const Vector3d& c, const double r)
        : m_c(c)
        , m_r(r)
    {}

    Sphere(const TopoOffsetMesh& mesh, const size_t t_id)
    {
        auto vs = mesh.oriented_tet_vids(t_id);
        double side_length;
        Vector3d center;
        fit_cube(
            mesh.m_vertex_attribute[vs[0]].m_posf,
            mesh.m_vertex_attribute[vs[1]].m_posf,
            mesh.m_vertex_attribute[vs[2]].m_posf,
            mesh.m_vertex_attribute[vs[3]].m_posf,
            center,
            side_length);
        m_c(0) = center(0);
        m_c(1) = center(1);
        m_c(2) = center(2);
        m_r = side_length * sqrt(3.0) / 2.0;
    }

    /**
     * @brief get radius of sphere
     */
    double radius() const { return m_r; }

    /**
     * @brief get center of sphere
     */
    Vector3d center() const { return m_c; }

    /**
     * @brief add refinements of sphere to given queue
     */
    void refine(std::queue<Sphere>& q) const
    {
        double c_off_l = m_r / (sqrt(3.0) * 2.0);
        double new_r = m_r / 2.0;
        q.emplace(m_c + (c_off_l * Vector3d(-1, -1, -1)), new_r);
        q.emplace(m_c + (c_off_l * Vector3d(-1, -1, 1)), new_r);
        q.emplace(m_c + (c_off_l * Vector3d(-1, 1, -1)), new_r);
        q.emplace(m_c + (c_off_l * Vector3d(-1, 1, 1)), new_r);
        q.emplace(m_c + (c_off_l * Vector3d(1, -1, -1)), new_r);
        q.emplace(m_c + (c_off_l * Vector3d(1, -1, 1)), new_r);
        q.emplace(m_c + (c_off_l * Vector3d(1, 1, -1)), new_r);
        q.emplace(m_c + (c_off_l * Vector3d(1, 1, 1)), new_r);
    }

    /**
     * @brief check if a sphere overlaps given tet
     */
    bool overlaps_tet(const TopoOffsetMesh& mesh, const size_t t_id) const
    {
        auto vs = mesh.oriented_tet_vids(t_id);
        MatrixXd V(4, 3);
        V.row(0) = mesh.m_vertex_attribute[vs[0]].m_posf;
        V.row(1) = mesh.m_vertex_attribute[vs[1]].m_posf;
        V.row(2) = mesh.m_vertex_attribute[vs[2]].m_posf;
        V.row(3) = mesh.m_vertex_attribute[vs[3]].m_posf;

        MatrixXi F(4, 3);
        F << 0, 2, 1, 0, 1, 3, 1, 2, 3, 0, 3, 2;

        MatrixXd P(1, 3);
        P.row(0) = m_c;

        // check if sphere center inside tet (if so, there is overlap)
        MatrixXd L;
        igl::barycentric_coordinates(P, V.row(0), V.row(1), V.row(2), V.row(3), L);
        bool in_tet = (L.array() >= 1e-6).all();
        if (in_tet) {
            return true;
        }

        // center not in tet, compute min distance to faces
        VectorXd sqr_dists;
        VectorXi I;
        MatrixXd C_closest;
        igl::point_mesh_squared_distance(P, V, F, sqr_dists, I, C_closest);
        return (sqr_dists(0) < (m_r * m_r));
    }
};


} // namespace wmtk::components::topological_offset