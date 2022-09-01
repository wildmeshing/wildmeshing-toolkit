#include "TriWild.h"
#include <Eigen/Core>
#include <igl/write_triangle_mesh.h>
#include <wmtk/utils/AMIPS2D.h>
#include <igl/predicates/predicates.h>

namespace triwild {

void TriWild::create_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    // Register attributes
    p_vertex_attrs = &vertex_attrs;

    // Convert from eigen to internal representation (TODO: move to utils and remove it from all app)
    std::vector<std::array<size_t, 3>> tri(F.rows());
    
    for (int i = 0; i < F.rows(); i++) 
        for (int j = 0; j < 3; j++) 
            tri[i][j] = (size_t)F(i, j);
    
    // Initialize the trimesh class which handles connectivity
    wmtk::TriMesh::create_mesh(V.rows(), tri);

    // Save the vertex position in the vertex attributes
    for (unsigned i = 0; i<V.rows();++i)
        vertex_attrs[i].pos << V.row(i)[0], V.row(i)[1];
}

bool TriWild::write_mesh(std::string path)
{
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(vert_capacity(), 3);
    for (auto& t : get_vertices()) {
        auto i = t.vid(*this);
        V.row(i) = vertex_attrs[i].pos;
    }

    Eigen::MatrixXi F = Eigen::MatrixXi::Constant(tri_capacity(), 3, -1);
    for (auto& t : get_faces()) {
        auto i = t.fid(*this);
        auto vs = oriented_tri_vertices(t);
        for (int j = 0; j < 3; j++) {
            F(i, j) = vs[j].vid(*this);
        }
    }

    return igl::write_triangle_mesh(path, V, F);
}

double TriWild::get_quality(const Tuple& loc) const
{
    // Global ids of the vertices of the triangle
    auto its = oriented_tri_vids(loc);

    // Temporary variable to store the stacked coordinates of the triangle
    std::array<double, 6> T;
    auto energy = -1.;
    for (auto k = 0; k < 3; k++)
        for (auto j = 0; j < 2; j++) 
            T[k * 2 + j] = vertex_attrs[its[k]].pos[j];

    // Energy evaluation
    energy = wmtk::AMIPS2D_energy(T);

    // Filter for numerical issues
    if (std::isinf(energy) || std::isnan(energy)) 
        return MAX_ENERGY;

    return energy;
}

bool TriWild::is_inverted(const Tuple& loc) const
{
    auto vs = oriented_tri_vertices(loc);

    igl::predicates::exactinit();

    auto res = igl::predicates::orient2d(
        vertex_attrs[vs[0].vid(*this)].pos,
        vertex_attrs[vs[1].vid(*this)].pos,
        vertex_attrs[vs[2].vid(*this)].pos);

    return (res != igl::predicates::Orientation::POSITIVE);
}

}