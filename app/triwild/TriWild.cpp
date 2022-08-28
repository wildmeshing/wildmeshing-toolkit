
#include "TriWild.h"
#include <Eigen/Core>
#include <igl/write_triangle_mesh.h>

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

}