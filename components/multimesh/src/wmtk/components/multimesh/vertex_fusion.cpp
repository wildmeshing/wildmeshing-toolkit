#include "vertex_fusion.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/DisjointSet.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "from_facet_bijection.hpp"
#include <wmtk/Types.hpp>


namespace wmtk::components::multimesh {

std::shared_ptr<Mesh> vertex_fusion(const attribute::MeshAttributeHandle& attr, const std::string_view& name, double eps)
{
    Mesh& mesh = const_cast<Mesh&>(attr.mesh());
    // get mesh dimension and checks
    int64_t mesh_dim = mesh.top_cell_dimension();


    // TODO: a lot of matrix copies to remove

    Eigen::MatrixX<double> V;

    wmtk::utils::EigenMatrixWriter writer;
    mesh.serialize(writer);

    writer.get_position_matrix(V);
    assert(V.rows() == mesh.get_all(PrimitiveType::Vertex).size());

    // rescale to [0, 1]
    Eigen::MatrixXd V_rescale(V.rows(), V.cols());

    Eigen::RowVectorXd min_pos = V.colwise().minCoeff();


    Eigen::RowVectorXd max_pos = V_rescale.colwise().maxCoeff();

    Eigen::RowVectorXd range = max_pos - min_pos;
    V_rescale = V_rescale.array().rowwise() / range.array();

    MatrixXl S = writer.get_simplex_vertex_matrix();
    assert(S.rows() == mesh.get_all(mesh.top_simplex_type()).size());

    std::map<int64_t, int64_t> vertex_map;
    std::vector<size_t> vertex_roots;
    Eigen::MatrixXd V_new;
    {
        utils::DisjointSet ptt(V.rows());

        const double eps2 = eps * eps;
        // TOdo: use a KDTree / accelleration structure ofc
        for (int j = 0; j < V.rows(); ++j) {
            for (int k = 0; k < V.rows(); ++k) {
                if (eps == 0) {
                    if (V.row(j) == V.row(k)) {
                        ptt.merge(j, k);
                    }
                } else if ((V.row(j) - V.row(k)).squaredNorm() < eps2) {
                    ptt.merge(j, k);
                }
            }
        }

        std::map<int64_t, int64_t> root_indexer;
        vertex_roots = ptt.roots();
        for (size_t j = 0; j < vertex_roots.size(); ++j) {
            root_indexer[vertex_roots[j]] = j;
        }
        for (size_t j = 0; j < V.rows(); ++j) {
            vertex_map[j] = root_indexer[ptt.get_root(j)];
        }
        V_new = V(vertex_roots, Eigen::all);
    }


    wmtk::MatrixXl S_new = S.unaryExpr([&](int64_t index) { return vertex_map[index]; });


    std::shared_ptr<Mesh> root_mesh;
    switch (mesh_dim) {
    case 1: {
        auto fusion_mesh = std::make_shared<EdgeMesh>();
        fusion_mesh->initialize(S_new);
        root_mesh = fusion_mesh;
        break;
    }
    case 2: {
        auto fusion_mesh = std::make_shared<TriMesh>();
        fusion_mesh->initialize(S_new);
        root_mesh = fusion_mesh;
        break;
    }
    case 3: {
        auto fusion_mesh = std::make_shared<TetMesh>();
        fusion_mesh->initialize(S_new);
        root_mesh = fusion_mesh;
        break;
    }
    default: {
        throw std::runtime_error("mesh dimension not supported");
    }
    }
    Mesh& child = mesh;
    Mesh& parent = *root_mesh;

    from_facet_bijection(parent, child);
    mesh_utils::set_matrix_attribute(V_new, std::string(name), PrimitiveType::Vertex, *root_mesh);
    return root_mesh;
}
} // namespace wmtk::components::multimesh
