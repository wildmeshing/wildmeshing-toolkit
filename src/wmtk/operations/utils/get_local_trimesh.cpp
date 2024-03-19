// #include "get_local_trimesh.hpp"
#include <wmtk/simplex/top_dimension_cofaces_iterable.hpp>
#include <wmtk/TriMesh.hpp>
namespace wmtk::operations::utils {
    std::tuple<Eigen::MatrixXi, Eigen::MatrixXd, std::vector<int64_t> >
    get_local_trimesh(const wmtk::TriMesh& mesh, const wmtk::simplex::Simplex& simplex)
    {
        // auto pos_handle = mesh.get_attribute_handle<double>("vertex", PrimitiveType::Vertex);
        // const auto cofaces = wmtk::simplex::top_dimension_cofaces_iterable(mesh, simplex);
        return std::make_tuple(Eigen::MatrixXi(), Eigen::MatrixXd(), std::vector<int64_t>());
        
    }
} // namespace wmtk::operations::utils