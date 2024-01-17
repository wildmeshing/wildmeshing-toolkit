#include "ProjectOperation.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>

#include <SimpleBVH/BVH.hpp>

namespace wmtk::operations::composite {

ProjectOperation::ProjectOperation(
    std::shared_ptr<Operation> main_op,
    const attribute::MeshAttributeHandle& project_to_mesh,
    Mesh& m,
    attribute::MeshAttributeHandle& child_mesh_coordinates)
    : AttributesUpdate(m)
    , m_main_op(main_op)
    , m_coordinates(child_mesh_coordinates.as<double>())
    , m_child_mesh(child_mesh_coordinates.mesh())
{
    int64_t count = 0;
    int64_t index = 0;

    ConstAccessor<double> accessor =
        project_to_mesh.mesh().create_accessor(project_to_mesh.as<double>());

    const std::vector<Tuple>& facest =
        project_to_mesh.mesh().get_all(project_to_mesh.mesh().top_simplex_type());

    const int64_t dim = int64_t(project_to_mesh.mesh().top_simplex_type()) + 1;

    Eigen::MatrixXd vertices(dim * facest.size(), accessor.dimension());
    Eigen::MatrixXi faces(facest.size(), dim);

    for (const auto& f : facest) {
        auto tmp = faces_single_dimension_tuples(
            project_to_mesh.mesh(),
            simplex::Simplex(project_to_mesh.mesh().top_simplex_type(), f),
            PrimitiveType::Vertex);

        assert(tmp.size() == dim);
        for (int64_t j = 0; j < tmp.size(); ++j) {
            auto p = accessor.const_vector_attribute(tmp[j]);
            faces(index, j) = count;
            vertices.row(dim * index + j) = p;

            ++count;
        }
        ++index;
    }

    m_bvh = std::make_shared<SimpleBVH::BVH>();
    m_bvh->init(vertices, faces, 1e-10);
}

std::vector<simplex::Simplex> ProjectOperation::execute(const simplex::Simplex& simplex)
{
    // mesh has to be the same as the main_op mesh
    assert(&m_main_op->mesh() == &mesh());
    const auto main_simplices = (*m_main_op)(simplex);
    if (main_simplices.empty()) return {};
    assert(main_simplices.size() == 1);
    const auto main_tup = main_simplices.front().tuple();


    const std::vector<Tuple> mapped_tuples_after =
        mesh().map_tuples(m_child_mesh, primitive_type(), {main_tup});

    if (mapped_tuples_after.empty()) return main_simplices;

    auto accessor = m_child_mesh.create_accessor(m_coordinates);

    for (const auto& t : mapped_tuples_after) {
        auto p = accessor.const_vector_attribute(t);
        SimpleBVH::VectorMax3d nearest_point;
        double sq_dist;
        m_bvh->nearest_facet(p, nearest_point, sq_dist);
        accessor.vector_attribute(t) = nearest_point;
    }

    return main_simplices;
}

} // namespace wmtk::operations::composite
