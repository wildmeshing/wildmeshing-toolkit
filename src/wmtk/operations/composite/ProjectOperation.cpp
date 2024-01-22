#include "ProjectOperation.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>

#include <SimpleBVH/BVH.hpp>

namespace wmtk::operations::composite {

ProjectOperation::ProjectOperation(
    std::shared_ptr<Operation> main_op,
    const attribute::MeshAttributeHandle& project_to_mesh,
    attribute::MeshAttributeHandle& child_mesh_coordinates)
    : ProjectOperation(main_op, {std::make_pair(project_to_mesh, child_mesh_coordinates)})
{}


ProjectOperation::ProjectOperation(
    std::shared_ptr<Operation> main_op,
    const std::vector<MeshConstrainPair>& mesh_constaint_pairs)
    : AttributesUpdate(main_op->mesh())
    , m_main_op(main_op)
{
    for (auto& pair : mesh_constaint_pairs) {
        int64_t count = 0;
        int64_t index = 0;
        ConstAccessor<double> accessor = pair.first.mesh().create_accessor(pair.first.as<double>());

        const std::vector<Tuple>& facest =
            pair.first.mesh().get_all(pair.first.mesh().top_simplex_type());

        const int64_t dim = int64_t(pair.first.mesh().top_simplex_type()) + 1;

        Eigen::MatrixXd vertices(dim * facest.size(), accessor.dimension());
        Eigen::MatrixXi faces(facest.size(), dim);

        for (const auto& f : facest) {
            auto tmp = faces_single_dimension_tuples(
                pair.first.mesh(),
                simplex::Simplex(pair.first.mesh().top_simplex_type(), f),
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

        auto bvh = std::make_shared<SimpleBVH::BVH>();
        bvh->init(vertices, faces, 1e-10);

        m_bvh.emplace_back(pair.second, bvh);
    }
}

std::vector<simplex::Simplex> ProjectOperation::execute(const simplex::Simplex& simplex)
{
    // mesh has to be the same as the main_op mesh
    assert(&m_main_op->mesh() == &mesh());
    const auto main_simplices = (*m_main_op)(simplex);
    if (main_simplices.empty()) return {};
    assert(main_simplices.size() == 1);
    const auto main_tup = main_simplices.front().tuple();


    for (auto& pair : m_bvh) {
        const std::vector<Tuple> mapped_tuples_after =
            mesh().map_tuples(pair.first.mesh(), primitive_type(), {main_tup});

        if (mapped_tuples_after.empty()) continue;

        Accessor<double> accessor = pair.first.mesh().create_accessor(pair.first.as<double>());

        for (const auto& t : mapped_tuples_after) {
            auto p = accessor.const_vector_attribute(t);
            SimpleBVH::VectorMax3d nearest_point;
            double sq_dist;
            pair.second->nearest_facet(p, nearest_point, sq_dist);
            accessor.vector_attribute(t) = nearest_point;
        }
    }

    return main_simplices;
}

} // namespace wmtk::operations::composite
