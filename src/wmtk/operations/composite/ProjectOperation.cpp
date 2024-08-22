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
{
    operation_name = "ProjectOperation";
}


ProjectOperation::ProjectOperation(
    std::shared_ptr<Operation> main_op,
    const std::vector<MeshConstrainPair>& mesh_constaint_pairs)
    : AttributesUpdate(main_op->mesh())
    , m_main_op(main_op)
{
    for (auto& pair : mesh_constaint_pairs) {
        int64_t count = 0;
        int64_t index = 0;

        const std::vector<Tuple>& facest =
            pair.first.mesh().get_all(pair.first.mesh().top_simplex_type());

        const int64_t dim = int64_t(pair.first.mesh().top_simplex_type()) + 1;

        Eigen::MatrixXd vertices(dim * facest.size(), pair.first.dimension());
        Eigen::MatrixXi faces(facest.size(), dim);

        // hugly copy paste
        if (pair.first.holds<double>()) {
            const attribute::Accessor<double> accessor =
                pair.first.mesh().create_const_accessor(pair.first.as<double>());

            for (const auto& f : facest) {
                auto tmp = faces_single_dimension_tuples(
                    pair.first.mesh(),
                    simplex::Simplex(pair.first.mesh(), pair.first.mesh().top_simplex_type(), f),
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
        } else {
            const attribute::Accessor<Rational> accessor =
                pair.first.mesh().create_const_accessor(pair.first.as<Rational>());


            for (const auto& f : facest) {
                auto tmp = faces_single_dimension_tuples(
                    pair.first.mesh(),
                    simplex::Simplex(pair.first.mesh(), pair.first.mesh().top_simplex_type(), f),
                    PrimitiveType::Vertex);

                assert(tmp.size() == dim);
                for (int64_t j = 0; j < tmp.size(); ++j) {
                    auto p = accessor.const_vector_attribute(tmp[j]).cast<double>();
                    faces(index, j) = count;
                    vertices.row(dim * index + j) = p;

                    ++count;
                }
                ++index;
            }
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

        if (pair.first.holds<double>()) {
            wmtk::attribute::Accessor<double> accessor =
                pair.first.mesh().create_accessor(pair.first.as<double>());

            for (const auto& t : mapped_tuples_after) {
                auto p = accessor.vector_attribute(t);
                SimpleBVH::VectorMax3d nearest_point;
                double sq_dist;
                pair.second->nearest_facet(p, nearest_point, sq_dist);
                p = nearest_point;
            }
        } else {
            assert((pair.first.holds<Rational>()));
            wmtk::attribute::Accessor<Rational> accessor =
                pair.first.mesh().create_accessor(pair.first.as<Rational>());

            for (const auto& t : mapped_tuples_after) {
                auto p_map = accessor.vector_attribute(t);
                const Eigen::Vector3d p = p_map.cast<double>();
                SimpleBVH::VectorMax3d nearest_point;
                double sq_dist;
                pair.second->nearest_facet(p, nearest_point, sq_dist);
                for (int64_t d = 0; d < pair.first.dimension(); ++d) {
                    p_map(d) = Rational(nearest_point[d], true);
                }
            }
        }
    }

    return main_simplices;
}

} // namespace wmtk::operations::composite
