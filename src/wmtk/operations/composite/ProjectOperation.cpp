#include "ProjectOperation.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>

#include <SimpleBVH/BVH.hpp>

namespace wmtk::operations::composite {
ProjectOperation::ProjectOperation(
    Mesh& m,
    std::shared_ptr<Operation> main_op,
    const TypedAttributeHandle<double>& coordinates,
    const TypedAttributeHandle<int64_t>& proj_tag,
    wmtk::PrimitiveType proj_type,
    int64_t proj_value)
    : AttributesUpdate(m)
    , m_main_op(main_op)
    , m_coordinates(coordinates)
    , m_tag(proj_tag)
    , m_tag_value(proj_value)
{
    int64_t count = 0;
    int64_t index = 0;

    ConstAccessor<double> accessor = mesh().create_accessor(m_coordinates);
    ConstAccessor<int64_t> tag_accessor = mesh().create_accessor(m_tag);

    const std::vector<Tuple>& facest = mesh().get_all(proj_type);
    const int64_t dim = int64_t(proj_type) + 1;

    Eigen::MatrixXd vertices(dim * facest.size(), accessor.dimension());
    Eigen::MatrixXi faces(facest.size(), dim);

    for (const auto& f : facest) {
        if (tag_accessor.const_scalar_attribute(f) != m_tag_value) continue;

        auto tmp = faces_single_dimension_tuples(
            mesh(),
            simplex::Simplex(proj_type, f),
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
    faces.conservativeResize(index, faces.cols());
    vertices.conservativeResize(2 * index, vertices.cols());


    m_bvh = std::make_shared<SimpleBVH::BVH>();
    m_bvh->init(vertices, faces, 1e-10);
}

std::vector<simplex::Simplex> ProjectOperation::execute(const simplex::Simplex& simplex)
{
    const auto main_simplices = (*m_main_op)(simplex);
    if (main_simplices.empty()) return {};
    assert(main_simplices.size() == 1);
    const auto main_tup = main_simplices.front().tuple();

    auto tag_accessor = mesh().create_accessor(m_tag);
    auto accessor = mesh().create_accessor(m_coordinates);

    if (tag_accessor.const_scalar_attribute(main_tup) != m_tag_value)
        return AttributesUpdate::execute(main_simplices.front());

    auto p = accessor.const_vector_attribute(main_tup);
    SimpleBVH::VectorMax3d nearest_point;
    double sq_dist;
    m_bvh->nearest_facet(p, nearest_point, sq_dist);
    accessor.vector_attribute(simplex.tuple()) = nearest_point;

    return AttributesUpdate::execute(main_simplices.front());
}

} // namespace wmtk::operations::composite