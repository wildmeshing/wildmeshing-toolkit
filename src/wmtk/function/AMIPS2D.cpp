#include "AMIPS2D.hpp"
#include <wmtk/Types.hpp>
#include <wmtk/function/utils/AutoDiffUtils.hpp>
#include <wmtk/function/utils/amips.hpp>


namespace wmtk::function {
AMIPS2D::AMIPS2D(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle)
    : AMIPS(mesh, vertex_attribute_handle)
{
    assert(get_coordinate_attribute_handle().is_valid());
    // check the dimension of the position
    assert(embedded_dimension() == 2);
}


auto AMIPS2D::get_value_autodiff(const Tuple& simplex) const -> DScalar
{
    return function_eval<DScalar>(simplex);
}

template <typename T>
T AMIPS2D::function_eval(const Tuple& tuple) const
{
    // get_autodiff_value sets the autodiff size if necessary
    // get the uv coordinates of the triangle
    ConstAccessor<double> pos = mesh().create_const_accessor(get_coordinate_attribute_handle());

    auto tuple_value = pos.const_vector_attribute(tuple);
    Vector2<T> uv0;
    if constexpr (std::is_same_v<T, DScalar>) {
        uv0 = utils::as_DScalar<DScalar>(tuple_value);
    } else {
        uv0 = tuple_value;
    }
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;

    Eigen::Vector2d uv2 = pos.const_vector_attribute(mesh().switch_tuples(tuple, {PE, PV}));
    Eigen::Vector2d uv1 = pos.const_vector_attribute(mesh().switch_tuples(tuple, {PV, PE}));

    // return the energy
    return utils::amips(uv0, uv1, uv2);
}

} // namespace wmtk::function
