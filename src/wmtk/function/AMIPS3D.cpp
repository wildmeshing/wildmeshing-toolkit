#include "AMIPS3D.hpp"
#include <wmtk/Types.hpp>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/AutoDiffUtils.hpp>
#include <wmtk/function/utils/amips.hpp>

namespace wmtk::function {
AMIPS3D::AMIPS3D(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle)
    : AMIPS(mesh, vertex_attribute_handle)
{
    assert(get_coordinate_attribute_handle().is_valid());
    // check the dimension of the position
    assert(embedded_dimension() == 3);
}


auto AMIPS3D::get_value_autodiff(const Tuple& simplex) const -> DScalar
{
    return function_eval<DScalar>(simplex);
}

template <typename T>
T AMIPS3D::function_eval(const Tuple& tuple) const
{
    // get_autodiff_value sets the autodiff size if necessary
    // get the pos coordinates of the triangle
    ConstAccessor<double> pos = mesh().create_const_accessor(get_coordinate_attribute_handle());

    auto tuple_value = pos.const_vector_attribute(tuple);
    Vector3<T> pos0;
    if constexpr (std::is_same_v<T, DScalar>) {
        pos0 = utils::as_DScalar<DScalar>(tuple_value);
    } else {
        pos0 = tuple_value;
    }
    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;

    Eigen::Vector3d pos2 = pos.const_vector_attribute(mesh().switch_tuples(tuple, {PE, PV}));
    Eigen::Vector3d pos1 = pos.const_vector_attribute(mesh().switch_tuples(tuple, {PV, PE}));

    // return the energy
    return utils::amips(pos0, pos1, pos2);
}


} // namespace wmtk::function
