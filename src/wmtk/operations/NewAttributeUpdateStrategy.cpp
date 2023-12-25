#include "NewAttributeUpdateStrategy.hpp"
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>


namespace wmtk::operations {

NewAttributeUpdateStrategyBase::std::vector<Tuple>
get_parent_simplices(const Mesh& m, const Simplex& s, PrimitiveType pt)
{
    const PrimitiveType spt = s.primitive_type();

    if (spt < pt) { // simplex is a face of the parent

        return simplex::cofaces_single_dimension_tuples(mesh, s, pt);
    } else if (spt > pt) {
        return simplex::faces_single_dimension_tuples(mesh, s, pt);
    } else {
        return s.tuple();
    }
}

template <typename MyType, typename ParentType>
auto SingleNewAttributeUpdateStrategy<T>::read_parent_values(
    const simplex::Simplex& my_simplex) const -> ParentMatType
{
    auto simps = get_parent_simplices
}

template <typename T>
PrimitiveType NewAttributeUpdateStrategy<T>::parent_primitive_type() const
{
    return m_parent_handle.primitive_type();
}

template <typename T>
PrimitiveType NewAttributeUpdateStrategy<T>::primitive_type() const
{
    return m_handle.primitive_type();
}
template class NewAttributeUpdateStrategy<double>;
template class NewAttributeUpdateStrategy<long>;
template class NewAttributeUpdateStrategy<char>;
template class NewAttributeUpdateStrategy<Rational>;
} // namespace wmtk::operations
