#pragma once
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/Types.hpp>

#include "Dart.hpp"
namespace wmtk::autogen {
class SimplexDart
{
public:
    SimplexDart(wmtk::PrimitiveType simplex_type);

    // to avoid potential construction costs we have some singletons available
    static const SimplexDart& get_singleton(wmtk::PrimitiveType simplex_type);

    int8_t product(int8_t a, int8_t b) const;
    int8_t inverse(int8_t a) const;
    int8_t primitive_as_index(wmtk::PrimitiveType pt) const;
    int8_t identity() const;
    wmtk::Tuple tuple_from_valid_index(int64_t gid, int8_t valid_index) const;
    wmtk::Tuple update_tuple_from_valid_index(const Tuple& t, int8_t valid_index) const;

    wmtk::Tuple tuple_from_dart_tuple(const Dart& dart);
    Dart dart_from_tuple(const wmtk::Tuple& t) const;

    int8_t valid_index_from_tuple(const wmtk::Tuple& t) const;

    using binary_op_type = int8_t (*)(int8_t, int8_t);
    using unary_op_type = int8_t (*)(int8_t);
    using primitive_to_index_type = int8_t (*)(PrimitiveType);
    using nullary_op_type = int8_t (*)();

    size_t size() const;
    VectorX<int8_t>::ConstMapType valid_indices() const;


    // converts input valid_indx to the target mesh
    int8_t convert(int8_t valid_index, const SimplexDart& target) const;

private:
    const wmtk::PrimitiveType m_simplex_type;
    const binary_op_type m_product;
    const unary_op_type m_inverse;
    const primitive_to_index_type m_primitive_to_index;
    const nullary_op_type m_identity;
};
} // namespace wmtk::autogen
