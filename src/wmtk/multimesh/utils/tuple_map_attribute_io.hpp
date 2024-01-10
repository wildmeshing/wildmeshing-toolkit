#include <wmtk/Accessor.hpp>
#include <wmtk/Types.hpp>

namespace wmtk::multimesh::utils {
#if defined WMTK_USE_COMPRESSED_TUPLE
    constexpr static int64_t TWO_TUPLE_SIZE = 4;
#else
    constexpr static int64_t TWO_TUPLE_SIZE = 10;
#endif
    constexpr static int64_t DEFAULT_TUPLES_VALUES = -1;

Vector<int64_t, 5> tuple_to_vector5(const Tuple& t);
Tuple vector5_to_tuple(const Vector5l& v);
Vector<int64_t, 2> tuple_to_vector2(const Tuple& t);
Tuple vector2_to_tuple(const Vector2l& v);

// utility functions to simplify how we encode 2-Tuple attributes as 10-int64_t attribute
void write_tuple_map_attribute(
    Accessor<int64_t>& map_accessor,
    const Tuple& source_tuple,
    const Tuple& target_tuple);

std::tuple<Tuple, Tuple> read_tuple_map_attribute(
    const ConstAccessor<int64_t>& accessor,
    const Tuple& source_tuple);


void symmetric_write_tuple_map_attributes(
    Accessor<int64_t>& a_to_b,
    Accessor<int64_t>& b_to_a,
    const Tuple& a_tuple,
    const Tuple& b_tuple);
void write_tuple_map_attribute_slow(
    Mesh& source_mesh,
    TypedAttributeHandle<int64_t> map_handle,
    const Tuple& source_tuple,
    const Tuple& target_tuple);


std::tuple<Tuple, Tuple> read_tuple_map_attribute_slow(
    const Mesh& source_mesh,
    TypedAttributeHandle<int64_t> map_handle,
    const Tuple& source_tuple);
} // namespace wmtk::multimesh::utils
