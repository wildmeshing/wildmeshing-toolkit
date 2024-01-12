#include <wmtk/Accessor.hpp>
#include <wmtk/Types.hpp>

// #define WMTK_DISABLE_COMPRESSED_MULTIMESH_TUPLE
namespace wmtk::multimesh::utils {
#if defined WMTK_DISABLE_COMPRESSED_MULTIMESH_TUPLE
constexpr static int64_t TUPLE_SIZE = 5; // in terms of int64_t
constexpr static int64_t GLOBAL_ID_INDEX = 3;
#else
constexpr static int64_t TUPLE_SIZE = 2; // in terms of int64_t
constexpr static int64_t GLOBAL_ID_INDEX = 1;
#endif
constexpr static int64_t DEFAULT_TUPLES_VALUES = -1;
constexpr static int64_t TWO_TUPLE_SIZE = TUPLE_SIZE * 2; // in terms of int64_t
using TupleVector = Vector<int64_t, TUPLE_SIZE>;
using TwoTupleVector = Vector<int64_t, TWO_TUPLE_SIZE>;


Vector<int64_t, TUPLE_SIZE> tuple_to_vector(const Tuple& t);
Tuple vector_to_tuple(const Eigen::Ref<const TupleVector>& v);

std::tuple<Tuple, Tuple> vectors_to_tuples(const Eigen::Ref<const TwoTupleVector>& v);
TwoTupleVector tuples_to_vectors(const Tuple& a, const Tuple& b);

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
