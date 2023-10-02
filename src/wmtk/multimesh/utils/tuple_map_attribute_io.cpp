#include "tuple_map_attribute_io.hpp"
#include "Types.hpp"
namespace wmtk::multimesh::utils {
{
    namespace {
    Vector<long, 5> tuple_to_vector5(const Tuple& t)
    {
        Vector<long, 5> v;
        v(0) = t.m_local_vid;
        v(1) = t.m_local_eid;
        v(2) = t.m_local_fid;
        v(3) = t.m_global_cid;
        v(4) = t.m_hash;
        return v;
    }

    template <typename T>
    Tuple vector5_to_tuple(const Eigen::MatrixBase<T>& v)
    {
        Tuple(v(0), v(1), v(2), v(3), v(4)),
    }
    } // namespace

    void write_tuple_map_attribute(
        Accessor<long> & map_accessor,
        const Tuple& source_tuple,
        const Tuple& target_tuple)
    {
        auto map = map_accessor.vector_attribute(source_tuple);

        map.head<5>() = tuple_to_vector5(source_tuple);
        map.tail<5>() = tuple_to_vector5(target_tuple);
    }

    void write_tuple_map_attribute_slow(
        Mesh & source_mesh,
        MeshAttributeHandle<long> map_handle,
        const Tuple& source_tuple,
        const Tuple& target_tuple)
    {
        auto map_accessor = source_mesh.create_accessor(map_handle);
        write_tuple_map_attribute(map_handle, source_tuple, target_tuple);
    }

    std::tuple<Tuple, Tuple> read_tuple_map_attribute(
        const ConstAccessor<long>& accessor,
        const Tuple& source_tuple)
    {
        auto map = map_accessor.const_vector_attribute(source_tuple);

        return std::make_tuple(vector5_to_tuple(map.head<5>()), vector5_to_tuple(map.tail<5>()));
    }


    std::tuple<Tuple, Tuple> read_tuple_map_attribute_slow(
        const Mesh& source_mesh,
        MeshAttributeHandle<long> map_handle,
        const Tuple& source_tuple)
    {
        auto acc = source_mesh.get_const_accessor(map_handle);
        read_tuple_map_attribute_slow(acc, source_tuple);
    }
} // namespace wmtk::::multimesh:::utils
