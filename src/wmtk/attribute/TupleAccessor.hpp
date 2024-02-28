#pragma once

#include "Accessor.hpp"


namespace wmtk::attribute {


template <typename MeshType>
class TupleAccessor
{
public:
    TupleAccessor(MeshType& m, const TypedAttributeHandle<int64_t>& handle);
    TupleAccessor(const MeshType& m, const TypedAttributeHandle<int64_t>& handle);
    TupleAccessor(const Accessor<int64_t, MeshType>& accessor);

    // Eigen::Map<VectorX<T>>
    template <int D = Eigen::Dynamic>
    using MapResult = internal::MapResult<Tuple, D>;
    // Eigen::Map<const VectorX<T>>
    template <int D = Eigen::Dynamic>
    using ConstMapResult = internal::ConstMapResult<Tuple, D>;


    const Tuple& const_scalar_attribute(const Tuple& t) const;
    Tuple& scalar_attribute(const Tuple& t);

    template <int D = Eigen::Dynamic>
    ConstMapResult<D> const_vector_attribute(const Tuple& t) const;
    template <int D = Eigen::Dynamic>
    MapResult<D> vector_attribute(const Tuple& t);

    Eigen::Index dimension() const { return m_dimension; }

private:
    Accessor<int64_t, MeshType> m_base_accessor;
    Eigen::Index m_dimension;
};


template <typename MeshType>
TupleAccessor(MeshType& m, const TypedAttributeHandle<int64_t>& handle) -> TupleAccessor<MeshType>;
template <typename MeshType>
TupleAccessor(const MeshType& m, const TypedAttributeHandle<int64_t>& handle)
    -> TupleAccessor<MeshType>;

} // namespace wmtk::attribute
#include "TupleAccessor.hxx"
