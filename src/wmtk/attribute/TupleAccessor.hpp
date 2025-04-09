#pragma once

#include "Accessor.hpp"


namespace wmtk::attribute {


    // A wrapper around standard accessor treat the data as a vector of Tuple objects
template <typename MeshType, int Dim = Eigen::Dynamic>
class TupleAccessor
{
public:
    TupleAccessor(MeshType& m, const TypedAttributeHandle<int64_t>& handle);
    TupleAccessor(const MeshType& m, const TypedAttributeHandle<int64_t>& handle);
    template <int Dim2>
    TupleAccessor(const Accessor<int64_t, MeshType, CachingAttribute<int64_t>, Dim2>& accessor);

    // Eigen::Map<VectorX<T>>
    template <int D = Dim>
    using MapResult = MapResult<Tuple, D>;
    // Eigen::Map<const VectorX<T>>
    template <int D = Dim>
    using ConstMapResult = ConstMapResult<Tuple, D>;


    const Tuple& const_scalar_attribute(const Tuple& t) const;
    Tuple& scalar_attribute(const Tuple& t);

    template <int D = Dim>
    ConstMapResult<D> const_vector_attribute(const Tuple& t) const;
    template <int D = Dim>
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
