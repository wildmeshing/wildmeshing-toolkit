#pragma once

#include "TupleAccessor.hpp"

#include <Eigen/Dense>

namespace wmtk::attribute {

template <typename T>
class Accessor: public TupleAccessor<T> {
    public:
    using TupleAccessor<T>::TupleAccessor;
    using TupleAccessor<T>::scalar_attribute;
    using TupleAccessor<T>::const_scalar_attribute;
    using TupleAccessor<T>::vector_attribute;
    using TupleAccessor<T>::const_vector_attribute;
};

} // namespace wmtk
