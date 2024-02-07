#pragma once

#include "TupleAccessor.hpp"

#include <Eigen/Dense>

namespace wmtk::attribute {

template <typename T>
class Accessor: public TupleAccessor<T> {};

} // namespace wmtk
