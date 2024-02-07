#pragma once
#include "ConstAccessor.hpp"

namespace wmtk::attribute {
template <typename T>
using MutableAccessor = TupleAccessor<T>;

} // namespace wmtk::attribute
