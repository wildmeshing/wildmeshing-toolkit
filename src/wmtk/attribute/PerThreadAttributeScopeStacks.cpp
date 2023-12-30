#include "PerThreadAttributeScopeStacks.hpp"
#include <wmtk/utils/Rational.hpp>


namespace wmtk::attribute {
template class PerThreadAttributeScopeStacks<int64_t>;
template class PerThreadAttributeScopeStacks<double>;
template class PerThreadAttributeScopeStacks<char>;
template class PerThreadAttributeScopeStacks<Rational>;

} // namespace wmtk::attribute
