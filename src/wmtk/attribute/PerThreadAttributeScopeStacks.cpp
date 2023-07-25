#include "PerThreadAttributeScopeStacks.hpp"
#include <wmtk/utils/Rational.hpp>


namespace wmtk {
template class PerThreadAttributeScopeStacks<long>;
template class PerThreadAttributeScopeStacks<double>;
template class PerThreadAttributeScopeStacks<char>;
template class PerThreadAttributeScopeStacks<Rational>;

} // namespace wmtk
