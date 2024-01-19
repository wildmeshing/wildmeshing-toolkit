#include "AttributeCacheData.hpp"
#include <wmtk/utils/Rational.hpp>
namespace wmtk {
template class AttributeCacheData<int64_t>;
template class AttributeCacheData<double>;
template class AttributeCacheData<char>;
template class AttributeCacheData<Rational>;
} // namespace wmtk
