#include "hash.hpp"
std::size_t std::hash<wmtk::attribute::AttributeHandle>::operator()(
    const wmtk::attribute::AttributeHandle& handle) const noexcept
{
    return handle.index;
    // return std::hash<long>{}(handle.index);
}
