#include "hash.hpp"
std::size_t wmtk::hash<wmtk::attribute::AttributeHandle>::operator()(
    const wmtk::attribute::AttributeHandle& handle) const noexcept
{
    return handle.index;
    // return std::hash<long>{}(handle.index);
}
