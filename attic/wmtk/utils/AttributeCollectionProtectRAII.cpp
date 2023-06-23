
#include <wmtk/utils/AttributeCollectionProtectRAII.h>
#include <wmtk/AttributeCollection.hpp>

namespace wmtk {
AttributeCollectionProtectRAII::AttributeCollectionProtectRAII() = default;
AttributeCollectionProtectRAII::AttributeCollectionProtectRAII(
    AbstractAttributeCollection& attribute_collection)
    : _attribute_collection(&attribute_collection)
{
    if (_attribute_collection->is_in_protect()) {
        throw std::runtime_error(
            "Cannot protect an attribute collection that is already protected");
    }
    _attribute_collection->begin_protect();
}
std::optional<size_t> AttributeCollectionProtectRAII::release()
{
    std::optional<size_t> r;
    if (_attribute_collection != nullptr) {
        r = _attribute_collection->end_protect();
        _attribute_collection = nullptr;
    }
    return r;
}
AttributeCollectionProtectRAII::~AttributeCollectionProtectRAII()
{
    release();
}
AttributeCollectionProtectRAII::AttributeCollectionProtectRAII(AttributeCollectionProtectRAII&& o)
    : _attribute_collection(o._attribute_collection)
{
    o._attribute_collection = nullptr;
}
AttributeCollectionProtectRAII& AttributeCollectionProtectRAII::operator=(
    AttributeCollectionProtectRAII&& o)
{
    release();
    _attribute_collection = o._attribute_collection;
    o._attribute_collection = nullptr;
    return *this;
}
} // namespace wmtk
