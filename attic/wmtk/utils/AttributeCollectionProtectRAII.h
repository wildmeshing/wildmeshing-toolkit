
#pragma once
#include <optional>

namespace wmtk {

class AbstractAttributeCollection;

class AttributeCollectionProtectRAII
{
public:
    AttributeCollectionProtectRAII();
    AttributeCollectionProtectRAII(AbstractAttributeCollection& attribute_collection);
    AttributeCollectionProtectRAII(AttributeCollectionProtectRAII&& o);
    AttributeCollectionProtectRAII& operator=(AttributeCollectionProtectRAII&& o);
    ~AttributeCollectionProtectRAII();

    std::optional<size_t> release();

private:
    AbstractAttributeCollection* _attribute_collection = nullptr;
};

} // namespace wmtk
