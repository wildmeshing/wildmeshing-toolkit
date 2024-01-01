#pragma once
#include <memory>

#include "MeshAttributeHandle.hpp"

namespace wmtk::attribute {

// a handle that wraps around the standard mesh attribute handle to enable convenient access to the
// the new attribute strategies. Can be implicitly converted to a mesh attribute handle
//
// (this is the non-templated part)
class AttributeInitializationHandleBase
{
public:
    AttributeInitializationHandleBase();
    ~AttributeInitializationHandleBase();
};


template <typename T>
class AttributeInitializationHandle : public MeshAttributeHandle<T>,
                                      public AttributeInitializationHandleBase
{
public:
    AttributeInitializationHandle(const MeshAttributeHandle<T>& h)
        : MeshAttributeHandle<T>(h)
    {}
    operator MeshAttributeHandle<T>() const { return *this; }
};
} // namespace wmtk::attribute
