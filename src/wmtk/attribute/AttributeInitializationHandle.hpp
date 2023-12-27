#pragma once
#include <memory>
#include <wmtk/operations/NewAttributeStrategy.hpp>
#include "MeshAttributeHandle.hpp"

namespace wmtk::operations {
class CollapseNewAttributeStrategy;
class SplitNewAttributeStrategy;
namespace tri_mesh {
template <typename T>
class BasicCollapseNewAttributeStrategy;
template <typename T>
class BasicSplitNewAttributeStrategy;
} // namespace tri_mesh
namespace tet_mesh {
template <typename T>
class BasicCollapseNewAttributeStrategy;
template <typename T>
class BasicSplitNewAttributeStrategy;
} // namespace tet_mesh
} // namespace wmtk::operations
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
