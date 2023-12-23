#pragma once
#include <wmtk/operations/NewAttributeStrategy.hpp>
#include "MeshAttributeHandle.hpp"

namespace wmtk::attribute {

// a handle that wraps around the standard mesh attribute handle to enable convenient access to the
// the new attribute strategies. Can be implicitly converted to a mesh attribute handle
//
// (this is the non-templated part)
class AttributeInitializationHandleBase
{
public:
    AttributeInitializationHandleBase(
        std::shared_ptr<operations::SplitNewAttributeStrategy> split_strategy,
        std::shared_ptr<operations::CollapseNewAttributeStrategy> collapse_strategy);
    ~AttributeInitializationHandleBase();

private:
protected:
    std::shared_ptr<operations::SplitNewAttributeStrategy> m_split_strategy;

    // from here
    std::shared_ptr<operations::CollapseNewAttributeStrategy> m_collapse_strategy;
};
template <typename T>
class AttributeInitializationHandle : public MeshAttributeHandle<T>,
                                      public AttributeInitialiationHandleBase
{
public:
    AttributeInitializationHandle(
        const MeshAttributeHandle& h,
        std::shared_ptr<operations::SplitNewAttributeStrategy> a,
        std::shared_ptr<operations::SplitNewAttributeStrategy> b)
        : MeshAttributeHandle<T>(h)
        , AttributeInitialiationHandleBase(std::move(a), std::move(b))
    {}
    operator MeshAttributeHandle<T>() const { return *this; }

    auto& trimesh_standard_split_strategy()
    {
        auto ptr =
            std::dynamic_pointer_cast<operations::tri_mesh::BasicSplitNewAttributeStrategy<T>>(
                m_split_strategy);
        if (!bool(ptr)) {
            throw std::runtime_error(
                "Cannot call tri_basic_split_strategy because it wasn't cast properly");
        }
        return ptr;
    }

    auto trimesh_standard_collapse_strategy()
    {
        auto ptr =
            std::dynamic_pointer_cast<operations::tri_mesh::StandardCollapseNewAttributeStrategy>(
                m_collapse_strategy);
        if (!bool(ptr)) {
            throw std::runtime_error(
                "Cannot call tri_basic_split_strategy because it wasn't cast properly");
        }
        return ptr;
    }
}
} // namespace wmtk::attribute
