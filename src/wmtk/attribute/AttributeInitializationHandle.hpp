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
                                      public AttributeInitializationHandleBase
{
public:
    AttributeInitializationHandle(
        const MeshAttributeHandle<T>& h,
        std::shared_ptr<operations::SplitNewAttributeStrategy> a,
        std::shared_ptr<operations::CollapseNewAttributeStrategy> b)
        : MeshAttributeHandle<T>(h)
        , AttributeInitializationHandleBase(std::move(a), std::move(b))
    {}
    operator MeshAttributeHandle<T>() const { return *this; }

    operations::tri_mesh::BasicSplitNewAttributeStrategy<T>& trimesh_standard_split_strategy();

    operations::tri_mesh::BasicCollapseNewAttributeStrategy<T>&
    trimesh_standard_collapse_strategy();

    operations::tet_mesh::BasicSplitNewAttributeStrategy<T>& tetmesh_standard_split_strategy();

    operations::tet_mesh::BasicCollapseNewAttributeStrategy<T>&
    tetmesh_standard_collapse_strategy();
};
} // namespace wmtk::attribute
