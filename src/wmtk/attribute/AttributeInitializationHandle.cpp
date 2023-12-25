#include "AttributeInitializationHandle.hpp"
#include <wmtk/operations/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/SplitNewAttributeStrategy.hpp>
//#include <wmtk/operations/tet_mesh/BasicCollapseNewAttributeStrategy.hpp>
//#include <wmtk/operations/tet_mesh/BasicSplitNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/BasicCollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/BasicSplitNewAttributeStrategy.hpp>

namespace wmtk::attribute {
AttributeInitializationHandleBase::AttributeInitializationHandleBase(
    std::shared_ptr<operations::SplitNewAttributeStrategy> split_strategy,
    std::shared_ptr<operations::CollapseNewAttributeStrategy> collapse_strategy)
    : m_split_strategy(std::move(split_strategy))
    , m_collapse_strategy(std::move(collapse_strategy))
{}
AttributeInitializationHandleBase::~AttributeInitializationHandleBase() = default;

void AttributeInitializationHandleBase::swap_split_strategy(
    std::shared_ptr<operations::SplitNewAttributeStrategy>& other)
{
    m_split_strategy.swap(other);
}

void AttributeInitializationHandleBase::swap_collapse_strategy(
    std::shared_ptr<operations::CollapseNewAttributeStrategy>& other)
{
    m_collapse_strategy.swap(other);
}

template <typename T>
operations::tri_mesh::BasicSplitNewAttributeStrategy<T>&
AttributeInitializationHandle<T>::trimesh_standard_split_strategy()
{
    auto ptr = std::dynamic_pointer_cast<operations::tri_mesh::BasicSplitNewAttributeStrategy<T>>(
        m_split_strategy);
    if (!bool(ptr)) {
        throw std::runtime_error(
            "Cannot call tri_basic_split_strategy because it wasn't cast properly");
    }
    return *ptr;
}

template <typename T>
operations::tri_mesh::BasicCollapseNewAttributeStrategy<T>&
AttributeInitializationHandle<T>::trimesh_standard_collapse_strategy()
{
    auto ptr =
        std::dynamic_pointer_cast<operations::tri_mesh::BasicCollapseNewAttributeStrategy<T>>(
            m_collapse_strategy);
    if (!bool(ptr)) {
        throw std::runtime_error(
            "Cannot call tri_basic_split_strategy because it wasn't cast properly");
    }
    return *ptr;
}

// template <typename T>
// operations::tet_mesh::BasicSplitNewAttributeStrategy<T>&
// AttributeInitializationHandle<T>::trimesh_standard_split_strategy()
//{
//     auto ptr =
//     std::dynamic_pointer_cast<operations::tet_mesh::BasicSplitNewAttributeStrategy<T>>(
//         m_split_strategy);
//     if (!bool(ptr)) {
//         throw std::runtime_error(
//             "Cannot call tet_basic_split_strategy because it wasn't cast properly");
//     }
//     return *ptr;
// }

// template <typename T>
// operations::tet_mesh::BasicCollapseNewAttributeStrategy<T>&
// AttributeInitializationHandle<T>::trimesh_standard_collapse_strategy()
//{
//     auto ptr =
//         std::dynamic_pointer_cast<operations::tet_mesh::BasicCollapseNewAttributeStrategy<T>>(
//             m_collapse_strategy);
//     if (!bool(ptr)) {
//         throw std::runtime_error(
//             "Cannot call tet_basic_split_strategy because it wasn't cast properly");
//     }
//     return *ptr;
// }
template class AttributeInitializationHandle<double>;
template class AttributeInitializationHandle<long>;
template class AttributeInitializationHandle<char>;
template class AttributeInitializationHandle<Rational>;
} // namespace wmtk::attribute
