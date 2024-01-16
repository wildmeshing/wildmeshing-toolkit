#include "AccessorBase.hpp"
#include <wmtk/utils/Rational.hpp>
#include "AttributeManager.hpp"
// #include "MeshAttributeHandle.hpp"
// #include "MeshAttributes.hpp"
#include "wmtk/Mesh.hpp"

namespace wmtk::attribute {

template <typename T>
const AttributeManager& AccessorBase<T>::attribute_manager() const
{
    return mesh().m_attribute_manager;
}

template <typename T>
AttributeManager& AccessorBase<T>::attribute_manager()
{
    return mesh().m_attribute_manager;
}

template AttributeManager& AccessorBase<char>::attribute_manager();
template AttributeManager& AccessorBase<int64_t>::attribute_manager();
template AttributeManager& AccessorBase<double>::attribute_manager();
template AttributeManager& AccessorBase<Rational>::attribute_manager();
template const AttributeManager& AccessorBase<char>::attribute_manager() const;
template const AttributeManager& AccessorBase<int64_t>::attribute_manager() const;
template const AttributeManager& AccessorBase<double>::attribute_manager() const;
template const AttributeManager& AccessorBase<Rational>::attribute_manager() const;
} // namespace wmtk::attribute
