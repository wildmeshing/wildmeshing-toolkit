#include "AttributeScope.hpp"
namespace wmtk {

AttributeScope::AttributeScope(std::unique_ptr<AttributeScope>&& parent)
    : m_parent(parent)
{}

AttributeScope::AttributeScope(Mesh& mesh) {}

std::unique_ptr<AttributeScope> AttributeScope::pop_parent()
{
    return std::move(m_parent);
}
} // namespace wmtk
