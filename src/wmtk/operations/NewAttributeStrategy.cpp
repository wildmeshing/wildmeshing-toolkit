#include "NewAttributeStrategy.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk::operations {

NewAttributeStrategy::~NewAttributeStrategy() = default;


const Mesh& NewAttributeStrategy::mesh() const
{
    return const_cast<const Mesh&>(const_cast<NewAttributeStrategy*>(this)->mesh());
}
} // namespace wmtk::operations
