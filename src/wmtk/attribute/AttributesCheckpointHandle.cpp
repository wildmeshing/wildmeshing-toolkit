
#include "AttributesCheckpointHandle.hpp"

namespace wmtk {
    AttributesCheckpointHandle::AttributesCheckpointHandle() = default;
    AttributesCheckpointHandle::AttributesCheckpointHandle(long index): m_checkpoint_index(index) {}
    AttributesCheckpointHandle::AttributesCheckpointHandle(const AttributesCheckpointHandle& o) = default;
    AttributesCheckpointHandle::AttributesCheckpointHandle(AttributesCheckpointHandle&& o) = default;
    AttributesCheckpointHandle& AttributesCheckpointHandle::operator=(const AttributesCheckpointHandle& o) = default;
    AttributesCheckpointHandle& AttributesCheckpointHandle::operator=(AttributesCheckpointHandle&& o) = default;
}
