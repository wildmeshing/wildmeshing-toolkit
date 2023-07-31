#pragma once

namespace wmtk {

// handle used to refer to a single checkpointed state of attributes
struct AttributesCheckpointHandle
{
    friend class PerThreadAttributesCheckpointHandler;
    AttributesCheckpointHandle();
    AttributesCheckpointHandle(long index);
    AttributesCheckpointHandle(const AttributesCheckpointHandle& o);
    AttributesCheckpointHandle(AttributesCheckpointHandle&& o);
    AttributesCheckpointHandle& operator=(const AttributesCheckpointHandle& o);
    AttributesCheckpointHandle& operator=(AttributesCheckpointHandle&& o);


private:
    long m_checkpoint_index = -1;
};
} // namespace wmtk
