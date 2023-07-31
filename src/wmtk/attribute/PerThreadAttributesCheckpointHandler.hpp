#pragma once

#include <tbb/enumerable_thread_specific.h>
#include "AttributesCheckpointHandle.hpp"

namespace wmtk {
class PerThreadAttributesCheckpointHandler
{
public:
    PerThreadAttributesCheckpointHandler();
    PerThreadAttributesCheckpointHandler(const PerThreadAttributesCheckpointHandler&);
    PerThreadAttributesCheckpointHandler(PerThreadAttributesCheckpointHandler&&);
    PerThreadAttributesCheckpointHandler& operator=(PerThreadAttributesCheckpointHandler&&);
    PerThreadAttributesCheckpointHandler& operator=(const PerThreadAttributesCheckpointHandler&);
    ~PerThreadAttributesCheckpointHandler();
    AttributesCheckpointHandle current_checkpoint() const;
    void push_checkpoint();
    void pop_checkpoint();

private:
    mutable tbb::enumerable_thread_specific<long> m_current_index;
};
} // namespace wmtk
