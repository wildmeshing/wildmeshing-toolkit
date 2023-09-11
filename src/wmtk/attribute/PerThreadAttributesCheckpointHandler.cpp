#include "PerThreadAttributesCheckpointHandler.hpp"
#include <cassert>
namespace wmtk {
PerThreadAttributesCheckpointHandler::PerThreadAttributesCheckpointHandler() = default;
PerThreadAttributesCheckpointHandler::~PerThreadAttributesCheckpointHandler() = default;

PerThreadAttributesCheckpointHandler::PerThreadAttributesCheckpointHandler(
    const PerThreadAttributesCheckpointHandler&) = default;

PerThreadAttributesCheckpointHandler::PerThreadAttributesCheckpointHandler(
    PerThreadAttributesCheckpointHandler&&) = default;

PerThreadAttributesCheckpointHandler& PerThreadAttributesCheckpointHandler::operator=(
    const PerThreadAttributesCheckpointHandler&) = default;

PerThreadAttributesCheckpointHandler& PerThreadAttributesCheckpointHandler::operator=(
    PerThreadAttributesCheckpointHandler&&) = default;

AttributesCheckpointHandle PerThreadAttributesCheckpointHandler::current_checkpoint() const
{
    bool exists;
    long& l = m_current_index.local(exists);
    if (!exists) {
        l = -1;
    }
    return AttributesCheckpointHandle{l};
}
void PerThreadAttributesCheckpointHandler::push_checkpoint()
{
    bool exists;
    long& l = m_current_index.local(exists);
    if (exists) {
        l++;
    } else {
        l = 0;
    }
}
void PerThreadAttributesCheckpointHandler::pop_checkpoint()
{
    bool exists;
    long& l = m_current_index.local(exists);
    assert(exists);
    l--;
}

} // namespace wmtk
