#include <wmtk/serialization/AttributeCollectionRecorder.h>
#include <wmtk/AttributeCollection.hpp>


using namespace wmtk;

void AbstractAttributeCollection::add_recorder(AttributeCollectionRecorder* ptr)
{
    recorder_ptrs.emplace_back(ptr);
    ptr->record_initial_state();
}
void AbstractAttributeCollection::remove_recorder(AttributeCollectionRecorder* ptr)
{
    recorder_ptrs.remove(ptr);
}
void AbstractAttributeCollection::begin_protect()
{
    std::optional<size_t>& rollback_size_opt = m_rollback_size.local();
    const bool is_in_protected = rollback_size_opt.has_value();
    assert(is_in_protected == is_in_protect());
    assert(!is_in_protected);
    rollback_size_opt.emplace(size());
}
std::optional<size_t> AbstractAttributeCollection::end_protect()
{
    std::optional<size_t>& rollback_size_opt = m_rollback_size.local();
    const bool is_in_protected = rollback_size_opt.has_value();
    std::optional<size_t> ret;
    if (is_in_protect()) {
        for (auto recorder_ptr : recorder_ptrs) {
            ret = recorder_ptr->record();
        }
    }
    rollback_size_opt.reset();

    return ret;
}

bool AbstractAttributeCollection::is_in_protect() const
{
    const std::optional<size_t>& rollback_size_opt = m_rollback_size.local();
    return rollback_size_opt.has_value();
}
AbstractAttributeCollection::~AbstractAttributeCollection() = default;
AbstractAttributeCollection::AbstractAttributeCollection() = default;
