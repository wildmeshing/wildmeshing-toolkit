#include <wmtk/serialization/AttributeCollectionRecorder.h>
#include <wmtk/AttributeCollection.hpp>


using namespace wmtk;

void AbstractAttributeCollection::add_recorder(AttributeCollectionRecorder* ptr) {

    recorder_ptrs.emplace_back(ptr);
    ptr->record_initial_state();
}
void AbstractAttributeCollection::remove_recorder(AttributeCollectionRecorder* ptr) {
    recorder_ptrs.remove(ptr);

}
void AbstractAttributeCollection::begin_protect()
{
    in_protected.local() = true;
    m_rollback_size.local() = size();
}
std::optional<size_t> AbstractAttributeCollection::end_protect()
{
    bool& is_in_protected = in_protected.local();
    if (is_in_protected) {
        for (auto recorder_ptr : recorder_ptrs) {
            return recorder_ptr->record();
        }
    }
    is_in_protected = false;

    return {};
}
AbstractAttributeCollection::~AbstractAttributeCollection() = default;
AbstractAttributeCollection::AbstractAttributeCollection() = default;
