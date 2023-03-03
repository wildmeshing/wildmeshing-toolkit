#include <wmtk/TriMesh.h>
#include <wmtk/utils/AttributeRecorder.h>
#include <wmtk/utils/Hdf5Utils.h>
#include <wmtk/utils/OperationLogger.h>
#include <ostream>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/OperationRecordingDataTypes.hpp>


template <>
HighFive::DataType HighFive::create_datatype<wmtk::AttributeChanges>();
using namespace wmtk;


void OperationLogger::set_readonly()
{
    read_mode = true;
}
bool OperationLogger::is_readonly()
{
    return read_mode;
}
OperationLogger::OperationLogger(HighFive::File& f, const HighFive::DataType& op_type)
    : file(f)
    , operation_dataset(::create_dataset(f, "operations", op_type))
    , attribute_changes_dataset(::create_dataset<AttributeChanges>(f, "attribute_changes"))
{}
OperationLogger::~OperationLogger() = default;

HighFive::DataSet OperationLogger::create_dataset(
    const std::string& name,
    const HighFive::DataType& datatype)
{
    return ::create_dataset(file, name, datatype);
}


OperationRecorder::OperationRecorder(OperationLogger& logger_, const std::string& cmd)
    : logger(logger_)
    , name(cmd)
{
    for (auto& [_, p_attr_recorder] : logger.attribute_recorders) {
        p_attr_recorder->save_size();
    }
}
void OperationRecorder::OperationRecorder::cancel()
{
    is_cancelled = true;
}

OperationRecorder::~OperationRecorder()
{
    if (!cancelled()) {
        spdlog::error(
            "Recorder cannot be destroyed without being flushed! derived class must flush!");
    }
}

void OperationRecorder::flush()
{
    // only lock the mutex when we output to the output stream
    if (!cancelled()) {
        tbb::mutex::scoped_lock lock(logger.output_mutex);


        // commit update attributes
        std::vector<AttributeChanges> changes;

        for (auto&& [attr_name, p_recorder] : logger.attribute_recorders) {
            auto [start, end, size] = p_recorder->record();
            changes.emplace_back(AttributeChanges{attr_name, start, end, size});
        }
        auto [start, end] = append_values_to_1d_dataset(logger.attribute_changes_dataset, changes);

        // commit command itself
        commit(start, end);
    }
    is_cancelled = true;
}
size_t OperationLogger::attribute_changes_count() const
{
    return attribute_changes_dataset.getElementCount();
}

size_t OperationLogger::operation_count() const
{
    return operation_dataset.getElementCount();
}
void OperationLogger::add_attribute_recorder(
    std::string&& name,
    AttributeCollectionRecorderBase& attribute_recorder)
{
    attribute_recorders.emplace(std::move(name), &attribute_recorder);
}

/*
void OperationLogger::record_attributes()
{
    for (auto&& [name, attr_recorder] : attribute_recorders) {
        attr_recorder->record(dataset, name);
    }
}
bool OperationLogger::record_attribute(const std::string& attribute_name)
{
    if (auto it = attribute_recorders.find(attribute_name); it != attribute_recorders.end()) {
        attr_recorder->record(dataset, name);
    } else {
        return false;
    }
    return true;
}
*/

