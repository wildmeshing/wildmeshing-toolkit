#pragma once

#include <oneapi/tbb/concurrent_vector.h>
#include <oneapi/tbb/mutex.h>
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wcast-qual"
#elif (defined(__GNUC__) || defined(__GNUG__)) && !(defined(__clang__) || defined(__INTEL_COMPILER))
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wswitch-enum"
#endif
#include <highfive/H5DataSet.hpp>
#if defined(__clang__)
#pragma clang diagnostic pop
#elif (defined(__GNUC__) || defined(__GNUG__)) && !(defined(__clang__) || defined(__INTEL_COMPILER))
#pragma GCC diagnostic pop
#endif
#include <memory>
#include <nlohmann/json.hpp>
#include <string_view>
#include <wmtk/utils/Logger.hpp>

namespace HighFive {
class File;
}

namespace wmtk {
class TriMesh;
class OperationSerialization;
class OperationReplayer;
class AttributeCollectionRecorderBase;
;


//class OperationRecorder
//{
//public:
//    struct OperationData;
//    using Ptr = std::shared_ptr<OperationRecorder>;
//    OperationRecorder(OperationSerialization& logger_, const std::string& cmd);
//    virtual ~OperationRecorder();
//
//
//    // If the operation
//    void cancel();
//    bool cancelled() const { return this->is_cancelled; }
//    virtual size_t commit(size_t start, size_t end) = 0;
//    void flush();
//
//
//protected:
//    OperationSerialization& logger;
//    std::string name;
//    // each derived class is in charge of its own tuple data
//    std::vector<std::pair<std::string, std::array<size_t, 2>>> attribute_updates;
//    bool is_cancelled = false;
//};

// expects an operation serialization format 
class OperationSerialization
{
public:
    friend class OperationRecorder;
    friend class OperationReplayer;
    friend class AttributeCollectionRecorderBase;
    OperationSerialization(HighFive::File& file, const HighFive::DataType& operation_datatype);
    virtual ~OperationSerialization();


    // the total number of operations that were logged
    size_t operation_count() const;
    // the total number of attribute changes that were logged. multiple can happen per operation
    size_t attribute_changes_count() const;


protected:
    HighFive::DataSet create_dataset(const std::string& name, const HighFive::DataType& datatype);

private:
    oneapi::tbb::mutex output_mutex;
    HighFive::File& file;

protected:
    HighFive::DataSet m_operation_dataset;
    HighFive::DataSet attribute_changes_dataset;
    // std::ostream& output_stream;


    // returns true if attribute was successfully recorded
    std::array<size_t, 2> record_attribute(const std::string& attribute_name);
};


} // namespace wmtk
