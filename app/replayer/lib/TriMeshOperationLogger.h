#pragma once
#include <wmtk/TriMesh.h>
#include <wmtk/utils/OperationRecordingDataTypes.hpp>


template <>
HighFive::DataType HighFive::create_datatype<
    wmtk::AttributeCollectionRecorder<wmtk::TriMesh::TriangleConnectivity>::UpdateData>();
template <>
HighFive::DataType HighFive::create_datatype<wmtk::TriMesh::TriangleConnectivity>();
template <>
HighFive::DataType
HighFive::create_datatype<wmtk::AttributeUpdateData<wmtk::TriMesh::TriangleConnectivity>>();


#include <wmtk/utils/AttributeRecorder.h>
#include <wmtk/utils/OperationLogger.h>

namespace wmtk {


class TriMeshOperationRecorder : public OperationRecorder
{
public:
    using Ptr = std::shared_ptr<TriMeshOperationRecorder>;
    TriMeshOperationRecorder(
        TriMeshOperationLogger& logger,
        const std::string& cmd,
        const TriMesh::Tuple& tuple);
    ~TriMeshOperationRecorder();

    size_t commit(size_t start, size_t end) override;
    void set_output_tuple(const TriMesh::Tuple& tup);
    ;

protected:
    TriMeshOperationLogger& logger();

private:
    TriMesh::Tuple input_tuple;
    TriMesh::Tuple output_tuple;
};


class TriMeshOperationLogger : public OperationLogger
{
public:
    TriMeshOperationLogger(TriMesh& m, HighFive::File& file);
    TriMeshOperationRecorder::Ptr
    start_ptr(const TriMesh& m, const std::string& cmd, const TriMesh::Tuple& tuple);
    TriMeshOperationRecorder::Ptr
    start_ptr(const TriMesh& m, const std::string& cmd, const TriMesh::Tuple& tuple) const;
    // OperationRecorder start(
    //     const TriMesh& m,
    //     const std::string_view& cmd,
    //     const std::array<size_t, 3>& tuple) override;

    friend class TriMeshOperationRecorder;

protected:
    HighFive::DataSet& operation_dataset();

private:
    TriMesh& mesh;
    AttributeCollectionRecorder<TriMesh::TriangleConnectivity> tri_recorder;
};

} // namespace wmtk
