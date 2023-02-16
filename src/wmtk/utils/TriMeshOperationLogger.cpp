#include <wmtk/utils/TriMeshOperationLogger.h>


template <>
HighFive::DataType HighFive::create_datatype<wmtk::TriMeshOperationData>();
template <>
HighFive::DataType HighFive::create_datatype<wmtk::TriMeshTupleData>();

WMTK_HDF5_REGISTER_ATTRIBUTE_TYPE(wmtk::TriMesh::TriangleConnectivity)


template <>
HighFive::DataType HighFive::create_datatype<wmtk::TriMesh::TriangleConnectivity>()
{
    return HighFive::CompoundType{
        {"v0", HighFive::create_datatype<size_t>()},
        {"v1", HighFive::create_datatype<size_t>()},
        {"v2", HighFive::create_datatype<size_t>()},
        {"is_removed", HighFive::create_datatype<bool>()},
        {"hash", HighFive::create_datatype<size_t>()}};
}

using namespace wmtk;


TriMeshOperationRecorder::TriMeshOperationRecorder(
    TriMeshOperationLogger& logger_,
    const std::string& cmd,
    const TriMesh::Tuple& tuple_)
    : OperationRecorder(logger_, cmd)
    , input_tuple(tuple_)
{}

void TriMeshOperationRecorder::set_output_tuple(const TriMesh::Tuple& tup)
{
    output_tuple = tup;
}
TriMeshOperationRecorder::~TriMeshOperationRecorder()
{
    flush();
}
size_t TriMeshOperationRecorder::commit(size_t start, size_t end)
{
    TriMeshOperationData op;
    strncpy(
        op.name,
        name.c_str(),
        sizeof(op.name) / sizeof(char)); // yes sizeof(char)==1, maybe chartype changes someday?
    TriMesh& m = logger().mesh;
    op.input_tuple = TriMeshTupleData(input_tuple);
    op.update_range_begin = start;
    op.update_range_end = end;
    op.vertex_size = m.vert_capacity();
    op.triangle_size = m.tri_capacity();


    auto size = append_value_to_1d_dataset(logger().operation_dataset(), op);
    return size;
}
TriMeshOperationLogger& TriMeshOperationRecorder::logger()
{
    return dynamic_cast<TriMeshOperationLogger&>(this->OperationRecorder::logger);
}

HighFive::DataSet& TriMeshOperationLogger::operation_dataset()
{
    return this->OperationLogger::operation_dataset;
}

TriMeshOperationLogger::TriMeshOperationLogger(TriMesh& m, HighFive::File& file)
    : OperationLogger(file, HighFive::create_datatype<TriMeshOperationData>())
    , mesh(m)
    , tri_recorder(file, "tri_connectivity", m.m_tri_connectivity)
{
    add_attribute_recorder("tri_connectivity", tri_recorder);
    m.p_operation_logger = this;
}

// auto TriMeshOperationLogger::start(
//
//     const TriMesh& m,
//     const std::string_view& cmd,
//     const std::array<size_t, 3>& tuple) -> OperationRecorder
//{
//     return OperationRecorder(*this, OperationRecorder::OperationType::TriMesh, cmd, tuple);
// },

auto TriMeshOperationLogger::start_ptr(
    const TriMesh& m,
    const std::string& cmd,
    const TriMesh::Tuple& tuple) const -> TriMeshOperationRecorder::Ptr
{
    spdlog::info("was const, returning nullptr");
    return nullptr;
}
auto TriMeshOperationLogger::start_ptr(
    const TriMesh& m,
    const std::string& cmd,
    const TriMesh::Tuple& tuple) -> TriMeshOperationRecorder::Ptr
{
    if (is_readonly()) {
        return nullptr;
    } else {
        return std::make_shared<TriMeshOperationRecorder>(*this, cmd, tuple);
    }
}
