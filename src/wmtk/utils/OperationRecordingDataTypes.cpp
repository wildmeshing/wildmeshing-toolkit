#include <wmtk/utils/OperationRecordingDataTypes.hpp>

using namespace wmtk;
HIGHFIVE_REGISTER_TYPE(wmtk::AttributeChanges, wmtk::AttributeChanges::datatype);
HIGHFIVE_REGISTER_TYPE(wmtk::TriMeshOperationData, wmtk::TriMeshOperationData::datatype);
HIGHFIVE_REGISTER_TYPE(wmtk::TriMeshTupleData, wmtk::TriMeshTupleData::datatype);

HighFive::CompoundType AttributeChanges::datatype()
{
    return HighFive::CompoundType{
        {"attribute_name", HighFive::create_datatype<char[20]>()},
        {"attribute_size", HighFive::create_datatype<size_t>()},
        {"change_range_begin", HighFive::create_datatype<size_t>()},
        {"change_range_end", HighFive::create_datatype<size_t>()}};
}

HighFive::CompoundType TriMeshOperationData::datatype()
{
    return HighFive::CompoundType{
        {"name", HighFive::create_datatype<char[20]>()},
        {"input_tuple", HighFive::create_datatype<TriMeshTupleData>()},
        {"output_tuple", HighFive::create_datatype<TriMeshTupleData>()},
        {"update_range_begin", HighFive::create_datatype<size_t>()},
        {"update_range_end", HighFive::create_datatype<size_t>()},
        {"vertex_size", HighFive::create_datatype<size_t>()},
        {"triangle_size", HighFive::create_datatype<size_t>()}};
}
HighFive::CompoundType TriMeshTupleData::datatype()
{
    return HighFive::CompoundType{
        {"triangle_id", HighFive::create_datatype<size_t>()},
        {"local_edge_id", HighFive::create_datatype<size_t>()},
        {"vertex_id", HighFive::create_datatype<size_t>()}};
}

TriMeshTupleData::TriMeshTupleData(const TriMesh::Tuple& tuple)
    : triangle_id(tuple.m_fid)
    , local_edge_id(tuple.m_eid)
    , vertex_id(tuple.m_vid)
{}
