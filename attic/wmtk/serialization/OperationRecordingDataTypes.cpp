//#include <wmtk/utils/OperationRecordingDataTypes.hpp>
//
//
//
// HIGHFIVE_REGISTER_TYPE(wmtk::TriMeshOperationData, wmtk::TriMeshOperationData::datatype);
// HIGHFIVE_REGISTER_TYPE(wmtk::TriMeshTupleData, wmtk::TriMeshTupleData::datatype);
// HighFive::CompoundType TriMeshOperationData::datatype()
//{
//    return HighFive::CompoundType{
//        {"name", HighFive::create_datatype<char[20]>()},
//        {"input_tuple", HighFive::create_datatype<TriMeshTupleData>()},
//        {"output_tuple", HighFive::create_datatype<TriMeshTupleData>()},
//        {"update_range", HighFive::create_datatype<AttributeCollectionRange>()},
//        {"vertex_size", HighFive::create_datatype<size_t>()},
//        {"triangle_size", HighFive::create_datatype<size_t>()}};
//}
// HighFive::CompoundType TriMeshTupleData::datatype()
//{
//    return HighFive::CompoundType{
//        {"triangle_id", HighFive::create_datatype<size_t>()},
//        {"local_edge_id", HighFive::create_datatype<size_t>()},
//        {"vertex_id", HighFive::create_datatype<size_t>()}};
//}
//
// TriMeshTupleData::TriMeshTupleData(const TriMesh::Tuple& tuple)
//    : triangle_id(tuple.m_fid)
//    , local_edge_id(tuple.m_eid)
//    , vertex_id(tuple.m_vid)
//{}
