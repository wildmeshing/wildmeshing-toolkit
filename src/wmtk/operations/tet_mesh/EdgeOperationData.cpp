
#include "EdgeOperationData.hpp"
#include <wmtk/TetMesh.hpp>


namespace wmtk::operations::tet_mesh {


std::vector<std::array<Tuple, 2>> EdgeOperationData::ear_edges(const TetMesh& m) const
{
    std::vector<std::array<Tuple, 2>> ret;
    ret.reserve(m_incident_face_datas.size());

    for (const auto& ifd : m_incident_face_datas) {
        std::array<Tuple, 2>& r = ret.emplace_back();

        for (size_t j = 0; j < 2; ++j) {
            int64_t eid = ifd.ear_eids[j];
            r[j] = tuple_from_id(m, PrimitiveType::Edge, eid);
        }
    }
    return ret;
}
std::vector<std::array<Tuple, 2>> EdgeOperationData::ear_faces(const TetMesh& m) const
{
    std::vector<std::array<Tuple, 2>> ret;
    ret.reserve(m_incident_tet_datas.size());

    for (const auto& itd : m_incident_tet_datas) {
        std::array<Tuple, 2>& r = ret.emplace_back();

        for (size_t j = 0; j < 2; ++j) {
            int64_t fid = itd.ears[j].fid;
            r[j] = tuple_from_id(m, PrimitiveType::Face, fid);
        }
    }
    return ret;
}
std::array<Tuple, 2> EdgeOperationData::input_endpoints(const TetMesh& m) const
{
    std::array<Tuple, 2> r;
    r[0] = m_operating_tuple;
    r[1] = m.switch_tuple(m_operating_tuple, PrimitiveType::Vertex);
    return r;
}

std::vector<Tuple> EdgeOperationData::collapse_merged_ear_edges(const TetMesh& m) const
{
    std::vector<Tuple> ret;
    ret.reserve(incident_face_datas().size());

    for (const auto& ifd : incident_face_datas()) {
        ret.emplace_back(tuple_from_id(m, PrimitiveType::Edge, ifd.new_edge_id));
    }
    return ret;
}

std::vector<Tuple> EdgeOperationData::collapse_merged_ear_faces(const TetMesh& m) const
{
    std::vector<Tuple> ret;
    ret.reserve(incident_tet_datas().size());

    for (const auto& ifd : incident_tet_datas()) {
        ret.emplace_back(tuple_from_id(m, PrimitiveType::Face, ifd.new_face_id));
    }
    return ret;
}


} // namespace wmtk::operations::tet_mesh
