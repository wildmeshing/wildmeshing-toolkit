
#include "EdgeOperationData.hpp"
#include <wmtk/TetMesh.hpp>


namespace wmtk::operations::tet_mesh {


std::vector<std::array<Tuple, 2>> EdgeOperationData::ear_edges(const TetMesh& m) const
{
    return {};
    /*
    std::vector<std::array<Tuple, 2>> ret;
    ret.reserve(incident_face_datas().size());

    for (const auto& ifd : incident_face_datas()) {
        std::array<Tuple, 2>& r = ret.emplace_back();

        for (size_t j = 0; j < 2; ++j) {
            int64_t eid = ifd.ears[j].eid;
            r[j] = tuple_from_id(m, PrimitiveType::Edge, eid);
        }
    }
    return ret;
    */
}
std::vector<std::array<Tuple, 2>> EdgeOperationData::ear_faces(const TetMesh& m) const
{
    // std::vector<std::array<Tuple, 2>> ret;
    // ret.reserve(m_incident_tet_datas.size());

    // for (const auto& itd : m_incident_tet_datas) {
    //     std::array<Tuple, 2>& r = ret.emplace_back();

    //     for (size_t j = 0; j < 2; ++j) {
    //         int64_t eid = itd.ears[j].eid;
    //         r[j] = tuple_from_id(m, PrimitiveType::Face, eid);
    //     }
    // }
    // return ret;
    return {};
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
    // std::vector<std::array<Tuple, 2>> ret;
    // ret.reserve(m_incident_tet_datas.size());

    // for (const auto& itd : m_incident_tet_datas) {
    //     std::array<Tuple, 2>& r = ret.emplace_back();

    //     for (size_t j = 0; j < 2; ++j) {
    //         int64_t eid = itd.ears[j].eid;
    //         r[j] = tuple_from_id(m, PrimitiveType::Edge, itd.collapse_new_face_id);
    //     }
    // }
    // return ret;
    return {};
}


} // namespace wmtk::operations::tet_mesh
