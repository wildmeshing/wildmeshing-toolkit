#include "EdgeOperationData.hpp"
#include <wmtk/TriMesh.hpp>


namespace wmtk::operations::tri_mesh {


std::vector<std::array<Tuple, 2>> EdgeOperationData::ear_edges(const TriMesh& m) const
{
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
}
std::array<Tuple, 2> EdgeOperationData::input_endpoints(const TriMesh& m) const
{
    std::array<Tuple, 2> r;
    r[0] = m_operating_tuple;
    r[1] = m.switch_tuple(m_operating_tuple, PrimitiveType::Vertex);
    return r;
}

std::vector<Tuple> EdgeOperationData::collapse_merged_ear_edges(const TriMesh& m) const
{
    std::vector<Tuple> ret;
    ret.reserve(incident_face_datas().size());

    for (const auto& ifd : incident_face_datas()) {
        ret.emplace_back(tuple_from_id(m, PrimitiveType::Edge, ifd.new_edge_id));
    }
    return ret;
}

std::vector<Tuple> EdgeOperationData::split_new_rib_edges(const TriMesh& m) const
{
    std::vector<Tuple> ret;
    ret.reserve(incident_face_datas().size());

    for (const auto& ifd : incident_face_datas()) {
        ret.emplace_back(tuple_from_id(m, PrimitiveType::Edge, ifd.new_edge_id));
    }
    return ret;
}
std::vector<Tuple> EdgeOperationData::input_faces(const TriMesh& m) const
{
    std::vector<Tuple> ret;
    ret.reserve(incident_face_datas().size());

    for (const auto& ifd : incident_face_datas()) {
        ret.emplace_back(ifd.local_operating_tuple);
    }
    return ret;
}
std::vector<std::array<Tuple, 2>> EdgeOperationData::split_output_faces(const TriMesh& m) const
{
    std::vector<std::array<Tuple, 2>> ret;
    ret.reserve(incident_face_datas().size());

    for (const auto& ifd : incident_face_datas()) {
        std::array<Tuple, 2>& r = ret.emplace_back();
        for (size_t j = 0; j < 2; ++j) {
            r[j] = tuple_from_id(m, PrimitiveType::Triangle, ifd.split_f[j]);
        }
        // std::swap(r[0],r[1]);
    }
    return ret;
}

std::array<Tuple, 2> EdgeOperationData::split_output_edges(const TriMesh& m) const
{
    std::array<Tuple, 2> r;
    // logger().trace("[{}] < {}", fmt::join(split_spine_eids, ","),
    // m.capacity(PrimitiveType::Edge));
    for (size_t j = 0; j < 2; ++j) {
        r[j] = tuple_from_id(m, PrimitiveType::Edge, split_spine_eids[j]);
    }
    return r;
}
} // namespace wmtk::operations::tri_mesh
