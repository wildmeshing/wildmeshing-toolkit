
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
            long eid = ifd.ears[j].eid;
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
    //         long eid = itd.ears[j].eid;
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
    //         long eid = itd.ears[j].eid;
    //         r[j] = tuple_from_id(m, PrimitiveType::Edge, itd.collapse_new_face_id);
    //     }
    // }
    // return ret;
    return {};
}

// std::vector<Tuple> EdgeOperationData::deleted_simplex_tuples(const TetMesh& m, PrimitiveType pt)
//     const
// {
//     std::vector<Tuple> ret;
//     switch (pt) {
//     case PrimitiveType::Vertex: {
//         for (long i = 0; i < simplex_ids_to_delete[0].size(); ++i) {
//             ret.push_back(tuple_from_id(m, PrimitiveType::Vertex, simplex_ids_to_delete[0][i]));
//         }
//         break;
//     }
//     case PrimitiveType::Edge: {
//         for (long i = 0; i < simplex_ids_to_delete[1].size(); ++i) {
//             ret.push_back(tuple_from_id(m, PrimitiveType::Edge, simplex_ids_to_delete[1][i]));
//         }
//         break;
//     }
//     case PrimitiveType::Face: {
//         for (long i = 0; i < simplex_ids_to_delete[2].size(); ++i) {
//             ret.push_back(tuple_from_id(m, PrimitiveType::Face, simplex_ids_to_delete[2][i]));
//         }
//         break;
//     }
//     case PrimitiveType::Tetrahedron: {
//         for (long i = 0; i < simplex_ids_to_delete[3].size(); ++i) {
//             ret.push_back(
//                 tuple_from_id(m, PrimitiveType::Tetrahedron, simplex_ids_to_delete[3][i]));
//         }
//         break;
//     }
//     default: break;
//     }

//     return ret;
// }

std::vector<Tuple> EdgeOperationData::deleted_simplex_tuples(const TetMesh& m, PrimitiveType pt)
    const
{
    std::vector<Tuple> ret;
    switch (pt) {
    case PrimitiveType::Vertex: {
        return simplex_tuples_to_delete[0];
        break;
    }
    case PrimitiveType::Edge: {
        return simplex_tuples_to_delete[1];
        break;
    }
    case PrimitiveType::Face: {
        return simplex_tuples_to_delete[2];
        break;
    }
    case PrimitiveType::Tetrahedron: {
        return simplex_tuples_to_delete[3];
        break;
    }
    default: break;
    }

    return ret;
}

std::vector<Tuple> EdgeOperationData::new_simplex_tuples(const TetMesh& m, PrimitiveType pt) const
{
    std::vector<Tuple> ret;
    switch (pt) {
    case PrimitiveType::Vertex: {
        for (long i = 0; i < m_new_vertex_ids.size(); ++i) {
            ret.push_back(tuple_from_id(m, PrimitiveType::Vertex, m_new_vertex_ids[i]));
        }
        break;
    }
    case PrimitiveType::Edge: {
        for (long i = 0; i < m_new_edge_ids.size(); ++i) {
            ret.push_back(tuple_from_id(m, PrimitiveType::Edge, m_new_edge_ids[i]));
        }
        break;
    }
    case PrimitiveType::Face: {
        for (long i = 0; i < m_new_face_ids.size(); ++i) {
            ret.push_back(tuple_from_id(m, PrimitiveType::Face, m_new_face_ids[i]));
        }
        break;
    }
    case PrimitiveType::Tetrahedron: {
        for (long i = 0; i < m_new_tet_ids.size(); ++i) {
            ret.push_back(tuple_from_id(m, PrimitiveType::Tetrahedron, m_new_tet_ids[i]));
        }
        break;
    }
    default: break;
    }

    return ret;
}


} // namespace wmtk::operations::tet_mesh
