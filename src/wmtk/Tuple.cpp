#include <Tuple.h>

size_t Tuple::id(const Mesh& m, const int dimension) const
{
    switch (dimension) {
    case 0: return m.m_face_connectivity[m_local_fid * 3 + m_local_vid];
    case 1: return m.get_edge(m_local_eid).id();
    case 2: return m.get_face(m_local_fid).id();
    case 3: return m.get_cell(m_global_cid).id();
    default: throw std::runtime_error("Invalid dimension");
    }
}
void Tuple::sw(const Mesh& m, const int dimension) const {};
bool Tuple::is_valid(const Mesh& m) const;
bool Tuple::is_ccw(const Mesh& m) const;