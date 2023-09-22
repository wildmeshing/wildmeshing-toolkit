#include "AdaptiveTessellation.h"


namespace adaptive_tessellation {
VertexAttributes& AdaptiveTessellation::get_vertex_attrs(const Tuple& t)
{
    return vertex_attrs[t.vid(*this)];
}
const VertexAttributes& AdaptiveTessellation::get_vertex_attrs(const Tuple& t) const
{
    return vertex_attrs[t.vid(*this)];
}
FaceAttributes& AdaptiveTessellation::get_face_attrs(const Tuple& t)
{
    return face_attrs[t.fid(*this)];
}
const FaceAttributes& AdaptiveTessellation::get_face_attrs(const Tuple& t) const
{
    return face_attrs[t.fid(*this)];
}
EdgeAttributes& AdaptiveTessellation::get_edge_attrs(const Tuple& t)
{
    return edge_attrs[t.eid(*this)];
}
const EdgeAttributes& AdaptiveTessellation::get_edge_attrs(const Tuple& t) const
{
    return edge_attrs[t.eid(*this)];
}
} // namespace adaptive_tessellation
