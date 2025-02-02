#pragma once

#include <map>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/TypedAttributeHandle.hpp>

namespace wmtk::submesh {

class Embedding;

/**
 * Embedding m;
 * SubMesh s = m.add_sub_mesh();
 * s.add_simplex(IdSimplex);
 * s.add_from_tag(tag_handle, tag_value);
 */
class SubMesh
{
public:
    SubMesh(Embedding& embedding, int64_t submesh_id);
    SubMesh() = delete;
    SubMesh(const SubMesh&) = delete;
    SubMesh& operator=(const SubMesh&) = delete;
    SubMesh(SubMesh&&) = delete;
    SubMesh& operator=(SubMesh&&) = delete;

    void add_simplex(const Tuple& tuple, PrimitiveType pt);

    // throws if `type` is larger than the substructure max dimension of tuple
    Tuple switch_tuple(const Tuple& tuple, PrimitiveType pt) const;

    // call open_star_single_dimension if `pt` is larger or equal than max dim, use `switch_tuple`
    // otherwise
    std::vector<Tuple> switch_tuple_vector(const Tuple& tuple, PrimitiveType pt) const;

    // 1. get max dim in open star
    // 2. check if any incident max dim facet has less than two neighbors
    bool is_boundary(PrimitiveType pt, const Tuple& tuple) const;

    // This is going to be some ugly recursive stuff I guess...
    bool is_manifold(PrimitiveType pt, const Tuple& tuple) const;

    // Check if sub mesh contains the simplex
    bool contains(PrimitiveType pt, const Tuple& tuple) const;

    // must be part of the construction
    void initialize(
        const std::map<PrimitiveType, attribute::TypedAttributeHandle<int64_t>>& tag_attributes,
        const int64_t tag_value);

    // forward to Mesh
    int64_t id(const Tuple& tuple, PrimitiveType pt) const;

private:
    Embedding& m_embedding;
    const int64_t m_submesh_id;
};

} // namespace wmtk::submesh
