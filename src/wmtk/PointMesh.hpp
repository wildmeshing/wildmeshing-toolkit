#pragma once

#include "Mesh.hpp"
#include "Tuple.hpp"

#include <Eigen/Core>

namespace wmtk {
// Simple mesh without topology. Mainly useful for testing attributes without having to construct
// topologies
class PointMesh : public Mesh
{
private:
    Tuple vertex_tuple_from_id(long id) const;

public:
    PointMesh();
    PointMesh(long size);

    PrimitiveType top_simplex_type() const override { return PrimitiveType::Vertex; }
    Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const override;
    bool is_ccw(const Tuple& tuple) const override;
    bool is_boundary(const Tuple& tuple) const override;
    bool is_boundary_vertex(const Tuple& tuple) const override;
    // TODO: should just write is_boundary(PrimitiveType)
    bool is_boundary_edge(const Tuple& tuple) const override { return true; }

    void initialize(long count);


    bool is_valid(const Tuple& tuple, ConstAccessor<long>& hash_accessor) const override;

    Tuple split_edge(const Tuple&, Accessor<long>&) override { return {}; }
    Tuple collapse_edge(const Tuple&, Accessor<long>&) override { return {}; }
    bool is_connectivity_valid() const override { return true; }

protected:
    long id(const Tuple& tuple, PrimitiveType type) const override;

    /**
     * @brief internal function that returns the tuple of requested type, and has the global index
     * cid
     *
     * @param gid
     * @return Tuple
     */
    Tuple tuple_from_id(const PrimitiveType type, const long gid) const override;
};

} // namespace wmtk
