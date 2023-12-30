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

    long top_cell_dimension() const override { return 0; }
    [[noreturn]] Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const override;
    bool is_ccw(const Tuple& tuple) const override;
    using Mesh::is_boundary;
    bool is_boundary(const Tuple& tuple, PrimitiveType pt) const override;
    bool is_boundary_vertex(const Tuple& tuple) const override;

    void initialize(long count);


    bool is_valid(const Tuple& tuple, ConstAccessor<long>& hash_accessor) const override;

    bool is_connectivity_valid() const override { return true; }

    std::vector<std::vector<TypedAttributeHandle<long>>> connectivity_attributes() const override;

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
