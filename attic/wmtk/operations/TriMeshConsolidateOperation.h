
#pragma once
#include <wmtk/TriMeshOperation.h>

namespace wmtk {
/**
 * @brief removing the elements that are removed
 *
 * @param bnd_output when turn on will write the boundary vertices to "bdn_table.dmat"
 */
class TriMeshConsolidateOperation : public TriMeshOperation
{
public:
    bool execute(TriMesh& m, const Tuple& t) override;
    bool before(TriMesh& m, const Tuple& t) override;
    bool after(TriMesh& m) override;
    std::vector<TriMeshTuple> modified_triangles(const TriMesh& m) const override;
    std::string name() const override;
};
} // namespace wmtk
