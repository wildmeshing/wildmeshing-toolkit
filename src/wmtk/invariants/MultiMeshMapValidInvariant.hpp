#pragma once

#include "Invariant.hpp"

namespace wmtk {
class MultiMeshMapValidInvariant : public Invariant
{
public:
    /*@brief The MultiMeshMapValidInvariant checks whether the MultiMeshMap will remain valid after
     * a collapse operation. It detects situations where the edge targeted for the collapse
     * operation is not in child_meshes, but both of ears are in child_meshes
     *
     * @param m The mesh to check
     * @param t The tuple to check
     * @return True if the MultiMeshMap will still be valid after collapse operation
     */

    MultiMeshMapValidInvariant(const Mesh& m);
    bool before(const simplex::Simplex& t) const override;
};
} // namespace wmtk
