#include <wmtk/EdgeMesh.hpp>
#include "Invariant.hpp"

namespace wmtk {
class MultiMeshEdgeTopologyInvariant : public Invariant
{
public:
    MultiMeshEdgeTopologyInvariant(const Mesh& parent, const EdgeMesh& child);

    /**
     * @brief check if both the vertices are in the child mesh but the edge itself is not in the
     * child mesh (return false if so)
     *
     * @param t edge tuple
     * @return true otherwise
     * @return false both vertices are in the child edge mesh but the edge itself is not in.
     */
    bool before(const simplex::Simplex& t) const override;

private:
    // const Mesh& m_child_mesh;
};
} // namespace wmtk
