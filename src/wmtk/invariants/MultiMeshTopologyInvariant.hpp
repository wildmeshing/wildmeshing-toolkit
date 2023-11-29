#include "MeshInvariant.hpp"

namespace wmtk {
class MultiMeshEdgeTopologyInvariant : public MeshInvariant
{
public:
    MultiMeshEdgeTopologyInvariant(const Mesh& parent, const Mesh& child, const PrimitiveType pt);

    /**
     * @brief check if both the vertices are in the child mesh but the edge itself is not in the
     * child mesh (return false if so)
     *
     * @param t edge tuple
     * @return true otherwise
     * @return false both vertices are in the child edge mesh but the edge itself is not in.
     */
    bool before(const Tuple& t) const override;

private:
    const PrimitiveType m_primitive_type;
    const Mesh& m_child_mesh;
};
} // namespace wmtk
