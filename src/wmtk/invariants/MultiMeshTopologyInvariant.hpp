#include "MeshInvariant.hpp"

namespace wmtk {
class MultiMeshTopologyInvariant : public MeshInvariant
{
public:
    MultiMeshTopologyInvariant(const Mesh& m);
    bool before(const Tuple& t) const override;
};
} // namespace wmtk
