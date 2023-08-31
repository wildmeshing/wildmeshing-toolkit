#include <wmtk/TriMesh.hpp>
#include "Energy.hpp"
namespace wmtk {
namespace energy {

class TriMeshValenceEnergy : public Energy
{
public:
    TriMeshValenceEnergy(const TriMesh& mesh);
    double energy_eval(const Tuple& tuple) const override;

protected:
    TriMesh& mesh() const;
};
} // namespace energy
} // namespace wmtk