#include <wmtk/TriMesh.hpp>
#include "Energy.hpp"
namespace wmtk {
namespace energy {

class ValenceEnergy : public Energy
{
public:
    ValenceEnergy(const TriMesh& mesh);
    double energy_eval(const Tuple& tuple) const override;

protected:
    TriMesh& mesh() const;
};
} // namespace energy
} // namespace wmtk