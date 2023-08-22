#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>

class Energy
{
private:
    const Mesh& m;
    const MeshAttributeHandle<double> position_handle;


public:
    Energy(const Mesh& m, const MeshAttributeHandle<double>& position_handle);
    Enegry(const Mesh& m);

public:
    virtual double energy_eval(const Tuple& tuple) const = 0;
}