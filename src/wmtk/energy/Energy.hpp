#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>

class Energy
{
private:
    const Mesh& m_mesh;
    const MeshAttributeHandle<double> m_position_handle;


public:
    Energy(const Mesh& mesh, const MeshAttributeHandle<double>& position_handle);
    Enegry(const Mesh& mesh);

public:
    virtual double energy_eval(const Tuple& tuple) const = 0;
}