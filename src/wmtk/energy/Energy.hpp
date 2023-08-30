#pragma once
#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>
namespace wmtk {
namespace energy {
class Energy
{
protected:
    const Mesh& m_mesh;
    const MeshAttributeHandle<double> m_position_handle;


public:
    Energy(const Mesh& mesh);
    virtual ~Energy();

public:
    virtual double energy_eval(const Tuple& tuple) const = 0;
};
} // namespace energy
} // namespace wmtk