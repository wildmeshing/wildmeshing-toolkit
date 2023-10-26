#pragma once
#include "Function.hpp"
#include "ValenceEnergyPerEdge.hpp"
namespace wmtk {
namespace function {

class TriMeshValenceFunction : public Function
{
public:
    TriMeshValenceFunction(std::unique_ptr<ValenceEnergyPerEdge>&& function);
    double get_value(const Simplex& simplex) const override;

protected:
    const TriMesh& mesh() const;

private:
    std::unique_ptr<ValenceEnergyPerEdge> m_function;
};
} // namespace function
} // namespace wmtk
