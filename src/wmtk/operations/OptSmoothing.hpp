#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/function/Function.hpp>
#include <wmtk/operations/Operation.hpp>
#include "AttributesUpdateBase.hpp"

namespace wmtk::function {
class Function;
}

namespace wmtk::operations {
class OptSmoothing;

template <>
struct OperationSettings<OptSmoothing> : public OperationSettings<AttributesUpdateBase>
{
    OperationSettings<OptSmoothing>(Mesh& m)
        : OperationSettings<AttributesUpdateBase>(m)
    {}

    std::unique_ptr<wmtk::function::Function> energy;
    // coordinate for teh attribute used to evaluate the energy
    MeshAttributeHandle<double> coordinate_handle;

    void create_invariants();
};

class OptSmoothing : public AttributesUpdateBase
{
protected:
    OptSmoothing(Mesh& m, const Simplex& t, const OperationSettings<OptSmoothing>& settings);

public:
    std::string name() const override;
    bool execute() override;

protected:
    MeshAttributeHandle<double> coordinate_handle() const { return m_settings.coordinate_handle; }

    Accessor<double> coordinate_accessor();
    ConstAccessor<double> const_coordinate_accessor() const;
    const OperationSettings<OptSmoothing>& m_settings;
};

} // namespace wmtk::operations
