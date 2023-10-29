#pragma once
#include "AMIPS.hpp"
namespace wmtk::function {
class AMIPS2D : public AMIPS
{
public:
    AMIPS2D(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle);

protected:
    DScalar get_value_autodiff(const Tuple& simplex) const override;


    template <typename T>
    T function_eval(const Tuple& tuple) const;

private:
};
} // namespace wmtk::function
