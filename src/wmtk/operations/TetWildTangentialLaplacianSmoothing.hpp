#pragma once

#include <wmtk/utils/Rational.hpp>
#include "AttributesUpdate.hpp"


namespace wmtk::operations {

class TetWildTangentialLaplacianSmoothing : public AttributesUpdate
{
public:
    TetWildTangentialLaplacianSmoothing(
        Mesh& m,
        const TypedAttributeHandle<Rational>& coordinate,
        double damping_factor = 1.0);

    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

private:
    const TypedAttributeHandle<Rational>& m_coordinate_handle;
    double m_damping_factor = 1.0;
};

} // namespace wmtk::operations