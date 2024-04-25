#pragma once

#include "AttributesUpdate.hpp"

namespace wmtk::operations {
class AdjustSizingField : public AttributesUpdate
{
public:
    AdjustSizingField(Mesh& m, TypedAttributeHandle<Rational>& coordinate);

    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

private:
    TypedAttributeHandle<Rational>& m_coordinate_handle;
}
} // namespace wmtk::operations