#pragma once
#include <vector>
#include <wmtk/attribute/utils/HybridRationalAccessor.hpp>
#include <wmtk/operations/AttributesUpdate.hpp>
namespace wmtk {
class Mesh;
namespace simplex {
class Simplex;
}
} // namespace wmtk

namespace wmtk::operations::utils {


class RoundRationalToDouble : public AttributesUpdate
{
public:
    PrimitiveType primitive_type() const override;

    RoundRationalToDouble(
        const Mesh& m,
        const attribute::utils::HybridRationalAttribute<Eigen::Dynamic>& attr);

    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

protected:
    wmtk::attribute::utils::HybridRationalAccessor<>& hybrid_accessor()
    {
        return m_hybrid_accessor;
    }

private:
    wmtk::attribute::utils::HybridRationalAccessor<> m_hybrid_accessor;
};
} // namespace wmtk::operations::utils
