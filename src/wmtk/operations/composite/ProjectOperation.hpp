#pragma once

#include <wmtk/operations/AttributesUpdate.hpp>

#include <memory>

namespace SimpleBVH {
class BVH;
}

namespace wmtk::operations::composite {
class ProjectOperation : public AttributesUpdate
{
public:
    ProjectOperation(
        Mesh& m,
        std::shared_ptr<wmtk::operations::Operation> main_op,
        const TypedAttributeHandle<double>& coordinates,
        const TypedAttributeHandle<int64_t>& proj_tag,
        wmtk::PrimitiveType proj_type,
        int64_t proj_value);

    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

private:
    const std::shared_ptr<wmtk::operations::Operation> m_main_op;
    const TypedAttributeHandle<double> m_coordinates;
    const TypedAttributeHandle<int64_t> m_tag;
    const int64_t m_tag_value;
    std::shared_ptr<SimpleBVH::BVH> m_bvh = nullptr;
};
} // namespace wmtk::operations::composite
