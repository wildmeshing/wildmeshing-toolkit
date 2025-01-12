#pragma once
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <spdlog/spdlog.h>
#include <fmt/ranges.h>
#include <wmtk/utils/triangle_areas.hpp>
namespace wmtk {
class Mesh;
namespace simplex {
class Simplex;
}
} // namespace wmtk
namespace wmtk::components::mesh_info::simplex::operators {

template <typename MyType, typename ParentType, int ParentDim = Eigen::Dynamic>
class DihedralAngleOperator
    : public wmtk::operations::SingleAttributeTransferStrategyBase<MyType, ParentType>
{
public:
    using BaseType = wmtk::operations::SingleAttributeTransferStrategyBase<MyType, ParentType>;

    using BaseType::handle;
    using BaseType::mesh;
    using BaseType::parent_handle;
    using BaseType::primitive_type;

    void run(const wmtk::simplex::Simplex& s) const final
    {
        assert(handle().mesh().is_valid(s.tuple()));
        if (s.primitive_type() != primitive_type()) {
            // TODO: is this an error out or silent fail
            return;
        }
        // position mesh
        const TriMesh& pmesh = static_cast<const TriMesh&>(parent_handle().mesh());

        auto ang_acc =
            parent_handle().mesh().create_const_accessor(parent_handle().template as<ParentType>());
        auto acc = const_cast<Mesh&>(mesh()).create_accessor(handle().template as<MyType>());

        auto& angle = acc.scalar_attribute(s.tuple());
        if (pmesh.is_boundary(s)) {
            angle = 0;
        } else if constexpr(std::is_same_v<ParentType, double> && std::is_same_v<MyType, double>) { // only supports double for now due to needing a sqrt
            auto pos_acc = parent_handle().mesh().template create_const_accessor<ParentType, 3>(
                parent_handle()
                );


            // edge a -> b; input triangle has edge c, opp has d
            //      /c
            //     / |
            //    /t |
            //   /   |
            //   a---b
            //   \   |
            //    \  |
            //     \ |
            //      \d
            auto a = pos_acc.const_vector_attribute(s.tuple());

            auto b = pos_acc.const_vector_attribute(
                pmesh.switch_tuples(s.tuple(), {PrimitiveType::Vertex}));

            auto c = pos_acc.const_vector_attribute(
                pmesh.switch_tuples(s.tuple(), {PrimitiveType::Edge, PrimitiveType::Vertex}));
            auto d = pos_acc.const_vector_attribute(
                pmesh.switch_tuples(s.tuple(), {PrimitiveType::Triangle, PrimitiveType::Edge, PrimitiveType::Vertex}));

            auto ban = (b - a).eval();
            ParentType normSquared = ban.squaredNorm();
            auto ca = (c - a).eval();
            auto da = (d - a).eval();

            ca.noalias() = ca - (ca.dot(ban) / normSquared)* ban;
            da.noalias() = da - (da.dot(ban) / normSquared)* ban;

            angle = MyType(std::atan2(ca.cross(da).norm(), ca.dot(da)));
        } else {
            // bad type
            assert(false);
        }
    }

    DihedralAngleOperator(
        const attribute::MeshAttributeHandle& my_handle,
        const attribute::MeshAttributeHandle& my_parent_handle)
        : BaseType(my_handle, my_parent_handle)
    {
        assert(my_parent_handle.mesh().top_simplex_type() == PrimitiveType::Triangle);
        assert(my_parent_handle.primitive_type() == PrimitiveType::Vertex);
        assert(my_parent_handle.dimension() == 3);
        assert(my_handle.primitive_type() == PrimitiveType::Edge);
    }

private:
    attribute::MeshAttributeHandle m_parent_handle;
};
} // namespace wmtk::components::mesh_info::simplex::operators
