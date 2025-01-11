#include "volumes.hpp"
#include <wmtk/Mesh.hpp>
#include "operators/VolumeOperator.hpp"
namespace wmtk::components::mesh_info::simplex {


std::shared_ptr<wmtk::operations::AttributeTransferStrategyBase> volumes(
    const attribute::MeshAttributeHandle& pos,
    const attribute::MeshAttributeHandle& vol,
    bool run)
{
    auto ret = std::visit(
        [&](const auto& p,
            const auto& v) -> std::shared_ptr<wmtk::operations::AttributeTransferStrategyBase> {
            using PT = std::decay_t<decltype(p)>::Type;
            using VT = std::decay_t<decltype(v)>::Type;
            return std::make_shared<operators::VolumeOperator<VT, PT>>(vol, pos);
        },
        vol.handle(),
        pos.handle()
        );
    if (run) {
        ret->run_on_all();
    }
    return ret;
}
template <typename T>
std::shared_ptr<wmtk::operations::AttributeTransferStrategyBase> volumes(
    const attribute::MeshAttributeHandle& mah,
    wmtk::PrimitiveType primitive_type,
    const std::string_view& name,
    bool run)
{
    return volumes<T>(mah, const_cast<Mesh&>(mah.mesh()), primitive_type, name, run);
}

template <typename T>
std::shared_ptr<wmtk::operations::AttributeTransferStrategyBase> volumes(
    const attribute::MeshAttributeHandle& mah,
    Mesh& mesh,
    wmtk::PrimitiveType primitive_type,
    const std::string_view& name,
    bool run)
{
    auto vol_attr = mesh.register_attribute<T>(std::string(name), primitive_type, 1);
    return volumes(mah, vol_attr, run);
}

template std::shared_ptr<wmtk::operations::AttributeTransferStrategyBase> volumes<double>(
    const attribute::MeshAttributeHandle& m,
    Mesh&,
    wmtk::PrimitiveType primitive_type,
    const std::string_view& name,
    bool run);
template std::shared_ptr<wmtk::operations::AttributeTransferStrategyBase> volumes<wmtk::Rational>(
    const attribute::MeshAttributeHandle& m,
    Mesh&,
    wmtk::PrimitiveType primitive_type,
    const std::string_view& name,
    bool run);

template std::shared_ptr<wmtk::operations::AttributeTransferStrategyBase> volumes<double>(
    const attribute::MeshAttributeHandle& m,
    wmtk::PrimitiveType primitive_type,
    const std::string_view& name,
    bool run);
template std::shared_ptr<wmtk::operations::AttributeTransferStrategyBase> volumes<wmtk::Rational>(
    const attribute::MeshAttributeHandle& m,
    wmtk::PrimitiveType primitive_type,
    const std::string_view& name,
    bool run);

} // namespace wmtk::components::mesh_info::simplex
