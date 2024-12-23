#include <fmt/format.h>
#include <wmtk/components/output/OutputOptions.hpp>
namespace wmtk::components::output::utils {

template <typename... Args>
auto format(const OutputOptions& input, Args&&... args) -> OutputOptions
{
    OutputOptions opt = input;
    using Var = std::decay_t<decltype(input.position_attribute)>;
    opt.path = fmt::format(fmt::runtime(input.path.string()), std::forward<Args>(args)...);

    /*
    opt.position_attribute = std::visit(
        [&](auto&& attr) -> Var {
            using T = std::decay_t<decltype(attr)>;
            if constexpr (std::is_same_v<T, std::string>) {
                return attr;
            } else {
                throw std::runtime_error("Cannot use meshattributehandle as a position attribute");
                return attribute::MeshAttributeHandle{};
            }
        },
        input.position_attribute);
    */

    return opt;
}

} // namespace wmtk::components::output::utils
