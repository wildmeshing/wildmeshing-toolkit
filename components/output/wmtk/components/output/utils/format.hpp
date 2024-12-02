#include <fmt/format.h>
#include <wmtk/components/output/OutputOptions.hpp>
namespace wmtk::components::output::utils {

template <typename... Args>
auto format(const OutputOptions& input, Args&&... args) -> OutputOptions
{
    OutputOptions opt = input;
    opt.path = fmt::format(fmt::runtime(input.path.string()), std::forward<Args>(args)...);

    return opt;
}

} // namespace wmtk::components::output::options
