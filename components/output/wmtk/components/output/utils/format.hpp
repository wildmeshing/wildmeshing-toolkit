#include <fmt/format.h>
#include <wmtk/components/output/OutputOptions.hpp>
namespace wmtk::components::output::options {

template <typename... Args>
auto format(const OutputOptions& input, Args&&... args) -> OutputOptions
{
    OutputOptions opt = input;
    opt.path = fmt::format(input.path, std::forward<Args>(args)...);

    return opt;
}

} // namespace wmtk::components::output::options
