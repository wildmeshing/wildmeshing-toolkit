#include "PolyfemRunner.hpp"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <regex>
#include <system_error>

namespace wmtk::components::polyfem_ops {

namespace {

/// `polyfem_utils._ANSI_RE`, character class for character class.
const std::regex& ansi_re()
{
    static const std::regex re(std::string("\033") + R"((?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~]))");
    return re;
}

/// The six characters Python calls whitespace, which is what `str.strip()` and `str.split()` with
/// no argument both go by. The '\r' matters -- under the Python's pty every line ends "\r\n".
bool is_python_space(const char c)
{
    return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f';
}

/// `str.strip()` with no argument.
std::string python_strip(const std::string& s)
{
    size_t b = 0;
    size_t e = s.size();
    while (b < e && is_python_space(s[b])) ++b;
    while (e > b && is_python_space(s[e - 1])) --e;
    return s.substr(b, e - b);
}

/// `repr()` of a Python string, which is what the failure message interpolates with `{!r}`: single
/// quotes, backslash escapes for the control characters, and the word None for a missing line.
std::string python_repr_str(const std::optional<std::string>& s)
{
    if (!s.has_value()) {
        return "None";
    }
    std::string out = "'";
    for (const char c : *s) {
        switch (c) {
        case '\\': out += "\\\\"; break;
        case '\'': out += "\\'"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default: out += c; break;
        }
    }
    return out + "'";
}

} // namespace

std::string strip_ansi(const std::string& text)
{
    return std::regex_replace(text, ansi_re(), "");
}

std::vector<std::string> split_lines(const std::string& text)
{
    std::vector<std::string> lines;
    size_t start = 0;
    while (start <= text.size()) {
        const size_t nl = text.find('\n', start);
        if (nl == std::string::npos) {
            lines.push_back(text.substr(start) + "\n");
            break;
        }
        lines.push_back(text.substr(start, nl - start) + "\n");
        start = nl + 1;
    }
    if (!lines.empty() && lines.back() == "\n") {
        lines.pop_back();
    }
    return lines;
}

void check_polyfem_success(
    int returncode,
    const std::vector<polysolve::nonlinear::Status>& statuses,
    const std::vector<std::string>& lines,
    bool allow_out_of_iterations)
{
    using polysolve::nonlinear::Status;
    const auto accepted = [allow_out_of_iterations](const Status s) {
        return s == Status::GradNormTolerance || s == Status::RelGradNormTolerance ||
               (allow_out_of_iterations && s == Status::IterationLimit);
    };
    if (returncode == 0 && std::any_of(statuses.begin(), statuses.end(), accepted)) {
        return;
    }

    std::optional<std::string> finished_line;
    for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
        if (it->find("Finished:") != std::string::npos) {
            finished_line = python_strip(*it);
            break;
        }
    }
    // The Python prints the banner to stdout; here it goes through the logger, which is the one
    // difference, and it is the same difference every `print` in this port has.
    const std::string banner(72, '=');
    logger().error("\n{}\nPOLYFEM SOLVE FAILED (return code {})", banner, returncode);
    if (finished_line.has_value() && !finished_line->empty()) {
        logger().error("  last 'Finished:' line: {}", *finished_line);
    }
    logger().error("{}\n", banner);
    log_and_throw_error(
        "PolyFEM failed (return code {}); last Finished line: {}",
        returncode,
        python_repr_str(finished_line));
}

} // namespace wmtk::components::polyfem_ops
