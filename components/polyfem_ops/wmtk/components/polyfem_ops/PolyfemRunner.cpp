#include "PolyfemRunner.hpp"

#include <wmtk/utils/Logger.hpp>

#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

/// The child's environment: POSIX declares `environ` but no header does on macOS.
extern char** environ;

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
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

/// `str.strip()` with no argument: Python strips exactly these six whitespace characters, and the
/// '\r' matters -- under the Python's pty every line ends "\r\n".
std::string python_strip(const std::string& s)
{
    const auto is_space = [](char c) {
        return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f';
    };
    size_t b = 0;
    size_t e = s.size();
    while (b < e && is_space(s[b])) ++b;
    while (e > b && is_space(s[e - 1])) --e;
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

/// Run `argv` with stdout and stderr merged into one pipe and stdin on /dev/null, mirroring the
/// child `run_streaming` spawns. Returns the raw captured text and the Python's `returncode`
/// convention: the exit status, or minus the signal number when the child was killed.
std::pair<int, std::string> spawn_and_capture(
    const std::vector<std::string>& argv,
    const std::function<void(const std::string&)>& on_chunk)
{
    int fds[2];
    if (::pipe(fds) != 0) {
        log_and_throw_error("Unable to create a pipe for polyfem: {}", std::strerror(errno));
    }

    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addclose(&actions, fds[0]);
    posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_adddup2(&actions, fds[1], STDOUT_FILENO);
    posix_spawn_file_actions_adddup2(&actions, fds[1], STDERR_FILENO);
    posix_spawn_file_actions_addclose(&actions, fds[1]);

    std::vector<char*> c_argv;
    c_argv.reserve(argv.size() + 1);
    for (const auto& arg : argv) {
        c_argv.push_back(const_cast<char*>(arg.c_str()));
    }
    c_argv.push_back(nullptr);

    pid_t pid = -1;
    const int rc = posix_spawn(&pid, argv[0].c_str(), &actions, nullptr, c_argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    ::close(fds[1]);
    if (rc != 0) {
        ::close(fds[0]);
        log_and_throw_error("Unable to start {}: {}", argv[0], std::strerror(rc));
    }

    std::string captured;
    std::array<char, 4096> buffer;
    while (true) {
        const ssize_t n = ::read(fds[0], buffer.data(), buffer.size());
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (n == 0) break;
        const std::string chunk(buffer.data(), static_cast<size_t>(n));
        // Straight to stdout so the human sees the solve as it runs, as the Python does.
        std::fwrite(chunk.data(), 1, chunk.size(), stdout);
        std::fflush(stdout);
        captured += chunk;
        on_chunk(chunk);
    }
    ::close(fds[0]);

    int status = 0;
    while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
    }
    int returncode = 0;
    if (WIFEXITED(status)) {
        returncode = WEXITSTATUS(status);
    } else if (WIFSIGNALED(status)) {
        returncode = -WTERMSIG(status);
    }
    return {returncode, captured};
}

} // namespace

std::string polyfem_bin()
{
    const char* env = std::getenv("POLYFEM_BIN");
    const std::string p = env == nullptr ? "" : env;
    if (p.empty()) {
        log_and_throw_error("POLYFEM_BIN is not set — export POLYFEM_BIN=/path/to/PolyFEM_bin");
    }
    if (!std::filesystem::is_regular_file(p)) {
        log_and_throw_error("POLYFEM_BIN points to a missing file: {}", p);
    }
    return p;
}

namespace {

/// The subprocess backend. Its warm start is the pair of hdf5 files the simulation JSON names:
/// polyfem writes `curr_state.hdf5` and reads `prev_state.hdf5`, and committing is the rename
/// between them -- the three file operations `minimum_separation.step_run_polyfem` performs.
class SubprocessBackend : public PolyfemBackend
{
public:
    explicit SubprocessBackend(std::string binary)
        : m_binary(std::move(binary))
    {}

    SolveResult solve(
        const std::filesystem::path& json_path,
        const std::filesystem::path& out_dir,
        const std::filesystem::path& log_path) override
    {
        if (!log_path.parent_path().empty()) {
            std::filesystem::create_directories(log_path.parent_path());
        }
        std::ofstream log_file(log_path);
        if (!log_file.is_open()) {
            log_and_throw_error("Unable to open {} for writing", log_path.string());
        }

        // Buffer until a complete line before stripping, so an escape split across two reads is
        // not mangled; flush per line so `tail -f` works. Both are `run_streaming._log_chunk`.
        std::string pending;
        const auto on_chunk = [&log_file, &pending](const std::string& chunk) {
            pending += chunk;
            const size_t last_nl = pending.rfind('\n');
            if (last_nl == std::string::npos) {
                return;
            }
            log_file << strip_ansi(pending.substr(0, last_nl + 1));
            log_file.flush();
            pending.erase(0, last_nl + 1);
        };

        const auto [returncode, captured] = spawn_and_capture(
            {m_binary, "-j", json_path.string(), "-o", out_dir.string()},
            on_chunk);
        if (!pending.empty()) {
            log_file << strip_ansi(pending);
        }
        log_file.flush();

        SolveResult result{returncode, split_lines(captured), std::nullopt};
        // The child reports the active distance the only way it can, by printing it; this is the
        // parse the two Python loops do on the very same text.
        result.active_distance = parse_active_distance(result.lines);
        return result;
    }

    void reset_warm_start(
        const std::filesystem::path& curr_state,
        const std::filesystem::path& prev_state) override
    {
        m_curr_state = curr_state;
        m_prev_state = prev_state;
        std::filesystem::remove(m_curr_state);
        std::filesystem::remove(m_prev_state);
    }

    void commit_warm_start() override
    {
        std::filesystem::remove(m_prev_state);
        std::filesystem::rename(m_curr_state, m_prev_state);
    }

    bool has_warm_start() const override { return std::filesystem::exists(m_prev_state); }

private:
    std::string m_binary;
    std::filesystem::path m_curr_state;
    std::filesystem::path m_prev_state;
};

} // namespace

std::unique_ptr<PolyfemBackend> subprocess_backend(const std::string& binary)
{
    return std::make_unique<SubprocessBackend>(binary);
}

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
    const std::vector<std::string>& lines,
    bool allow_out_of_iterations)
{
    std::string stdout_text;
    for (const auto& line : lines) {
        stdout_text += line;
    }

    std::vector<std::string> success_phrases = {
        "Finished: Gradient vector norm too small",
        "Finished: Relative gradient vector too small"};
    if (allow_out_of_iterations) {
        success_phrases.push_back("Finished: Iteration limit reached");
    }
    if (returncode == 0) {
        for (const auto& phrase : success_phrases) {
            if (stdout_text.find(phrase) != std::string::npos) {
                return;
            }
        }
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

std::optional<double> parse_active_distance(const std::vector<std::string>& lines)
{
    static const std::string marker = "active distance:";

    const std::string* found = nullptr;
    for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
        if (it->find(marker) != std::string::npos) {
            found = &*it;
            break;
        }
    }
    if (found == nullptr) {
        return std::nullopt;
    }

    // `line.split(marker)[-1]`: everything after the LAST occurrence in that line.
    const std::string tail = found->substr(found->rfind(marker) + marker.size());
    // `str.split()` with no argument: skip leading whitespace, then take up to the next run.
    const auto is_space = [](char c) {
        return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f';
    };
    size_t b = 0;
    while (b < tail.size() && is_space(tail[b])) ++b;
    size_t e = b;
    while (e < tail.size() && !is_space(tail[e])) ++e;
    if (b == e) {
        // `.split()[0]` on an all-whitespace remainder is an IndexError in the Python; it cannot
        // happen with polyfem's own line, which always has the value right after the marker.
        log_and_throw_error(
            "polyfem's '{}' line carries no value: {}",
            marker,
            python_strip(*found));
    }
    std::string token = tail.substr(b, e - b);
    // `.rstrip(',;')` then `.replace(",", "")`: polyfem writes "<value>, dhat: ...", so the token
    // arrives with a trailing comma.
    while (!token.empty() && (token.back() == ',' || token.back() == ';')) {
        token.pop_back();
    }
    token.erase(std::remove(token.begin(), token.end(), ','), token.end());

    // `float(token)` on the token itself, so the value is the one polyfem printed to the last
    // digit. strtod and CPython's float() are both correctly rounded and both take "inf"/"nan";
    // the C locale is the process default and neither engine changes it, so the decimal point is
    // '.' on both sides.
    const char* start = token.c_str();
    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(start, &end);
    if (end != start + token.size()) {
        log_and_throw_error("could not read an active distance from '{}'", token);
    }
    return value;
}

} // namespace wmtk::components::polyfem_ops
