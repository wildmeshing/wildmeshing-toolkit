#include "DeformedMesh.hpp"

#include "PythonFormat.hpp"

#include <wmtk/utils/Logger.hpp>

#include <mshio/mshio.h>

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <locale>
#include <sstream>

namespace wmtk::components::polyfem_ops {

namespace {

/// Print every double the stream is handed as the shortest decimal that reads back as the same
/// double, which is what `python_repr` produces.
///
/// mshio's ASCII writer formats each value with `out << value`, so the stream's number facet is
/// the only place a caller can decide the format. Imbuing this one is what keeps the deformed
/// mesh lossless; the alternative, a fixed 16 significant digits, drops the last bits of about
/// 40% of coordinates (measured: 122 of 300 pseudo-random values do not survive that round trip).
class ShortestRoundTripDoubles : public std::num_put<char>
{
protected:
    iter_type do_put(iter_type out, std::ios_base&, char_type, double value) const override
    {
        const std::string text = python_repr(value);
        return std::copy(text.begin(), text.end(), out);
    }
};

} // namespace

MshNodes read_msh_nodes(const std::string& msh_path)
{
    if (!std::filesystem::exists(msh_path)) {
        log_and_throw_error("File {} does not exist.", msh_path);
    }
    const mshio::MshSpec spec = mshio::load_msh(msh_path);

    // gmsh.model.mesh.getNodes() hands back the node blocks in file order; mshio keeps that order.
    std::vector<int64_t> tags;
    std::vector<std::array<double, 3>> raw;
    for (const auto& block : spec.nodes.entity_blocks) {
        for (size_t i = 0; i < block.num_nodes_in_block; ++i) {
            tags.push_back(static_cast<int64_t>(block.tags[i]));
            raw.push_back({block.data[3 * i], block.data[3 * i + 1], block.data[3 * i + 2]});
        }
    }
    const int64_t n = static_cast<int64_t>(tags.size());
    if (n == 0) {
        log_and_throw_error("No nodes found in {}", msh_path);
    }

    MshNodes out;
    out.tags = tags;
    const int64_t max_tag = *std::max_element(tags.begin(), tags.end());
    if (max_tag != n) {
        std::vector<int64_t> sorted_tags = tags;
        std::sort(sorted_tags.begin(), sorted_tags.end());
        for (size_t i = 0; i < sorted_tags.size(); ++i) {
            out.tag_to_idx[sorted_tags[i]] = static_cast<int64_t>(i);
        }
    } else {
        for (const int64_t t : tags) {
            out.tag_to_idx[t] = t - 1;
        }
    }

    out.coords.assign(static_cast<size_t>(n), {0.0, 0.0, 0.0});
    for (size_t i = 0; i < tags.size(); ++i) {
        out.coords[static_cast<size_t>(out.tag_to_idx.at(tags[i]))] = raw[i];
    }
    return out;
}

std::vector<std::vector<double>> load_solution_txt(const std::filesystem::path& sol_path)
{
    std::ifstream in(sol_path);
    if (!in.is_open()) {
        log_and_throw_error("Unable to open {} for reading", sol_path.string());
    }

    std::vector<std::vector<double>> rows;
    std::string line;
    while (std::getline(in, line)) {
        // np.loadtxt: everything from '#' on is a comment, and an empty row is skipped.
        const size_t hash = line.find('#');
        if (hash != std::string::npos) {
            line.erase(hash);
        }
        std::vector<double> row;
        std::istringstream tokens(line);
        std::string token;
        while (tokens >> token) {
            const char* start = token.c_str();
            char* end = nullptr;
            errno = 0;
            const double value = std::strtod(start, &end);
            if (end != start + token.size()) {
                log_and_throw_error(
                    "{}: '{}' is not a number np.loadtxt would accept",
                    sol_path.string(),
                    token);
            }
            row.push_back(value);
        }
        if (row.empty()) {
            continue;
        }
        if (!rows.empty() && row.size() != rows.front().size()) {
            log_and_throw_error(
                "{}: row {} has {} columns, the first row has {}",
                sol_path.string(),
                rows.size(),
                row.size(),
                rows.front().size());
        }
        rows.push_back(std::move(row));
    }
    if (rows.empty()) {
        log_and_throw_error("{} is empty", sol_path.string());
    }

    // numpy returns a 1-D array whenever the file has one row or one column, and
    // `step_write_deformed_msh` then makes it a single COLUMN with `u[:, None]`. Mirrored so a
    // degenerate solution file lands on the same shape on both engines.
    if (rows.size() == 1 || rows.front().size() == 1) {
        std::vector<std::vector<double>> flat;
        for (const auto& row : rows) {
            for (const double v : row) {
                flat.push_back({v});
            }
        }
        return flat;
    }
    return rows;
}

void write_deformed_msh(
    const std::filesystem::path& msh_path,
    const std::filesystem::path& sol_path,
    const std::filesystem::path& output_msh,
    const double scale)
{
    logger().info("  sol : {}", sol_path.string());
    const std::vector<std::vector<double>> u = load_solution_txt(sol_path);
    const size_t dim = u.front().size();

    const MshNodes nodes = read_msh_nodes(msh_path.string());
    const size_t n_nodes = nodes.tag_to_idx.size();
    if (u.size() != n_nodes) {
        log_and_throw_error(
            "solution.txt has {} rows but .msh has {} nodes",
            u.size(),
            n_nodes);
    }

    // `u_mesh = u / scale` for the whole array, THEN the componentwise add: two roundings, in the
    // Python's order.
    std::vector<std::array<double, 3>> deformed = nodes.coords;
    for (size_t i = 0; i < n_nodes; ++i) {
        for (size_t d = 0; d < dim && d < 3; ++d) {
            deformed[i][d] = deformed[i][d] + u[i][d] / scale;
        }
    }

    mshio::MshSpec spec = mshio::load_msh(msh_path.string());
    for (auto& block : spec.nodes.entity_blocks) {
        for (size_t i = 0; i < block.num_nodes_in_block; ++i) {
            const int64_t idx = nodes.tag_to_idx.at(static_cast<int64_t>(block.tags[i]));
            for (size_t d = 0; d < 3; ++d) {
                block.data[3 * i + d] = deformed[static_cast<size_t>(idx)][d];
            }
        }
    }
    // gmsh.option Mesh.MshFileVersion 4.1 and gmsh's default ASCII output; see the header for why
    // ASCII rather than the binary the reduced-mesh writer uses.
    spec.mesh_format.version = "4.1";
    spec.mesh_format.file_type = 0;

    if (!output_msh.parent_path().empty()) {
        std::filesystem::create_directories(output_msh.parent_path());
    }
    std::ofstream out(output_msh, std::ios::binary);
    if (!out.is_open()) {
        log_and_throw_error("Unable to open {} for writing", output_msh.string());
    }
    // Every double in the file -- the node coordinates, and the entity bounding boxes carried
    // over from the input -- is written as the shortest decimal that reads back as the same
    // double, so no coordinate loses a bit on the way out. gmsh, which the Python engine writes
    // this file through, instead prints "%.16g" and does lose the last bits of some of them; that
    // difference is deliberate and is what the write-back parity test now states.
    out.imbue(std::locale(out.getloc(), new ShortestRoundTripDoubles));
    mshio::save_msh(out, spec);
    out.close();

    logger().info("  Out : {}", output_msh.string());
}

} // namespace wmtk::components::polyfem_ops
