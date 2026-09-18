#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace wmtk::components::polyfem_ops {

/// What `polyfem_utils.read_msh_nodes` returns, in the order of its 3-tuple.
struct MshNodes
{
    std::vector<int64_t> tags; ///< the gmsh node tags, in file order
    std::vector<std::array<double, 3>> coords; ///< indexed by `tag_to_idx`, always 3 components
    std::map<int64_t, int64_t> tag_to_idx; ///< node tag -> row index
};

/**
 * @brief Read every node of a .msh. Mirrors `polyfem_utils.read_msh_nodes`.
 *
 * The index rule is the Python's: `tag - 1` when the largest tag equals the node count (the
 * contiguous case, which is also what polyfem's MshReader assumes), otherwise the rank in sorted
 * tag order. `solution.txt` is written in that same order -- polyfem's `reorder_nodes` puts it
 * back into input-node order -- so the rule is what lines the displacements up with the mesh.
 */
MshNodes read_msh_nodes(const std::string& msh_path);

/**
 * @brief Read polyfem's `solution.txt` as rows of float64. Mirrors `np.loadtxt` on it.
 *
 * Blank lines and '#' comments are skipped and every row must have the same width, as numpy
 * requires. numpy also SQUEEZES a single-row or single-column file to a 1-D array, which
 * `step_write_deformed_msh` then turns back into one column with `u[:, None]`; that is mirrored,
 * so a one-column solution (a 1-D problem) is read as one displacement component per node exactly
 * as the Python reads it.
 */
std::vector<std::vector<double>> load_solution_txt(const std::filesystem::path& sol_path);

/**
 * @brief Apply `solution.txt` to the ORIGINAL mesh and write the deformed one. Mirrors
 * `polyfem_utils.step_write_deformed_msh` and the `_write_deformed_msh` it calls.
 *
 * The displacements are in SOLVER units and are divided by `scale` to get mesh units; the whole
 * array is divided first and then added, as the Python does, so the two engines round at the same
 * two places. Only the first `dim` components move: in 2D the z coordinate is carried over
 * untouched. The write-back is deliberately applied to the original multi-tag mesh rather than to
 * the reduced one, which is what preserves the caller's full tag set on the output.
 *
 * Written as ASCII msh 4.1 -- the format gmsh writes, so the file stays readable by everything
 * that read it before -- but with every double printed as the shortest decimal that reads back as
 * the same double (`python_repr`), so the file carries the computed coordinates exactly. The
 * Python engine writes this file through gmsh, whose ASCII writer prints "%.16g" and so loses the
 * last bits of about 40% of the coordinates (measured: 122 of 300 pseudo-random values do not
 * survive that round trip). The two engines' deformed meshes therefore differ in the last bits of
 * some coordinates, by design: this one is the exact result. Every other section -- entities,
 * physical groups and names, element blocks -- is carried over from the input unchanged.
 */
void write_deformed_msh(
    const std::filesystem::path& msh_path,
    const std::filesystem::path& sol_path,
    const std::filesystem::path& output_msh,
    double scale);

} // namespace wmtk::components::polyfem_ops
