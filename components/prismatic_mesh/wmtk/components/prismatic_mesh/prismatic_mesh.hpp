#pragma once

#include <wmtk/TetMesh.h>
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace wmtk::components::prismatic_mesh {

struct OffsetComponent
{
    size_t input_vertex; // mesh row of the corresponding input vertex
    std::vector<size_t> vertices;
    Vector3d optimal_normal = Vector3d::Zero(); // unit direction, zero for singular components
    Vector3d target_position = Vector3d::Zero(); // valid only when !singular
    bool singular = true;
    std::string singular_reason;
};

struct JacobianSmoothingOptions
{
    int iterations = 0; // opt-in postprocess, after tet optimization and before hybrid conversion
    double target = 0.01; // det(J) / (twice input area * initial mean column length)
    double position_weight = 1e-3;
    double max_step_ratio = 0.25; // Euclidean bound relative to target/initial column thickness
    double max_displacement_ratio = 0.5; // total displacement from the post-tet-optimization mesh
};

struct BackgroundRemeshingOptions
{
    bool enabled = false;
    int passes = 2;
    double quality_threshold = 0.1; // mean ratio: 1 regular, 0 degenerate
    int max_operations = 500; // accepted swaps/moves per optimization iteration
    int max_attempts = 20000; // bounds work even when no operation is admissible
};

struct OptimizationOptions
{
    int iterations = 5;
    // Acceptance floor for new/modified shell tets. Background only requires positive volume.
    double min_tet_volume = 1e-12; // absolute signed volume, in coordinate units cubed
    int smoothing_max_backtracks = 40; // try alpha=1, then halve at most this many times
    bool keep_background_mesh = false; // include exterior tets in optimization validity checks
    JacobianSmoothingOptions jacobian_smoothing;
    BackgroundRemeshingOptions background_remeshing;
};

struct Tau22Element
{
    std::array<size_t, 2> input_vertices;
    // offset_vertices[i] corresponds to input_vertices[i]. All entries are mesh row IDs.
    std::array<size_t, 2> offset_vertices;
};

struct UnlockStatistics
{
    size_t candidates = 0;
    size_t attempted = 0;
    size_t unlocked = 0;
    size_t remaining = 0;
};

struct OptimizationIteration
{
    size_t collapse_attempts = 0;
    size_t collapses = 0;
    size_t smoothed_vertices = 0;
    size_t smoothing_failures = 0;
    size_t singular_skipped = 0;
    UnlockStatistics unlock;
};

// Row indices address the loaded mesh. Source IDs are preserved separately and need not
// equal row indices. The optimizer preserves vertex rows and synchronizes cell arrays
// after accepted collapses. Deleted cell rows are -1 inside a collapse pass and compacted
// before returning; direct external WMTK edits require equivalent synchronization.
struct PrismaticMeshInput
{
    std::unique_ptr<TetMesh> mesh;
    MatrixXd vertices;
    MatrixXi tetrahedra;
    std::vector<int> vertex_tags; // -1: other, 1: input, 2: offset
    std::vector<int64_t> source_vertex_ids;
    std::vector<int64_t> corr_input_vid; // source input ID, or -1
    std::vector<int64_t> corr_input_vertex; // resolved mesh row, or -1
    std::vector<size_t> input_vertices;
    std::vector<size_t> offset_vertices;
    std::vector<std::vector<size_t>> input_to_offset_vertices; // indexed by mesh row
    std::vector<int> input_cells; // tag_0
    std::vector<int> offset_tet_tags; // 1: offset band, -1: other; derived from VTU offset_tag
    // Without background remeshing, far background cells live outside mesh. A marked vertex
    // has a static incident cell even if its entire active one-ring disappears in a collapse.
    std::vector<bool> fixed_background_vertices;
    // Indexed by face.fid(*mesh). -1: non-offset; 1: three distinct correspondence IDs;
    // 2: exactly two equal; 3: all equal. Recompute after topology/correspondence changes.
    std::vector<int> offset_face_tags;
    std::vector<OffsetComponent> offset_components;
    std::vector<std::vector<size_t>> input_to_components; // indexed by input vertex row
    std::vector<int64_t> vertex_component_ids; // -1 outside active offset components
    std::vector<int> singular_vertex_tags; // 1: singular, 0: valid, -1: not active offset
    MatrixXd optimal_normals; // per-vertex unit directions; zero where no valid direction exists
    MatrixXd target_positions; // singular/non-offset entries retain current positions
    double input_average_edge_length = 0; // unique edges of the input surface in the band
    double target_thickness = 0;
    std::vector<OptimizationIteration> optimization_iterations;
    nlohmann::json jacobian_smoothing_report;
    nlohmann::json background_remeshing_report;
};

enum class HybridCellType : int { Tetrahedron = 10, Prism = 13, Pyramid = 14 };

struct HybridCell
{
    HybridCellType type = HybridCellType::Tetrahedron;
    std::vector<size_t> vertices; // VTK corner ordering; rows in PrismaticMeshInput::vertices
    std::vector<size_t> source_tets; // exact partition of the final reference tetrahedral mesh
    int64_t prism_candidate = -1;
};

struct PrismDominantMesh
{
    std::vector<HybridCell> cells;
    std::vector<size_t> source_tet_to_cell;
    nlohmann::json report;
};

// Matched, nonoverlapping six-corner band columns, including geometrically invalid prisms.
std::vector<HybridCell> prism_candidates_for_smoothing(const PrismaticMeshInput& input);
void smooth_prism_jacobians(
    PrismaticMeshInput& input,
    double min_tet_volume,
    const JacobianSmoothingOptions& options);

// Reconstruct the band, with input-apex pyramids and compatible prism triangulations.
// Updates band tetrahedra to the chosen reference splits, retaining vertex rows/attributes
// and fixed input/background cells. The report records every changed three-tet region.
// Prisms require positive Jacobians at 21 samples and all side-edge quadratic minima;
// unused bad tet splits do not reject the prism. The interface solve admits positive splits,
// requiring newly introduced tets to exceed the operation floor. Existing tets need only positive
// volume. Pyramids still require both base-diagonal decompositions to exceed the volume floor.
PrismDominantMesh build_prism_dominant_mesh(PrismaticMeshInput& input, double min_tet_volume);
// Throws on invalid geometry, incomplete/overlapping source ownership or nonconforming faces.
void validate_prism_dominant_mesh(
    const PrismaticMeshInput& input,
    const PrismDominantMesh& hybrid,
    double min_tet_volume);

// Tetrahedral VTU (ASCII/uncompressed inline binary) or Gmsh 4.1 MSH (ASCII/binary).
PrismaticMeshInput load_prismatic_mesh(const std::filesystem::path& path);
// Export tetrahedra or validated hybrid cells as VTU or Gmsh 4.1 ASCII (.msh),
// plus all input vertices and a companion offset-face VTU.
// Hybrid output includes oriented reference decompositions and a companion _hybrid.json report.
// include_background=false exports the union of input volume and offset band only,
// retaining reference tet IDs in the full reconstructed mesh.
void write_prismatic_mesh(
    const PrismaticMeshInput& input,
    const std::filesystem::path& path,
    const PrismDominantMesh* hybrid = nullptr,
    bool include_background = true);
// Keep band tetrahedra in their original order and filter cell attributes with them.
// Vertex arrays/IDs stay unchanged; unused vertices are inactive in the rebuilt TetMesh.
void keep_offset_band(PrismaticMeshInput& input);
void label_offset_faces(PrismaticMeshInput& input);
// Computes targets only; does not move vertices. Call on the band mesh.
void evaluate_target_positions(PrismaticMeshInput& input, double thicknessratio);
// Minimum-norm least-squares solution in the normals' span, using dense LU, then normalized.
// Returns false if no usable LS direction satisfies n.dot(direction) > 1e-8 for every normal.
bool solve_target_direction(const std::vector<Vector3d>& normals, Vector3d& direction);
// Exact signed-volume comparison on the stored double coordinates; zero/negative volumes fail.
bool tet_volume_above_threshold(
    const MatrixXd& vertices,
    const std::array<size_t, 4>& tet,
    double threshold);
// Keep the survivor's position/IDs. No state changes on rejection.
bool try_collapse_offset_edge(
    PrismaticMeshInput& input,
    size_t removed,
    size_t survivor,
    double min_tet_volume);
// Return accepted alpha along current->fixed target; zero means no move.
double smooth_offset_vertex(
    PrismaticMeshInput& input,
    size_t vertex,
    double min_tet_volume,
    int max_backtracks);
void optimize_prismatic_mesh(PrismaticMeshInput& input, const OptimizationOptions& options);
std::optional<Tau22Element> classify_tau22(const PrismaticMeshInput& input, size_t tet_id);
// side=0: split(input a,offset b), then collapse(x->offset a); side=1: symmetric.
// Atomic: rejection leaves the entire input unchanged. No original vertex is moved.
bool try_unlock_tau22(PrismaticMeshInput& input, size_t tet_id, int side, double min_tet_volume);
// One pass over the starting tau22 candidates; newly created tau22s wait for the next round.
UnlockStatistics unlock_tau22(PrismaticMeshInput& input, double min_tet_volume);
// Compute targets on the band, optionally include offset-incident background in optimization,
// then reattach fixed background/input volume for output. Background remeshing, when enabled,
// retains all background stars and can move their interior vertices and swap interior faces/edges.
void prism_main(
    PrismaticMeshInput& input,
    double thicknessratio = 0.1,
    const OptimizationOptions& optimization = {});
void prismatic_mesh(nlohmann::json json_params);

} // namespace wmtk::components::prismatic_mesh
