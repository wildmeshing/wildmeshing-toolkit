#include "read_image_msh.hpp"

#include <wmtk/SurfaceTagAttributes.h>
#include <algorithm>
#include <filesystem>
#include <map>
#include <set>
#include <wmtk/Types.hpp>
#include <wmtk/components/simwild/EmbedCurves.hpp>
#include <wmtk/components/simwild/EmbedSurface.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/predicates.hpp>

namespace wmtk::components::simwild {

Matrix3d get_input_transform_2d(const nlohmann::json& json_params, const size_t input_index)
{
    const nlohmann::json& its_j = json_params["input_transform"];

    if (input_index >= its_j.size() || its_j[input_index].size() == 0) {
        return Matrix3d::Identity();
    }
    if (its_j[input_index].size() != 3) {
        log_and_throw_error(
            "Input transform for input {} is invalid: a 2D input takes a 3x3 homogeneous "
            "matrix, not {}x{}.",
            input_index,
            its_j[input_index].size(),
            its_j[input_index].size());
    }

    const std::array<std::array<double, 3>, 3> it_arr = its_j[input_index];
    Matrix3d A;
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            A(i, j) = it_arr[i][j];
        }
    }
    logger().info("Input transform for input {}:\n{}", input_index, A);

    return A;
}

Matrix4d get_input_transform(const nlohmann::json& json_params, const size_t input_index)
{
    const nlohmann::json& its_j = json_params["input_transform"];

    if (input_index >= its_j.size()) {
        return Matrix4d::Identity();
    }
    if (its_j[input_index].size() == 0) {
        return Matrix4d::Identity();
    }
    if (its_j[input_index].size() != 4) {
        log_and_throw_error("Input transform for input {} is invalid.", input_index);
    }

    const std::array<std::array<double, 4>, 4> it_arr = its_j[input_index];
    Matrix4d A;
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            A(i, j) = it_arr[i][j];
        }
    }
    logger().info("Input transform for input {}:\n{}", input_index, A);

    return A;
}

void positive_orientation_3D(const MatrixXd& V, MatrixXi& T)
{
    const Vector3d p0 = V.row(T(0, 0));
    const Vector3d p1 = V.row(T(0, 1));
    const Vector3d p2 = V.row(T(0, 2));
    const Vector3d p3 = V.row(T(0, 3));

    wmtk::utils::predicates::exactinit();
    auto res = wmtk::utils::predicates::orient3d(p0, p1, p2, p3);
    int result;
    if (res == wmtk::utils::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == wmtk::utils::predicates::Orientation::NEGATIVE)
        result = -1;
    else {
        log_and_throw_error(
            "First tet is degenerate! Vertices: \n{},\n{},\n{},\n{}",
            p0,
            p1,
            p2,
            p3);
    }

    bool is_inverted = result >= 0;
    if (is_inverted) {
        logger().warn("First tet of input is inverted -> invert all tets.");
        T.col(2).swap(T.col(3));
    }
}

void positive_orientation_2D(const MatrixXd& V, MatrixXi& F)
{
    const Vector2d p0 = V.row(F(0, 0));
    const Vector2d p1 = V.row(F(0, 1));
    const Vector2d p2 = V.row(F(0, 2));

    wmtk::utils::predicates::exactinit();
    auto res = wmtk::utils::predicates::orient2d(p0, p1, p2);
    int result;
    if (res == wmtk::utils::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == wmtk::utils::predicates::Orientation::NEGATIVE)
        result = -1;
    else {
        log_and_throw_error("First face is degenerate! Vertices: \n{},\n{},\n{}", p0, p1, p2);
    }

    bool is_inverted = result <= 0;
    if (is_inverted) {
        logger().warn("First face of input is inverted -> invert all faces.");
        F.col(1).swap(F.col(2));
    }
}

void tags_from_label_images(MeshData& mesh, const std::string& what)
{
    // A 2D mesh without regions may carry label images instead: integer cell fields tag_<k>
    // whose value v > 0 puts a cell in region v of image k (img_to_msh_2d.py writes these).
    // Regions are numbered across the images in k order and named tag_0, tag_1, ...
    const auto top =
        mesh.cells.count(CellType::Tetrahedron) ? CellType::Tetrahedron : CellType::Triangle;
    if (auto block = mesh.cells.find(top);
        block != mesh.cells.end() && !block->second.attributes.count("tags")) {
        std::map<int, DataMatrix<double>> images;
        for (const auto& [name, a] : block->second.attributes)
            if (name.rfind("tag_", 0) == 0) images[std::stoi(name.substr(4))] = a.doubles();
        if (!images.empty() && top == CellType::Tetrahedron)
            log_and_throw_error(
                "{}: 3D label images (tag_<k> cell fields) are not supported",
                what);
        std::vector<std::string> labels;
        std::vector<std::pair<int, int>> columns; // (image, value) per region
        for (const auto& [k, img] : images)
            for (int v = 1; v <= int(img.maxCoeff()); ++v) {
                columns.emplace_back(k, v);
                labels.push_back("tag_" + std::to_string(labels.size()));
            }
        if (!images.empty()) {
            const auto rows = block->second.connectivity.rows();
            DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Zero(rows, columns.size());
            for (Eigen::Index i = 0; i < rows; ++i)
                for (size_t j = 0; j < columns.size(); ++j)
                    member(i, j) = int(images.at(columns[j].first)(i, 0)) == columns[j].second;
            block->second.attributes["tags"] = {std::move(member), std::move(labels)};
            for (const auto& [k, img] : images)
                block->second.attributes.erase("tag_" + std::to_string(k));
        }
    }
}

InputData read_image_msh(const std::string& path)
{
    if (!std::filesystem::exists(path)) {
        log_and_throw_error("File {} does not exist.", path);
    }
    // The .msh route reads through the shared reader, so a file and the mesh it holds give the
    // components the same input. As it always has, it takes the geometry and the region
    // membership only: other fields in a file (diagnostics, older per-tag scalars such as
    // tag_0) are not inputs.
    MeshData mesh = io::read_mesh(path);
    tags_from_label_images(mesh, path);
    mesh.vertex_attributes.clear();
    for (auto& [type, cells] : mesh.cells)
        for (auto it = cells.attributes.begin(); it != cells.attributes.end();)
            it = it->first == "tags" ? std::next(it) : cells.attributes.erase(it);
    return read_mesh_data(mesh);
}

InputData read_mesh(
    const std::vector<std::string>& input_paths,
    const std::string& output_filename,
    const nlohmann::json& json_params,
    const io::ComponentIO* component_io)
{
    InputData input_data;

    std::vector<Matrix4d> input_transforms(input_paths.size());
    for (size_t i = 0; i < input_transforms.size(); ++i) {
        input_transforms[i] = get_input_transform(json_params, i);
    }

    const int NUM_THREADS = json_params["num_threads"];
    const bool debug_output = !component_io && bool(json_params["DEBUG_output"]);
    const bool perform_sanity_checks = json_params["DEBUG_sanity_checks"];
    const bool preserve_topology = json_params["preserve_topology"];
    const bool skip_simplify = json_params["skip_simplify"];
    const bool tag_from_winding_number = json_params["tag_from_winding_number"];
    const bool split_connected_components = json_params["split_connected_components"];
    const double epsr_simplify = json_params["eps_simplify_rel"];
    double eps_simplify = json_params["eps_simplify"];
    const bool simplify_boundary_envelope = json_params["simplify_boundary_envelope"];
    const double order2_envelope_ratio = json_params["order2_envelope_ratio"];
    const std::vector<std::string> input_names = json_params["input_names"];
    // Which inputs enclose a volume (tet tags) and which stay surfaces (face tags).
    const std::vector<std::string> input_type = json_params["input_type"];
    std::vector<bool> is_volume(input_paths.size(), true);
    if (!input_type.empty()) {
        if (input_type.size() != input_paths.size())
            log_and_throw_error(
                "input_type needs one entry per input: {} entries for {} inputs",
                input_type.size(),
                input_paths.size());
        for (size_t i = 0; i < input_type.size(); ++i) is_volume[i] = input_type[i] == "volume";
    }
    const bool has_surface = std::count(is_volume.begin(), is_volume.end(), false) > 0;
    if (has_surface && split_connected_components)
        log_and_throw_error(
            "Surface inputs (input_type surface) need split_connected_components off");
    if (has_surface && !tag_from_winding_number)
        log_and_throw_error("Surface inputs (input_type surface) need tag_from_winding_number");

    const double remove_duplicate_eps = json_params["remove_duplicate_eps"];
    const double tol_rel = remove_duplicate_eps;
    const double tol_abs = -1;

    const bool record_history = !std::string(json_params["history_output"]).empty();

    // convert mesh into tet mesh
    EmbedSurface image_mesh(
        input_paths,
        input_transforms,
        tol_rel,
        tol_abs,
        split_connected_components,
        component_io,
        record_history);
    image_mesh.m_num_threads = NUM_THREADS;
    image_mesh.m_perform_sanity_checks = perform_sanity_checks;

    if (debug_output) {
        image_mesh.write_surf_off(output_filename + "_input.off");
    }

    if (!preserve_topology && !skip_simplify) {
        /**
         * Simplify the input surface only if topology preservation is not required, since
         * simplification can change the topology. If topology preservation is required,
         * simplification is performed after insertion.
         */
        auto [bbox_min, bbox_max] = image_mesh.bbox_surf_minmax();
        double diag = (bbox_max - bbox_min).norm();
        if (epsr_simplify > 0) {
            eps_simplify = diag * epsr_simplify;
        }
        logger().info("Simplify before insertion with eps = {:.4}", eps_simplify);

        // The open boundary. Frozen, the simplification cannot touch anything enclosed by
        // boundary loops packed too tightly to leave a free vertex between them -- on Thingi10K
        // 55928 a crumpled knot holding every boundary edge of the model, which survives as a
        // non-manifold tangle and costs tetwild ~570 tiny tets (see tetwild.cpp). Within a tube
        // around the input's boundary edges instead, it coarsens like the rest of the surface.
        //
        // Scaled as tetwild scales it: the simplification counterpart of the optimizer's
        // order-2 envelope, i.e. order2_envelope_ratio of the simplification's surface eps.
        // Only reached with preserve_topology off; under it the surface is simplified after
        // insertion instead, by SimWildMesh::simplify against its own order-2 envelope.
        const double boundary_eps = simplify_boundary_envelope && eps_simplify > 0
                                        ? eps_simplify * order2_envelope_ratio
                                        : 0.0;
        if (boundary_eps > 0) {
            logger().info("simplification boundary envelope eps {:.4}", boundary_eps);
        } else {
            logger().info("simplification freezes the open boundary");
        }

        image_mesh.simplify_surface(
            eps_simplify,
            !json_params["simplify_use_sample_envelope"],
            NUM_THREADS,
            boundary_eps);

        if (debug_output) {
            image_mesh.write_surf_off(output_filename + "_input_simplified.off");
        }
    }

    input_data.V_envelope = image_mesh.V_surface();
    input_data.F_envelope = image_mesh.F_surface();

    const bool all_rounded =
        image_mesh.embed_surface(preserve_topology, tag_from_winding_number, has_surface);
    image_mesh.consolidate();

    if (debug_output) {
        image_mesh.write_emb_surf_off(output_filename + "_input_emb.off");
    }

    input_data.V_input = image_mesh.V_emb();
    if (!all_rounded) {
        input_data.V_input_r = image_mesh.V_emb_r();
    }
    input_data.T_input = image_mesh.T_emb();
    if (record_history) {
        input_data.has_input_history = true;
        input_data.input_history = image_mesh.input_history();
        input_data.facet_sources = std::move(image_mesh.facet_sources());
    }
    const auto& T_tags = image_mesh.T_tags();
    // An input's name, or tag_<i> without one.
    auto name = [&](size_t i) {
        return i < input_names.size() ? input_names[i] : "tag_" + std::to_string(i);
    };
    // Each input's label bit, as the mesh's faces will carry it: a surface input's face-tag
    // column, a volume input's tet tag after the face tags.
    std::vector<int64_t> input_bit(is_volume.size(), -1);
    if (!has_surface) {
        input_data.T_input_tag = T_tags;
        for (int i = 0; i < T_tags.cols(); ++i) input_data.tag_names.push_back(name(i));
        for (size_t i = 0; i < input_bit.size(); ++i) input_bit[i] = int64_t(i);
    } else {
        // A volume input keeps its column of tet tags; a surface input's column (its winding
        // number means nothing for an open surface) is dropped, and the embedded faces it tiles
        // get its name as a face tag instead.
        std::vector<int> volume_column(is_volume.size(), -1), surface_column(is_volume.size(), -1);
        for (size_t i = 0; i < is_volume.size(); ++i) {
            if (is_volume[i]) {
                volume_column[i] = int(input_data.tag_names.size());
                input_data.tag_names.push_back(name(i));
            } else {
                surface_column[i] = int(input_data.face_tag_names.size());
                input_data.face_tag_names.push_back(name(i));
            }
        }
        for (size_t i = 0; i < is_volume.size(); ++i) {
            if (surface_column[i] >= 0) {
                input_bit[i] = surface_column[i];
            } else {
                input_bit[i] = int64_t(input_data.face_tag_names.size()) + volume_column[i];
            }
        }
        input_data.T_input_tag.resize(T_tags.rows(), input_data.tag_names.size());
        for (int k = 0; k < T_tags.outerSize(); ++k)
            for (MatrixSi::InnerIterator it(T_tags, k); it; ++it)
                if (it.value() != 0 && volume_column[it.col()] >= 0)
                    input_data.T_input_tag.coeffRef(it.row(), volume_column[it.col()]) = 1;
        const auto& F = image_mesh.F_on_surface();
        const auto& tiles = image_mesh.F_tags_surface();
        std::vector<int> rows;
        for (int r = 0; r < F.rows(); ++r)
            for (const size_t in : tiles[r])
                if (surface_column[in] >= 0) {
                    rows.push_back(r);
                    break;
                }
        input_data.F_tagged.resize(rows.size(), 3);
        input_data.F_tags.resize(rows.size(), input_data.face_tag_names.size());
        // A face points as its surface inputs do: the sum of their directions on it, opposite
        // coincident sheets cancelling (SurfaceTagAttributes::m_orientation). The remesher's
        // vertex order carries none; a face whose sum is 0 keeps it, as the writer does.
        const auto& facing = image_mesh.F_tags_surface_orientation();
        for (size_t k = 0; k < rows.size(); ++k) {
            input_data.F_tagged.row(k) = F.row(rows[k]);
            int sum = 0;
            for (size_t j = 0; j < tiles[rows[k]].size(); ++j) {
                const size_t in = tiles[rows[k]][j];
                if (surface_column[in] < 0) continue;
                input_data.F_tags.coeffRef(k, surface_column[in]) = 1;
                sum += facing[rows[k]][j];
            }
            if (sum < 0) std::swap(input_data.F_tagged(k, 1), input_data.F_tagged(k, 2));
        }
    }

    {
        const auto& F_input = image_mesh.F_input();
        // Only where every envelope triangle's input is known; else the mesh's faces serve.
        if (F_input.size() == size_t(input_data.F_envelope.rows())) {
            input_data.F_envelope_labels.assign(input_data.F_envelope.rows(), 0);
            for (Eigen::Index r = 0; r < input_data.F_envelope.rows(); ++r) {
                const size_t in = F_input[r];
                if (in < input_bit.size() && input_bit[in] >= 0 &&
                    input_bit[in] < SurfaceTagAttributes::max_inputs) {
                    input_data.F_envelope_labels[r] = InputSet(1) << input_bit[in];
                }
            }
        }
    }

    wmtk::logger().info("======= finish image-tet conversion =========");

    return input_data;
}

InputData read_curves(
    const std::vector<std::string>& input_paths,
    const std::string& output_filename,
    const nlohmann::json& json_params,
    const io::ComponentIO* component_io)
{
    {
        const std::vector<std::string> input_type = json_params["input_type"];
        if (std::count(input_type.begin(), input_type.end(), "surface"))
            log_and_throw_error("Surface inputs (input_type surface) are 3D only");
    }
    InputData input_data;

    std::vector<Matrix3d> input_transforms(input_paths.size());
    for (size_t i = 0; i < input_transforms.size(); ++i) {
        input_transforms[i] = get_input_transform_2d(json_params, i);
    }

    const int NUM_THREADS = json_params["num_threads"];
    const bool debug_output = !component_io && bool(json_params["DEBUG_output"]);
    const bool preserve_topology = json_params["preserve_topology"];
    const bool skip_simplify = json_params["skip_simplify"];
    const double epsr_simplify = json_params["eps_simplify_rel"];
    double eps_simplify = json_params["eps_simplify"];
    const std::vector<std::string> input_names = json_params["input_names"];

    double tol_rel = -1;
    double tol_abs = -1;
    if (!preserve_topology) {
        tol_rel = 0.1 * epsr_simplify;
        tol_abs = 0.1 * eps_simplify;
    }

    EmbedCurves curves(input_paths, input_transforms, tol_rel, tol_abs, component_io);
    curves.m_num_threads = NUM_THREADS;

    if (debug_output) {
        curves.write_curves_obj(output_filename + "_input.obj");
    }

    if (!preserve_topology && !skip_simplify) {
        // Same condition the 3D route uses: simplification can change the topology of the
        // input, so it only runs when the caller has said that is acceptable.
        const auto [bbox_min, bbox_max] = curves.bbox_curves_minmax();
        const double diag = (bbox_max - bbox_min).norm();
        if (epsr_simplify > 0) {
            eps_simplify = diag * epsr_simplify;
        }
        curves.simplify_curves(
            eps_simplify,
            !json_params["simplify_use_sample_envelope"],
            NUM_THREADS);

        if (debug_output) {
            curves.write_curves_obj(output_filename + "_input_simplified.obj");
        }
    }

    // The envelope is built from the SIMPLIFIED input curves, taken here -- after
    // simplification, before the arrangement -- exactly as read_mesh does in 3D.
    input_data.V_envelope = curves.V_curves();
    input_data.F_envelope = curves.E_curves();

    const bool all_rounded = curves.embed_curves(json_params["tag_from_winding_number"]);
    curves.consolidate();

    input_data.V_input = curves.V_emb();
    if (!all_rounded) {
        input_data.V_input_r = curves.V_emb_r();
    }
    input_data.T_input = curves.F_emb();
    input_data.T_input_tag = curves.F_tags();
    for (int i = 0; i < input_data.T_input_tag.cols(); ++i) {
        input_data.tag_names.push_back(
            i < int(input_names.size()) ? input_names[i] : "tag_" + std::to_string(i));
    }

    wmtk::logger().info("======= finish curve-triangle conversion =========");

    return input_data;
}

int resolve_input_dimension(
    const std::vector<std::string>& input_paths,
    const nlohmann::json& json_params,
    const io::ComponentIO* component_io)
{
    if (component_io) {
        // A memory input states its topology, so a forced dimension is always checked: a
        // tagged tet mesh is 3D and a tagged triangle mesh 2D (both take the tagged-mesh route,
        // 0); an untagged triangle surface is 3D and a curve network 2D.
        const auto& data = component_io->mesh(input_paths.at(0));
        const bool tets = data.cells.count(CellType::Tetrahedron) != 0;
        const bool triangles = data.cells.count(CellType::Triangle) != 0;
        // Regions, or label images (tag_<k> cell fields) that tags_from_label_images turns into
        // regions.
        const auto has_regions = [](const MeshAttributes& attributes) {
            for (const auto& [name, field] : attributes)
                if (name == "tags" || name.rfind("tag_", 0) == 0) return true;
            return false;
        };
        const bool tagged =
            tets || (triangles && has_regions(data.cell(CellType::Triangle).attributes));
        const int topology = tets || (triangles && !tagged) ? 3 : 2;
        const std::string d = json_params.value("dimension", std::string("auto"));
        if (d != "auto") {
            const int forced = std::stoi(d);
            if (forced != 2 && forced != 3) {
                log_and_throw_error("dimension must be \"auto\", \"2\" or \"3\", not \"{}\"", d);
            }
            if (forced != topology) {
                log_and_throw_error(
                    "dimension is set to {} but input mesh {} is a {}D input ({}). Remove the "
                    "dimension key to accept the detected one.",
                    forced,
                    input_paths[0],
                    topology,
                    tets        ? "tagged tetrahedra"
                    : tagged    ? "tagged triangles"
                    : triangles ? "a triangle surface"
                                : "curves");
            }
        }
        return tagged ? 0 : topology;
    }

    // 0 means "let the .msh reader decide from the file's content", which is what it has
    // always done: a msh with faces and no tets is 2D, one with tets is 3D.
    const std::string extension = std::filesystem::path(input_paths[0]).extension().string();

    int sniffed = 0;
    if (extension == ".msh") {
        sniffed = 0; // let the msh reader decide
    } else if (extension == ".obj") {
        // A curve network and a surface are both OBJ, so the extension cannot separate them.
        // 'f' records mean a surface; 'l' records with no 'f' mean a curve network.
        bool has_f = false, has_l = false;
        std::ifstream in(input_paths[0]);
        if (!in) {
            log_and_throw_error("Could not open input file {}", input_paths[0]);
        }
        std::string line;
        while (std::getline(in, line)) {
            if (line.size() < 2 || line[1] != ' ') {
                continue;
            }
            if (line[0] == 'f') {
                has_f = true;
                break; // a surface: no need to read further
            }
            if (line[0] == 'l') {
                has_l = true;
            }
        }
        sniffed = has_f ? 3 : (has_l ? 2 : 3);
    } else {
        logger().warn(
            "Cannot determine dimension from file with extension {}. Assuming 3D surface.",
            extension);
        sniffed = 3;
    }

    const std::string d = json_params.value("dimension", std::string("auto"));
    if (d == "auto") {
        logger().info(
            "dimension: auto -> {}",
            sniffed == 0 ? "from the msh content" : (sniffed == 2 ? "2D curves" : "3D surface"));
        return sniffed;
    }

    const int forced = std::stoi(d);
    if (forced != 2 && forced != 3) {
        log_and_throw_error("dimension must be \"auto\", \"2\" or \"3\", not \"{}\"", d);
    }
    if (sniffed != 0 && sniffed != forced) {
        // Disagree loudly rather than proceeding: without the sniff, a curve network read as
        // a surface surfaces much later as an empty-face assertion inside EmbedSurface.
        log_and_throw_error(
            "dimension is set to {} but {} looks like a {}D input ({}). Remove the dimension "
            "key to accept the detected one.",
            forced,
            input_paths[0],
            sniffed,
            sniffed == 2 ? "OBJ with 'l' polylines and no 'f' faces" : "OBJ with 'f' faces");
    }
    logger().info("dimension: {} (forced)", forced);
    return forced;
}


InputData read_mesh_data(const MeshData& mesh, const MeshData* envelope)
{
    mesh.validate();
    const auto type =
        mesh.cells.count(CellType::Tetrahedron) ? CellType::Tetrahedron : CellType::Triangle;
    const auto& block = mesh.cell(type);
    if (!block.connectivity.rows()) throw std::runtime_error("Empty tagged mesh");
    io::ComponentIO::attributes(
        mesh.vertex_attributes,
        {"sizing_field",
         "sizing_scalar",
         "vid",
         "labels",
         "is_rounded",
         "is_false_surface",
         "order",
         "input_distance",
         "distance_over_target",
         "front_error_bars",
         "band_force_n",
         "amips_force_n",
         "offset_foldover",
         "front_last_change",
         "corr_input_vid",
         "front_ring_ratio"});
    const std::set<std::string> fields = {
        "tags",
        "quality",
        "quality_target",
        "quality_rel",
        "offset_tag",
        "amips",
        "tet_energy",
        "amips_term",
        "band_term",
        "band_gap_bars",
        "plastic",
        "face_energy",
        "front_err_ratio",
        "settle_inside_bars",
        "settle_outside_bars",
        "chord_length"};
    // The one lower-dimensional block allowed next to the top cells: in a tet mesh, triangles,
    // which are tagged faces; in a triangle mesh, edges, which are an embedded envelope, as a
    // lower physical group is on the .msh route.
    const auto envelope_type = type == CellType::Tetrahedron ? CellType::Triangle : CellType::Edge;
    for (const auto& [t, c] : mesh.cells) {
        if (t != type && t != envelope_type)
            throw std::runtime_error("Incompatible tagged mesh topology");
        io::ComponentIO::attributes(c.attributes, fields, std::set<std::string>{});
    }
    const auto lower = mesh.cells.find(envelope_type);
    const bool has_lower = lower != mesh.cells.end() && lower->second.connectivity.rows();
    const bool has_embedded = has_lower && type == CellType::Triangle;
    if (has_embedded && envelope)
        throw std::runtime_error("Tagged mesh has an embedded envelope and an `envelope` input");
    // Cells without a tag are the background; a mesh without a tags field is all background.
    const auto found = block.attributes.find("tags");
    const bool tagged = found != block.attributes.end();
    const MeshAttribute untagged{
        DataMatrix<uint8_t>::Zero(block.connectivity.rows(), 0).eval(),
        {}};
    const auto& a = tagged ? found->second : untagged;
    if (!std::holds_alternative<DataMatrix<uint8_t>>(a.values) ||
        a.labels.size() != size_t(a.cols()))
        throw std::runtime_error("tags must be a UInt8 membership matrix with column labels");
    const auto& tags = std::get<DataMatrix<uint8_t>>(a.values);
    if ((tags.array() > 1).any()) throw std::runtime_error("tags must be binary");
    InputData out;
    const int dim = type == CellType::Tetrahedron ? 3 : 2;
    // The vertices of `cells` only, renumbered in order, so an embedded envelope's vertices do
    // not enter V_input (and its bbox).
    auto referenced = [&](const DataMatrix<int64_t>& cells, MatrixXd& V, MatrixXi& C) {
        std::vector<int> remap(mesh.V.rows(), -1);
        for (Eigen::Index i = 0; i < cells.size(); ++i) remap[cells.data()[i]] = 0;
        int n = 0;
        for (auto& r : remap)
            if (r == 0) r = n++;
        V.resize(n, dim);
        for (Eigen::Index v = 0; v < mesh.V.rows(); ++v)
            if (remap[v] >= 0) V.row(remap[v]) = mesh.V.row(v).head(dim);
        C = io::ComponentIO::indices(cells);
        for (Eigen::Index i = 0; i < C.size(); ++i) C.data()[i] = remap[C.data()[i]];
        return remap;
    };
    const auto remap = referenced(block.connectivity, out.V_input, out.T_input);
    // An earlier offset's correspondence names the `vid` of each offset vertex's input vertex (a
    // mesh row when there is no `vid`, as prismatic_mesh reads it); translated to V_input rows.
    if (const auto c = mesh.vertex_attributes.find("corr_input_vid");
        c != mesh.vertex_attributes.end()) {
        const auto corr = c->second.integers();
        const auto v = mesh.vertex_attributes.find("vid");
        const bool has_vid = v != mesh.vertex_attributes.end();
        const auto vid = has_vid ? v->second.integers() : DataMatrix<int64_t>();
        if (corr.cols() != 1 || (has_vid && vid.cols() != 1))
            throw std::runtime_error("corr_input_vid and vid must be scalar");
        std::map<int64_t, Eigen::Index> row_of;
        for (Eigen::Index r = 0; r < mesh.V.rows(); ++r)
            if (!row_of.emplace(has_vid ? vid(r, 0) : r, r).second)
                throw std::runtime_error("vid must be unique");
        out.corr_input_vid.assign(out.V_input.rows(), -1);
        for (Eigen::Index r = 0; r < mesh.V.rows(); ++r) {
            if (remap[r] < 0 || corr(r, 0) < 0) continue;
            const auto it = row_of.find(corr(r, 0));
            if (it == row_of.end() || remap[it->second] < 0)
                throw std::runtime_error(
                    fmt::format("corr_input_vid {} names no vertex of the tets", corr(r, 0)));
            out.corr_input_vid[remap[r]] = remap[it->second];
        }
    }
    // A 2D input may come clockwise; it is turned around, as the .msh route always did for
    // images. A counter-clockwise input is unchanged.
    if (dim == 2) positive_orientation_2D(out.V_input, out.T_input);
    std::vector<int> columns;
    for (int j = 0; j < tags.cols(); ++j)
        if (a.labels[j] != "ambient" && a.labels[j] != "ImageVolume") {
            columns.push_back(j);
            out.tag_names.push_back(a.labels[j]);
        }
    out.T_input_tag.resize(tags.rows(), columns.size());
    for (int i = 0; i < tags.rows(); ++i)
        for (size_t j = 0; j < columns.size(); ++j)
            if (tags(i, columns[j])) out.T_input_tag.coeffRef(i, j) = 1;
    if (has_lower && type == CellType::Tetrahedron) {
        // The triangle block. Rows labelled EnvelopeSurface, simwild's envelope group (a reserved
        // name, like ambient), are the envelope; every other label is a face tag. A face tag is
        // one bit of a face's input set, so at most max_inputs, and a label names a volume (tets)
        // or a surface (faces), never both.
        const auto& faces = lower->second;
        const auto ft_it = faces.attributes.find("tags");
        if (ft_it == faces.attributes.end())
            throw std::runtime_error(
                "A triangle block in a tet mesh holds tagged faces; it needs tags");
        const auto& fa = ft_it->second;
        if (!std::holds_alternative<DataMatrix<uint8_t>>(fa.values) ||
            fa.labels.size() != size_t(fa.cols()))
            throw std::runtime_error("tags must be a UInt8 membership matrix with column labels");
        const auto& ft = std::get<DataMatrix<uint8_t>>(fa.values);
        if ((ft.array() > 1).any()) throw std::runtime_error("tags must be binary");
        if ((ft.cast<int>().rowwise().sum().array() == 0).any())
            throw std::runtime_error("Every tagged face needs a tag");
        std::vector<int> face_columns;
        int envelope_column = -1;
        for (int j = 0; j < ft.cols(); ++j) {
            if (fa.labels[j] == "EnvelopeSurface") {
                envelope_column = j;
                continue;
            }
            if (std::count(a.labels.begin(), a.labels.end(), fa.labels[j]))
                throw std::runtime_error("Tag " + fa.labels[j] + " names both tets and faces");
            face_columns.push_back(j);
            out.face_tag_names.push_back(fa.labels[j]);
        }
        if (face_columns.size() > size_t(SurfaceTagAttributes::max_inputs))
            throw std::runtime_error(
                fmt::format(
                    "At most {} face tags, not {}",
                    SurfaceTagAttributes::max_inputs,
                    face_columns.size()));
        std::vector<Eigen::Index> envelope_rows, face_rows;
        for (Eigen::Index i = 0; i < ft.rows(); ++i) {
            if (envelope_column >= 0 && ft(i, envelope_column)) envelope_rows.push_back(i);
            for (const int j : face_columns)
                if (ft(i, j)) {
                    face_rows.push_back(i);
                    break;
                }
        }
        if (!envelope_rows.empty()) {
            if (envelope)
                throw std::runtime_error(
                    "Tagged mesh has an embedded envelope and an `envelope` input");
            DataMatrix<int64_t> cells(envelope_rows.size(), 3);
            for (size_t k = 0; k < envelope_rows.size(); ++k)
                cells.row(k) = faces.connectivity.row(envelope_rows[k]);
            referenced(cells, out.V_envelope, out.F_envelope);
            out.envelope_name = "EnvelopeSurface";
        }
        out.F_tagged.resize(face_rows.size(), 3);
        out.F_tags.resize(face_rows.size(), face_columns.size());
        for (size_t k = 0; k < face_rows.size(); ++k) {
            for (int c = 0; c < 3; ++c) {
                const int v = remap[faces.connectivity(face_rows[k], c)];
                if (v < 0) throw std::runtime_error("A tagged face has a vertex no tet has");
                out.F_tagged(k, c) = v;
            }
            for (size_t j = 0; j < face_columns.size(); ++j)
                if (ft(face_rows[k], face_columns[j])) out.F_tags.coeffRef(k, j) = 1;
        }
    }
    if (envelope) {
        envelope->validate();
        io::ComponentIO::attributes(envelope->vertex_attributes, {});
        const auto t = type == CellType::Tetrahedron ? CellType::Triangle : CellType::Edge;
        for (const auto& [ct, c] : envelope->cells) {
            if (ct != t) throw std::runtime_error("Incompatible envelope topology");
            io::ComponentIO::attributes(c.attributes, {});
        }
        out.V_envelope = envelope->V.leftCols(type == CellType::Tetrahedron ? 3 : 2);
        out.F_envelope = io::ComponentIO::indices(envelope->cell(t).connectivity);
        out.envelope_name = "EnvelopeSurface";
    }
    if (has_embedded) {
        referenced(lower->second.connectivity, out.V_envelope, out.F_envelope);
        const auto tag = lower->second.attributes.find("tags");
        out.envelope_name = tag != lower->second.attributes.end() && tag->second.cols() == 1 &&
                                    tag->second.labels.size() == 1
                                ? tag->second.labels[0]
                                : "EnvelopeSurface";
    }
    return out;
}

} // namespace wmtk::components::simwild