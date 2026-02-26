#include "read_image_msh.hpp"
#include <wmtk/utils/io.hpp>


namespace wmtk::components::topological_offset {


void read_image_msh(
    const std::string& path,
    MatrixXd& V_input,
    MatrixXi& F_input,
    MatrixXd& F_input_tags,
    const std::string& tag_name,
    std::vector<std::string>& all_tag_names)
{
    MshData msh;
    msh.load(path);

    if (msh.get_num_tets() == 0) { // 2d triangle mesh
        V_input.resize(msh.get_num_face_vertices(), 2);
        F_input.resize(msh.get_num_faces(), 3);
        msh.extract_face_vertices(
            [&V_input](size_t i, double x, double y, double z) { V_input.row(i) << x, y; });
        msh.extract_faces([&F_input](size_t i, size_t v0, size_t v1, size_t v2) {
            F_input.row(i) << v0, v1, v2;
        });

        { // inversion check
            const Vector2d p0 = V_input.row(F_input(0, 0));
            const Vector2d p1 = V_input.row(F_input(0, 1));
            const Vector2d p2 = V_input.row(F_input(0, 2));

            igl::predicates::exactinit();
            auto res = igl::predicates::orient2d(p0, p1, p2);
            int result;
            if (res == igl::predicates::Orientation::POSITIVE) {
                result = 1;
            } else if (res == igl::predicates::Orientation::NEGATIVE) {
                result = -1;
            } else {
                log_and_throw_error(
                    "First tet is degenerate! Vertices: \n{},\n{},\n{}",
                    p0,
                    p1,
                    p2);
            }

            if (result <= 0) {
                logger().warn("First triangle of input is inverted -> invert all triangles.");
                F_input.col(1).swap(F_input.col(2));
            }
        }
    } else { // 3d tet mesh
        V_input.resize(msh.get_num_tet_vertices(), 3);
        F_input.resize(msh.get_num_tets(), 4);
        msh.extract_tet_vertices(
            [&V_input](size_t i, double x, double y, double z) { V_input.row(i) << x, y, z; });
        msh.extract_tets([&F_input](size_t i, size_t v0, size_t v1, size_t v2, size_t v3) {
            F_input.row(i) << v0, v1, v2, v3;
        });

        { // inversion check
            const Vector3d p0 = V_input.row(F_input(0, 0));
            const Vector3d p1 = V_input.row(F_input(0, 1));
            const Vector3d p2 = V_input.row(F_input(0, 2));
            const Vector3d p3 = V_input.row(F_input(0, 3));

            igl::predicates::exactinit();
            auto res = igl::predicates::orient3d(p0, p1, p2, p3);
            int result;
            if (res == igl::predicates::Orientation::POSITIVE) {
                result = 1;
            } else if (res == igl::predicates::Orientation::NEGATIVE) {
                result = -1;
            } else {
                log_and_throw_error(
                    "First tet is degenerate! Vertices: \n{},\n{},\n{},\n{}",
                    p0,
                    p1,
                    p2,
                    p3);
            }

            if (result >= 0) {
                logger().warn("First tet of input is inverted -> invert all tets.");
                F_input.col(2).swap(F_input.col(3));
            }
        }
    }

    std::vector<MatrixXd> tag_vals; // for reading tag values
    // check if data exists, is right size, and is scalar. if so, save and update tag list
    auto m_spec = msh.m_spec;
    for (const auto& data : m_spec.element_data) {
        if ((data.header.int_tags.size() >= 3) &&
            (data.header.int_tags[2] == F_input.rows())) { // data size == tet size
            if (data.header.int_tags[1] != 1) { // value is not scalar. bad
                logger().warn(
                    "Non-scalar data [{}] ignored (size: {})",
                    data.header.string_tags[0],
                    data.header.int_tags[1]);
                continue;
            }

            all_tag_names.push_back(data.header.string_tags[0]); // get name
            MatrixXd curr_tags(F_input.rows(), 1);
            auto entries = data.entries;
            int index = 0;
            for (const auto& entry : entries) {
                curr_tags(index, 0) = entry.data[0];
                index++;
            }
            tag_vals.push_back(curr_tags);
        } else { // num entries != num tets
            logger().warn(
                "Size-mismatched facet data [{}] ignored ({} entries, {} facets)",
                data.header.string_tags[0],
                data.header.int_tags[2],
                F_input.rows());
        }
    }

    // throw error if desired tag name not found
    std::vector<std::string>::iterator it =
        std::find(all_tag_names.begin(), all_tag_names.end(), tag_name);
    if (it == all_tag_names.end()) {
        log_and_throw_error("Tag [{}] not found in input mesh '{}'", tag_name, path);
    }

    // save tag values to tag matrix
    int num_cols = tag_vals.size();
    F_input_tags.resize(F_input.rows(), tag_vals.size());
    for (int j = 0; j < num_cols; j++) {
        F_input_tags.col(j) = tag_vals[j];
    }
}


} // namespace wmtk::components::topological_offset