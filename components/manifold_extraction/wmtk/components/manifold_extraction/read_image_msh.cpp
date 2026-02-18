#include "read_image_msh.hpp"

#include <wmtk/utils/io.hpp>

namespace wmtk::components::manifold_extraction {

void read_image_msh(
    const std::string& path,
    MatrixXd& V_input,
    MatrixXi& T_input,
    MatrixXd& T_input_tag,
    const std::string& tag_label)
{
    MshData msh;
    msh.load(path);

    // load input verts and tets
    V_input.resize(msh.get_num_tet_vertices(), 3);
    T_input.resize(msh.get_num_tets(), 4);
    msh.extract_tet_vertices(
        [&V_input](size_t i, double x, double y, double z) { V_input.row(i) << x, y, z; });
    msh.extract_tets([&T_input](size_t i, size_t v0, size_t v1, size_t v2, size_t v3) {
        T_input.row(i) << v0, v1, v2, v3;
    });

    // check if data exists, is right size, and is scalar. if so, load. otherwise, error
    bool tag_found = false;
    auto m_spec = msh.m_spec;
    std::vector<std::string> found_labels;
    for (const auto& data : m_spec.element_data) {
        if (data.header.string_tags[0] == tag_label) { // desired tag label
            if ((data.header.int_tags.size() >= 3) &&
                (data.header.int_tags[2] == T_input.rows())) { // data size == tet size
                if (data.header.int_tags[1] != 1) { // value is not scalar. bad
                    log_and_throw_error(
                        "Per-tet scalar data expected, given {}-dimensional",
                        data.header.int_tags[1]);
                }

                tag_found = true;
                T_input_tag.resize(T_input.rows(), 1);
                auto entries = data.entries;
                int index = 0;
                for (const auto& entry : entries) {
                    T_input_tag(index, 0) = entry.data[0];
                    index++;
                }
            } else { // num entries != num tets
                log_and_throw_error(
                    "'{}' data size ({}) != num tets ({})",
                    tag_label,
                    data.header.int_tags[2],
                    T_input.rows());
            }
        } else { // not label trying to find
            found_labels.push_back(data.header.string_tags[0]);
        }
    }
    if (!tag_found) { // label not found in any data object in msh
        std::string found_labels_str = "";
        for (const std::string& str : found_labels) {
            found_labels_str += (" [" + str + "]");
        }
        log_and_throw_error(
            "data with label '{}' not found. found labels:{}",
            tag_label,
            found_labels_str);
    }

    { // inversion check
        const Vector3d p0 = V_input.row(T_input(0, 0));
        const Vector3d p1 = V_input.row(T_input(0, 1));
        const Vector3d p2 = V_input.row(T_input(0, 2));
        const Vector3d p3 = V_input.row(T_input(0, 3));

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

        bool is_inverted = result >= 0;
        if (is_inverted) {
            logger().warn("First tet of input is inverted -> invert all tets.");
            T_input.col(2).swap(T_input.col(3));
        }
        // end inversion check
    }
}

} // namespace wmtk::components::manifold_extraction