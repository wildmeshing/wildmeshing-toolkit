#include "read_image_msh.hpp"

#include <wmtk/utils/io.hpp>

namespace wmtk::components::topological_offset {

void read_image_msh(
    const std::string& path,
    MatrixXd& V_input,
    MatrixXi& T_input,
    MatrixXd& T_input_tags,
    std::map<std::string, int>& tag_label_map)
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

    // inversion check
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

    // define tag label -> index map
    int tet_tags_count = 0;
    for (const std::string& attr_name : msh.get_tet_attribute_names()) {
        tag_label_map[attr_name] = tet_tags_count;
        ++tet_tags_count;
    }
    assert(tet_tags_count > 0);

    // // TESTING
    // for (const auto& pair : tag_label_map) {
    //     logger().info("{}: {}", pair.first, pair.second);
    // }

    // extract tags
    T_input_tags.resize(T_input.rows(), tet_tags_count);
    for (const auto& pair : tag_label_map) {
        int tag_ind = pair.second;
        msh.extract_tet_attribute(
            pair.first,
            [&T_input_tags, &tag_ind](size_t i, std::vector<double> val) {
                assert(val.size() == 1);
                T_input_tags(i, tag_ind) = val[0];
            });
    }
}

} // namespace wmtk::components::topological_offset