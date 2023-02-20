#include <spdlog/fmt/bundled/format.h>
#include <wmtk/TriMeshOperation.h>
#include <wmtk/utils/AttributeRecorder.h>
#include <wmtk/utils/OperationLogger.h>
#include <wmtk/utils/TriMeshOperationLogger.h>
#include <wmtk/AttributeCollection.hpp>
//#include <nlohmann/json.hpp>
#include <wmtk/utils/AttributeRecorder.h>
#include <wmtk/utils/OperationReplayer.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/TransportablePoints.hpp>

namespace {
struct TestVec2
{
    double x = 0;
    double y = 0;

    TestVec2 operator-(const TestVec2& o) const { return TestVec2{x - o.x, y - o.y}; }
    TestVec2 operator+(const TestVec2& o) const { return TestVec2{x + o.x, y + o.y}; }
    TestVec2 operator*(const double v) const { return TestVec2{v * x, v * y}; }
    double cross2d(const TestVec2& a) const { return this->x * a.y - this->y * a.x; }
    // returns the 2d (a-*this).cross(b-*this)
    double cross2d(const TestVec2& a, const TestVec2& b) const
    {
        const TestVec2 A = a - *this;
        const TestVec2 B = b - *this;
        return A.cross2d(B);
    }
    static HighFive::CompoundType datatype()
    {
        return HighFive::CompoundType{
            {"x", HighFive::create_datatype<double>()},
            {"y", HighFive::create_datatype<double>()}};
    }

    static TestVec2 weighted_add(
        const TestVec2& a,
        const TestVec2& b,
        const TestVec2& c,
        const std::array<double, 3>& pts)
    {
        return a * pts[0] + b * pts[1] + c * pts[2];
    }
    double norm() const { return std::sqrt(x * x + y * y); }
};


class SimpleMesh : public wmtk::TriMesh
{
public:
    SimpleMesh() { p_vertex_attrs = &vertices; }
    wmtk::AttributeCollection<TestVec2> vertices;
    bool point_in_triangle(const std::array<size_t, 3>& t, const TestVec2& p) const;
    std::array<double, 3> barycentric_coords(const std::array<size_t, 3>& t, const TestVec2& p)
        const;

private:
    // defines whether a point p is in a halfplane or not. Which halfplane is not specified in this
    // implementation
    bool in_halfplane(size_t a, size_t b, const TestVec2& p) const;
};
bool SimpleMesh::point_in_triangle(const std::array<size_t, 3>& t, const TestVec2& p) const
{
    //spdlog::info("{} / {}", fmt::join(t, ","), vertices.size());
    bool all_in = true;
    bool all_out = true;
    std::array<bool, 3> dat;
    spdlog::info("{} / {}", fmt::join(t, ","), vertices.size());
    for (size_t j = 0; j < 3; ++j) {
        bool in = in_halfplane(t[j], t[(j + 1) % 3], p);
        dat[j] = in;
        all_in &= in;
        all_out &= !in;
    }
    // spdlog::info("Point in triangle {} got {}", p, fmt::join(dat, ","));
    return all_in || all_out;
}
std::array<double, 3> SimpleMesh::barycentric_coords(
    const std::array<size_t, 3>& t,
    const TestVec2& p) const
{
    std::array<double, 3> ret;
    for (size_t j = 0; j < 3; ++j) {
        const TestVec2& a = vertices.at(t[(j + 1) % 3]);
        const TestVec2& b = vertices.at(t[(j + 2) % 3]);
        ret[j] = p.cross2d(a, b);
    }
    double T = std::accumulate(ret.begin(), ret.end(), double(0), std::plus<double>{});
    for (double& v : ret) {
        v /= T;
    }
    {
        const auto& a = vertices.at(t[0]);
        const auto& b = vertices.at(t[1]);
        const auto& c = vertices.at(t[2]);
        spdlog::info(
            "got barycentric({}) => {} (pt {})",
            p,
            fmt::join(ret, ","),
            TestVec2::weighted_add(a, b, c, ret));
        spdlog::warn("barycentric interp {} => {} {} {}", fmt::join(ret, ","), a, b, c);
    }
    return ret;
}
bool SimpleMesh::in_halfplane(size_t ai, size_t bi, const TestVec2& p) const
{
    const TestVec2& a = vertices.at(ai);
    const TestVec2& b = vertices.at(bi);
    return b.cross2d(p, a) > 0;
}

void make_diamond_mesh(SimpleMesh& mesh)
{
    std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}, {{3, 1, 0}}};

    mesh.create_mesh(4, tris);

    mesh.vertices[0].x = 0;
    mesh.vertices[0].y = 1;
    mesh.vertices[1].x = 1;
    mesh.vertices[1].y = 0;
    mesh.vertices[2].x = 1;
    mesh.vertices[2].y = 1;
    mesh.vertices[3].x = 0;
    mesh.vertices[3].y = 0;
}


} // namespace


template <>
struct fmt::formatter<TestVec2>
{
    template <typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template <typename FormatContext>
    auto format(TestVec2 const& vec, FormatContext& ctx)
    {
        return fmt::format_to(ctx.out(), "({},{})", vec.x, vec.y);
    }
};

struct SimpleMeshSplitEdgeOperation : public wmtk::TriMeshSplitEdgeOperation
{
    bool after_check(const ExecuteReturnData& ret_data, wmtk::TriMesh& m) override
    {
        auto& sm = dynamic_cast<SimpleMesh&>(m);
        auto [ta, tb] = original_endpoints(ret_data.tuple, m);
        auto tn = new_vertex(ret_data.tuple, m);
        const auto& a = sm.vertices[ta.vid(m)];
        const auto& b = sm.vertices[tb.vid(m)];


        sm.vertices[tn.vid(m)] = (a + b) * .5;
        return true;
    }
};


HIGHFIVE_REGISTER_TYPE(TestVec2, TestVec2::datatype)

WMTK_HDF5_REGISTER_ATTRIBUTE_TYPE(TestVec2)

#include <catch2/catch.hpp>
#include <iostream>

using namespace wmtk;


// WMTK_HDF5_REGISTER_ATTRIBUTE_TYPE(double)

TEST_CASE("create_recorder", "[attribute_recording]")
{
    /*
    using namespace HighFive;
    File file("create_recorder.hd5", File::ReadWrite | File::Create | File::Truncate);

    size_t size = 20;

    AttributeCollection<double> attribute_collection;
    attribute_collection.resize(size);
    AttributeCollectionRecorder<double> attribute_recorder(file, "attribute1",
    attribute_collection);

    for(size_t j = 0; j < attribute_collection.size(); ++j) {
        attribute_collection[j] = -double(j);
    }

    attribute_collection.begin_protect();


    for(size_t j = 0; j < attribute_collection.size(); ++j) {
        attribute_collection[j] = double(2 * j);
    }

    for(size_t j = 0; j < attribute_collection.size(); ++j) {
        REQUIRE(attribute_collection[j] == 2 * j);
    }

    const auto& rollback_list = attribute_collection.m_rollback_list.local();


    REQUIRE(rollback_list.size() <= attribute_collection.size());

    {
        auto [start,end] = attribute_recorder.record();
        CHECK(start==0);
        CHECK(end==20);
    }

    attribute_collection.end_protect();

    attribute_collection.begin_protect();
    for(size_t j = 0; j < attribute_collection.size(); j+=2) {
        attribute_collection[j] = double(3 * j);
    }
    {
        auto [start,finish] = attribute_recorder.record();
        CHECK(start == size);
        CHECK(finish == (size + size/2));
    }
    attribute_collection.end_protect();

    for(size_t j = 0; j < attribute_collection.size(); ++j) {
        if(j%2 == 0) {
        REQUIRE(attribute_collection[j] == 3 * j);
        } else {
        REQUIRE(attribute_collection[j] == 2 * j);
        }
    }

    */
}

auto edge_vids(const TriMesh& m, const TriMesh::Tuple& edge) -> std::array<size_t, 2>
{
    std::array<size_t, 2> ret;
    ret[0] = edge.vid(m);
    TriMesh::Tuple other = edge.switch_vertex(m);
    ret[1] = other.vid(m);
    return ret;
}


auto check_vertex_indices(
    const TriMesh& m,
    const TriMesh::Tuple& tuple,
    const std::set<size_t>& expected)
{
    REQUIRE(tuple.is_valid(m));
    int size = expected.size();
    std::set<size_t> orig;
    if (size == 3) {
        auto tri_vids = m.oriented_tri_vids(tuple);
        orig = {tri_vids.begin(), tri_vids.end()};
    } else if (size == 2) {
        auto e_vids = edge_vids(m, tuple);
        orig = {e_vids.begin(), e_vids.end()};
    }
    REQUIRE(orig == expected);
}

auto check_face_equality(const TriMesh& a, const TriMesh& b)
{
    auto a_face_tuples = a.get_faces();
    auto b_face_tuples = b.get_faces();
    REQUIRE(a_face_tuples.size() == b_face_tuples.size());
    for (size_t j = 0; j < a_face_tuples.size(); ++j) {
        const auto& a_tup = a_face_tuples[j];
        const auto& b_tup = b_face_tuples[j];
        auto [a_vid, a_eid, a_fid, a_hash] = a_tup.as_stl_tuple();
        auto [b_vid, b_eid, b_fid, b_hash] = b_tup.as_stl_tuple();
        CHECK(a_vid == b_vid);
        CHECK(a_eid == b_eid);
        CHECK(a_fid == b_fid);
    }
}
TEST_CASE("barycentric_with_swapping", "[test_2d_operation]")
{
    return;
    ExecutePass<TriMesh, ExecutionPolicy::kSeq> scheduler;


    SimpleMesh final_mesh;
    make_diamond_mesh(final_mesh);
    {
        // 0-----2
        // |\    |
        // | \   |
        // |  \  |
        // |   \ |
        // |    \|
        // 3-----1
        auto face_tuples = final_mesh.get_faces();
        REQUIRE(face_tuples.size() == 2);
        check_vertex_indices(final_mesh, face_tuples[0], {0, 1, 2});
        check_vertex_indices(final_mesh, face_tuples[1], {0, 1, 3});
    }


    // We will simultaneously track operations and run them to validate the logger

    {
        using namespace HighFive;
        spdlog::info("Creating replay file");
        File file("replay_operations_2d.hd5", File::ReadWrite | File::Create | File::Truncate);
        TriMeshOperationLogger op_logger(final_mesh, file);

        std::vector<std::pair<std::string, TriMesh::Tuple>> recorded_operations;

        AttributeCollectionRecorder vert_recorder(file, "vertices", final_mesh.vertices);

        // AttributeCollectionRecorder tri_recorder(file, "triangles",
        //         final_mesh.m_tri_connectivity);

        op_logger.add_attribute_recorder("vertices", vert_recorder);
        // op_recorder.add_attribute_recorder("triangles", tri_recorder);
        //  record do a flip, split, and then collapse
        // SECTION("logging_operations")
        //
        {
            std::vector<std::pair<std::string, TriMesh::Tuple>> operations;
            TriMesh::Tuple edge(0, 2, 0, final_mesh);


            REQUIRE(edge.is_valid(final_mesh));
            std::string op_name = "edge_swap";
            spdlog::info("Performing {}", op_name);
            scheduler.edit_operation_maps[op_name](final_mesh, edge);
            operations.emplace_back(op_name, edge);
            {
                // 0-----2
                // |    /|
                // |   / |
                // |  /  |
                // | /   |
                // |/    |
                // 3-----1
                auto face_tuples = final_mesh.get_faces();
                REQUIRE(face_tuples.size() == 2);
                check_vertex_indices(final_mesh, face_tuples[0], {0, 2, 3});
                check_vertex_indices(final_mesh, face_tuples[1], {1, 2, 3});
            }


            // flip the 2,3 edge
            edge = TriMesh::Tuple(2, 0, 0, final_mesh);
            REQUIRE(edge.is_valid(final_mesh));
            check_vertex_indices(final_mesh, edge, {2, 3});

            op_name = "edge_swap";
            spdlog::info("Performing {}", op_name);
            scheduler.edit_operation_maps[op_name](final_mesh, edge);
            operations.emplace_back(op_name, edge);
            {
                // 0-----2
                // |\    |
                // | \   |
                // |  \  |
                // |   \ |
                // |    \|
                // 3-----1
                auto face_tuples = final_mesh.get_faces();
                REQUIRE(face_tuples.size() == 2);
                {
                    auto tri_vids = final_mesh.oriented_tri_vids(face_tuples[0]);
                    spdlog::info("{}", fmt::join(tri_vids, ","));
                }
                {
                    auto tri_vids = final_mesh.oriented_tri_vids(face_tuples[1]);
                    spdlog::info("{}", fmt::join(tri_vids, ","));
                }
                check_vertex_indices(final_mesh, face_tuples[0], {0, 1, 2});
                check_vertex_indices(final_mesh, face_tuples[1], {3, 1, 0});
            }
        }
    }


    // SECTION("replay the old operations");
    using namespace HighFive;
    File file("replay_operations_2d.hd5", File::ReadOnly);
    spdlog::info("{}", file.getDataSet("operations").getElementCount());
    SimpleMesh m;
    make_diamond_mesh(m);

    TransportablePoints<TestVec2> points;
    TransportablePointsBase* points_base = &points;
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wshadow"
#elif (defined(__GNUC__) || defined(__GNUG__)) && !(defined(__clang__) || defined(__INTEL_COMPILER))
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#endif
    points.barycentric_interp_callback =
        [&](const TriMesh& m, const TriMesh::Tuple& t, const std::array<double, 3>& B) -> TestVec2 {
        spdlog::info("Start");
        ;
        const auto& sm = dynamic_cast<const SimpleMesh&>(m);
        auto [ai, bi, ci] = m.oriented_tri_vids(t);

        spdlog::info("{} {} {} / {}", ai, bi, ci, sm.vertices.size());
        ;
        const auto& a = sm.vertices.at(ai);
        const auto& b = sm.vertices.at(bi);
        const auto& c = sm.vertices.at(ci);
        return TestVec2::weighted_add(a, b, c, B);
    };
    points.point_in_triangle_callback =
        [&](const TriMesh& m, const TriMesh::Tuple& t, const TestVec2& point) -> bool {
        const auto& sm = dynamic_cast<const SimpleMesh&>(m);
        return sm.point_in_triangle(m.oriented_tri_vids(t), point);
    };
    points.get_barycentric_callback = [&](const TriMesh& m,
                                          const TriMesh::Tuple& t,
                                          const TestVec2& point) -> std::array<double, 3> {
        const auto& sm = dynamic_cast<const SimpleMesh&>(m);
        return sm.barycentric_coords(m.oriented_tri_vids(t), point);
    };
#if defined(__clang__)
#pragma clang diagnostic pop
#elif (defined(__GNUC__) || defined(__GNUG__)) && !(defined(__clang__) || defined(__INTEL_COMPILER))
#pragma GCC diagnostic pop
#endif

    std::vector<TestVec2> points_raw;
    points_raw.resize(10);
    Eigen::AlignedBox<double, 2> bbox(Eigen::Vector2d::Zero(), Eigen::Vector2d::Ones());
    for (auto& v : points_raw) {
        auto p = bbox.sample();
        v.x = p.x();
        v.y = p.y();
    }
    points.set_points(points_raw);

    // initialize remembered face tuple state
    std::vector<TriMesh::Tuple> face_tuples = m.get_faces();

    m.p_vertex_attrs = &m.vertices;

    spdlog::info("Input Points:: {}", fmt::join(points.points_global, ","));
    points.update_local_coordinates(m);
    TriMeshOperationLogger logger(m, file);
    logger.set_readonly();
    OperationReplayer replayer(m, logger);
    for (size_t j = 0; j < replayer.operation_count(); ++j) {
        spdlog::info("Operation {}", j);
        // prepare for update
        points_base->before_hook(m, face_tuples);
        size_t new_index = replayer.play(1);
        // update the active faces and call the after hook
        // ( in the future this will be embedded into the play functionality )
        face_tuples = m.get_faces();
        points_base->after_hook(m, face_tuples);
        for (const auto& f : face_tuples) {
            auto tri_vids = m.oriented_tri_vids(f);
            spdlog::info("{}", fmt::join(tri_vids, ","));
        }
        REQUIRE(new_index == j + 1);
        spdlog::info("Done Operation {}", j);
    }

    check_face_equality(m, final_mesh);
}

TEST_CASE("dynamic_boundary_updates", "[test_2d_operation]")
{
    // TODO: fix this test
    return;
    ExecutePass<TriMesh, ExecutionPolicy::kSeq> scheduler;


    // the mesh that will eventaully become the resulting mesh we hope to replay
    SimpleMesh final_mesh;
    make_diamond_mesh(final_mesh);


    // We will simultaneously track operations and run them to validate the logger

    {
        using namespace HighFive;
        spdlog::info("Creating replay file");
        File file("replay_operations_2d.hd5", File::ReadWrite | File::Create | File::Truncate);
        TriMeshOperationLogger op_logger(final_mesh, file);

        std::vector<std::pair<std::string, TriMesh::Tuple>> recorded_operations;

        AttributeCollectionRecorder vert_recorder(file, "vertices", final_mesh.vertices);

        // AttributeCollectionRecorder tri_recorder(file, "triangles",
        //         final_mesh.m_tri_connectivity);

        op_logger.add_attribute_recorder("vertices", vert_recorder);
        // op_recorder.add_attribute_recorder("triangles", tri_recorder);
        //  record do a flip, split, and then collapse
        SECTION("logging_operations")
        {
            std::vector<std::pair<std::string, TriMesh::Tuple>> operations;
            // collapse 4 into 2
            TriMesh::Tuple edge(0, 2, 0, final_mesh);
            REQUIRE(edge.is_valid(final_mesh));
            check_vertex_indices(final_mesh, edge, {0, 1});


            REQUIRE(edge.is_valid(final_mesh));
            std::string op_name = "edge_collapse";
            spdlog::info("Performing {}", op_name);
            for(auto& [k,v]: scheduler.edit_operation_maps) {
                spdlog::info("{}", k);
            }
            auto ret = scheduler.edit_operation_maps[op_name](final_mesh, edge);
            REQUIRE(ret.has_value());
            operations.emplace_back(op_name, edge);
            {
                // 0-----2
                // |\    |
                // | \   |
                // |  \  |
                // |   \ |
                // |    \|
                // 3-----1
                auto face_tuples = final_mesh.get_faces();
                for (auto& f : face_tuples) {
                    auto tri_vids = final_mesh.oriented_tri_vids(f);
                    spdlog::info("{} => {}", f.info(), fmt::join(tri_vids, ","));
                }
                REQUIRE(face_tuples.size() == 1);
                check_vertex_indices(final_mesh, face_tuples[0], {1, 2, 3});
            }
        }
    }
}
TEST_CASE("all_three_operation_replay", "[test_2d_operation]")
{
    return;
    ExecutePass<TriMesh, ExecutionPolicy::kSeq> scheduler;


    // the mesh that will eventaully become the resulting mesh we hope to replay
    SimpleMesh final_mesh;
    make_diamond_mesh(final_mesh);


    // We will simultaneously track operations and run them to validate the logger

    {
        using namespace HighFive;
        spdlog::info("Creating replay file");
        File file("replay_operations_2d.hd5", File::ReadWrite | File::Create | File::Truncate);
        TriMeshOperationLogger op_logger(final_mesh, file);

        std::vector<std::pair<std::string, TriMesh::Tuple>> recorded_operations;

        AttributeCollectionRecorder vert_recorder(file, "vertices", final_mesh.vertices);

        // AttributeCollectionRecorder tri_recorder(file, "triangles",
        //         final_mesh.m_tri_connectivity);

        op_logger.add_attribute_recorder("vertices", vert_recorder);
        // op_recorder.add_attribute_recorder("triangles", tri_recorder);
        //  record do a flip, split, and then collapse
        SECTION("logging_operations")
        {
            std::vector<std::pair<std::string, TriMesh::Tuple>> operations;
            TriMesh::Tuple edge(0, 2, 0, final_mesh);


            REQUIRE(edge.is_valid(final_mesh));
            std::string op_name = "edge_swap";
            spdlog::info("Performing {}", op_name);
            scheduler.edit_operation_maps[op_name](final_mesh, edge);
            operations.emplace_back(op_name, edge);
            {
                // 0-----2
                // |    /|
                // |   / |
                // |  /  |
                // | /   |
                // |/    |
                // 3-----1
                auto face_tuples = final_mesh.get_faces();
                REQUIRE(face_tuples.size() == 2);
                check_vertex_indices(final_mesh, face_tuples[0], {0, 2, 3});
                check_vertex_indices(final_mesh, face_tuples[1], {1, 2, 3});
            }


            // flip the 2,3 edge
            edge = TriMesh::Tuple(2, 0, 0, final_mesh);
            REQUIRE(edge.is_valid(final_mesh));
            check_vertex_indices(final_mesh, edge, {2, 3});

            op_name = "edge_split";
            spdlog::info("Performing {}", op_name);
            scheduler.edit_operation_maps[op_name](final_mesh, edge);
            operations.emplace_back(op_name, edge);
            {
                // 0-----2
                // |\   /|
                // | \ / |
                // |  4  |
                // | / \ |
                // |/   \|
                // 3-----1
                auto face_tuples = final_mesh.get_faces();
                REQUIRE(face_tuples.size() == 4);
                check_vertex_indices(final_mesh, face_tuples[0], {0, 4, 2});
                check_vertex_indices(final_mesh, face_tuples[1], {4, 1, 2});
                check_vertex_indices(final_mesh, face_tuples[2], {0, 3, 4});
                check_vertex_indices(final_mesh, face_tuples[3], {3, 1, 4});
            }


            // collapse 4 into 2
            edge = TriMesh::Tuple(2, 0, 0, final_mesh);
            REQUIRE(edge.is_valid(final_mesh));
            check_vertex_indices(final_mesh, edge, {2, 4});

            op_name = "edge_collapse";
            spdlog::info("Performing {}", op_name);
            scheduler.edit_operation_maps[op_name](final_mesh, edge);
            operations.emplace_back(op_name, edge);
            {
                // 0      .
                // |\     .
                // | \    .
                // |  5   .
                // | / \  .
                // |/   \ .
                // 3-----1
                // (dots are to remove multi-line comment warnings)
                auto face_tuples = final_mesh.get_faces();
                REQUIRE(face_tuples.size() == 2);
                check_vertex_indices(final_mesh, face_tuples[0], {0, 3, 5});
                check_vertex_indices(final_mesh, face_tuples[1], {1, 3, 5});
            }
            REQUIRE(operations.size() == 3);
        }
        REQUIRE(op_logger.operation_count() == 3);
    }


    /*

    for (std::string line; std::getline(output, line);) {
        nlohmann::json js = nlohmann::json::parse(line);
        //std::cout << "JSON:" << js << std::endl;

        auto& [op_name, tup] = recorded_operations.emplace_back();
        const auto& arr = js["tuple"];
        REQUIRE(arr.size() == 3);
        tup = TriMesh::Tuple(arr[0], arr[1], arr[2], final_mesh);
        op_name = js["operation"];
    }
    REQUIRE(operations.size() == recorded_operations.size());
    REQUIRE(3 == recorded_operations.size());

    for (size_t j = 0; j < operations.size(); ++j) {
        const auto& [op, tup] = operations[j];
        const auto& [rec_op, rec_tup] = recorded_operations[j];

        CHECK(op == rec_op);
        auto [vid, eid, fid, hash] = tup.as_stl_tuple();
        auto [rec_vid, rec_eid, rec_fid, rec_hash] = rec_tup.as_stl_tuple();
        CHECK(vid == rec_vid);
        CHECK(eid == rec_eid);
        CHECK(fid == rec_fid);
    }
    */

    // SECTION("replay the old operations");
    {
        using namespace HighFive;
        File file("replay_operations_2d.hd5", File::ReadOnly);
        spdlog::info("{}", file.getDataSet("operations").getElementCount());
        SimpleMesh m;
        make_diamond_mesh(m);

        TransportablePoints<TestVec2> points;
        TransportablePointsBase* points_base = &points;
        points.barycentric_interp_callback = [&](const TriMesh& m,
                                                 const TriMesh::Tuple& t,
                                                 const std::array<double, 3>& B) -> TestVec2 {
            const auto& sm = dynamic_cast<const SimpleMesh&>(m);
            auto [ai, bi, ci] = m.oriented_tri_vids(t);

            const auto& a = sm.vertices.at(ai);
            const auto& b = sm.vertices.at(bi);
            const auto& c = sm.vertices.at(ci);
            spdlog::warn("barycentric interp {} => {} {} {}", fmt::join(B, ","), a, b, c);
            return TestVec2::weighted_add(a, b, c, B);
        };
        points.point_in_triangle_callback =
            [&](const TriMesh& m, const TriMesh::Tuple& t, const TestVec2& point) -> bool {
            const auto& sm = dynamic_cast<const SimpleMesh&>(m);
            return sm.point_in_triangle(m.oriented_tri_vids(t), point);
        };
        points.get_barycentric_callback = [&](const TriMesh& m,
                                              const TriMesh::Tuple& t,
                                              const TestVec2& point) -> std::array<double, 3> {
            const auto& sm = dynamic_cast<const SimpleMesh&>(m);
            return sm.barycentric_coords(m.oriented_tri_vids(t), point);
        };

        std::vector<TestVec2> points_raw;
        points_raw.resize(10);
        Eigen::AlignedBox<double, 2> bbox(Eigen::Vector2d::Zero(), Eigen::Vector2d::Ones());
        for (auto& v : points_raw) {
            auto p = bbox.sample();
            v.x = p.x();
            v.y = p.y();
        }
        points.set_points(points_raw);

        // initialize remembered face tuple state
        std::vector<TriMesh::Tuple> face_tuples = m.get_faces();

        m.p_vertex_attrs = &m.vertices;

        spdlog::info("Input Points:: {}", fmt::join(points.points_global, ","));
        points.update_local_coordinates(m);
        TriMeshOperationLogger logger(m, file);
        logger.set_readonly();

        // attach the vertices so they'll update properly
        AttributeCollectionRecorder vert_recorder(file, "vertices", m.vertices);
        logger.add_attribute_recorder("vertices", vert_recorder);

        REQUIRE(logger.operation_count() == 3);
        OperationReplayer replayer(m, logger);
        {
            // go forward
            for (size_t j = 0; j < replayer.operation_count(); ++j) {
                spdlog::info("Operation {}", j);
                // prepare for update
                points_base->before_hook(m, face_tuples);
                size_t new_index = replayer.play(1);
                // update the active faces and call the after hook
                // ( in the future this will be embedded into the play functionality )
                face_tuples = m.get_faces();
                points_base->after_hook(m, face_tuples);
                for (const auto& f : face_tuples) {
                    auto tri_vids = m.oriented_tri_vids(f);
                    spdlog::info("{}", fmt::join(tri_vids, ","));
                }
                REQUIRE(new_index == j + 1);
            }

            check_face_equality(m, final_mesh);
        }
    }
}

TEST_CASE("replay_operations_repeatedly", "[test_2d_operation]")
{
    SimpleMesh final_mesh;
    make_diamond_mesh(final_mesh);
    // if (false) {
    //     for (int k = 0; k < 100; ++k) {
    //         for (size_t j = 0; j < replayer.operation_count(); ++j) {
    //            // prepare for update
    //            points_base->before_hook(m, face_tuples);
    //            size_t new_index = replayer.play(1);
    //            // update the active faces and call the after hook
    //            // ( in the future this will be embedded into the play functionality )
    //            face_tuples = m.get_faces();
    //            points_base->after_hook(m, face_tuples);
    //        }
    //        for (size_t j = 0; j < replayer.operation_count(); ++j) {
    //            // prepare for update
    //            points_base->before_hook(m, face_tuples);
    //            size_t new_index = replayer.play(-1);
    //            // update the active faces and call the after hook
    //            // ( in the future this will be embedded into the play functionality )
    //            face_tuples = m.get_faces();
    //            points_base->after_hook(m, face_tuples);
    //        }
    //    }
    //}
}
