#include <catch2/catch_test_macros.hpp>

#include <numeric>
#include <wmtk/Accessor.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/TriMeshOperation.hpp>
#include <wmtk/utils/Logger.hpp>


using namespace wmtk;

using TM = TriMesh;
using TMOP = TriMesh::TriMeshOperationState;
using MapResult = typename Eigen::Matrix<long, Eigen::Dynamic, 1>::MapType;
class DEBUG_TriMesh : public TriMesh
{
public:
    auto edge_tuple_between_v1_v2(const long v1, const long v2, const long fid) const -> Tuple
    {
        ConstAccessor<long> fv = create_accessor<long>(m_fv_handle);
        auto fv_base = create_base_accessor<long>(m_fv_handle);
        Tuple face = face_tuple_from_id(fid);
        auto fv0 = fv.vector_attribute(face);
        REQUIRE(fv0 == fv_base.vector_attribute(fid));
        long local_vid1 = -1, local_vid2 = -1;
        for (long i = 0; i < fv0.size(); ++i) {
            if (fv0[i] == v1) {
                local_vid1 = i;
            }
            if (fv0[i] == v2) {
                local_vid2 = i;
            }
        }
        return Tuple(local_vid1, (3 - local_vid1 - local_vid2) % 3, -1, fid, 0);
    }

    template <typename T>
    AccessorBase<T, false> create_base_accessor(const MeshAttributeHandle<T>& handle)
    {
        return AccessorBase<T, false>(*this, handle);
    }

    template <typename T>
    AccessorBase<T, true> create_const_base_accessor(const MeshAttributeHandle<T>& handle) const
    {
        return AccessorBase<T, true>(*this, handle);
    }
    template <typename T>
    AccessorBase<T, true> create_base_accessor(const MeshAttributeHandle<T>& handle) const
    {
        return create_const_base_accessor(handle);
    }

    const MeshAttributeHandle<long>& f_handle(const PrimitiveType type) const
    {
        switch (type) {
        case PrimitiveType::Vertex: return m_fv_handle;
        case PrimitiveType::Edge: return m_fe_handle;
        case PrimitiveType::Face: return m_ff_handle;
        default: throw std::runtime_error("Invalid PrimitiveType");
        }
    }

    void reserve_attributes(PrimitiveType type, long size) {
        Mesh::reserve_attributes(type,size);
    }
};

TEST_CASE("get per face data")
{
    SECTION("single face")
    {
        DEBUG_TriMesh m;
        {
            //         0
            //        / \   
            //       2   1  \ 
            //      /  0  \  \|
            //     /       \ 
            //  1  ----0---- 2
            //
            RowVectors3l tris;
            tris.resize(1, 3);
            tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(0, 2, 0);
        REQUIRE(m._debug_id(edge, PrimitiveType::Vertex) == 0);
        REQUIRE(m._debug_id(edge, PrimitiveType::Face) == 0);
        REQUIRE(
            m._debug_id(m.switch_tuple(edge, PrimitiveType::Vertex), PrimitiveType::Vertex) == 2);
        TMOP state(m);

        TMOP::PerFaceData face_data = state.get_per_face_data(edge);
        REQUIRE(face_data.V_C_id == 1);
        REQUIRE(face_data.deleted_fid == 0);
        REQUIRE(face_data.ears.size() == 2);
        TMOP::EarGlobalIDs ear1 = face_data.ears[0];
        TMOP::EarGlobalIDs ear2 = face_data.ears[1];
        REQUIRE(ear1.fid == -1);
        REQUIRE(ear1.eid > -1);
        REQUIRE(ear2.fid == -1);
        REQUIRE(ear2.eid > -1);
    }
    SECTION("one ear")
    {
        DEBUG_TriMesh m;
        {
            //  3--1--- 0
            //   |     / \ 
            //   2 f1 /2   1
            //   |  0/ f0  \ 
            //   |  /       \ 
            //  1  ----0---- 2
            //
            RowVectors3l tris;
            tris.resize(2, 3);
            tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
            tris.row(1) = Eigen::Matrix<long, 3, 1>{3, 1, 0};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        TMOP state(m);
        TMOP::PerFaceData face_data = state.get_per_face_data(edge);
        REQUIRE(face_data.V_C_id == 0);
        REQUIRE(face_data.deleted_fid == 0);
        REQUIRE(face_data.ears.size() == 2);
        TMOP::EarGlobalIDs ear1 = face_data.ears[0];
        TMOP::EarGlobalIDs ear2 = face_data.ears[1];
        REQUIRE(ear1.fid == 1);
        REQUIRE(ear1.eid > -1);
        REQUIRE(ear2.fid == -1);
        REQUIRE(ear2.eid > -1);
    }

    SECTION("two ears")
    {
        DEBUG_TriMesh m;
        {
            //  3--1--- 0 --1- 4
            //   |     / \     |
            //   2 f1 /2 1\ f2 |
            //   |  0/ f0  \1  0
            //   |  /       \  |
            //   1  ----0----  2
            //
            RowVectors3l tris;
            tris.resize(3, 3);
            tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
            tris.row(1) = Eigen::Matrix<long, 3, 1>{3, 1, 0};
            tris.row(2) = Eigen::Matrix<long, 3, 1>{0, 2, 4};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        TMOP state(m);
        TMOP::PerFaceData face_data = state.get_per_face_data(edge);
        REQUIRE(face_data.V_C_id == 0);
        REQUIRE(face_data.deleted_fid == 0);
        REQUIRE(face_data.ears.size() == 2);
        TMOP::EarGlobalIDs ear1 = face_data.ears[0];
        TMOP::EarGlobalIDs ear2 = face_data.ears[1];
        REQUIRE(ear1.fid == 1);
        REQUIRE(ear1.eid > -1);
        REQUIRE(ear2.fid == 2);
        REQUIRE(ear2.eid > -1);
    }
}

TEST_CASE("delete simplices")
{
    // things can be marked as deleted but will still have the connectivity information
    DEBUG_TriMesh m;
    {
        //  3--1--- 0 --1- 4
        //   |     / \     |
        //   2 f1 /2 1\ f2 |
        //   |  0/ f0  \1  0
        //   |  /       \  |
        //   1  ----0----  2
        //
        RowVectors3l tris;
        tris.resize(3, 3);
        tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
        tris.row(1) = Eigen::Matrix<long, 3, 1>{3, 1, 0};
        tris.row(2) = Eigen::Matrix<long, 3, 1>{0, 2, 4};
        m.initialize(tris);
    }
    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
    std::vector<std::vector<long>> simplices_to_delete(3);
    simplices_to_delete[1] = std::vector<long>{m._debug_id(edge, PrimitiveType::Edge)};
    simplices_to_delete[2] = std::vector<long>{m._debug_id(edge, PrimitiveType::Face)};


    TMOP state(m);
    state.simplices_to_delete = simplices_to_delete;
    state.delete_simplices();
    REQUIRE(state.flag_accessors[1].scalar_attribute(edge) == 0);
    REQUIRE(state.flag_accessors[2].scalar_attribute(edge) == 0);
    REQUIRE(state.ff_accessor.vector_attribute(edge)[0] == -1);
    REQUIRE(state.ff_accessor.vector_attribute(edge)[1] == 2);
    REQUIRE(state.ff_accessor.vector_attribute(edge)[2] == 1);
    REQUIRE(state.ef_accessor.scalar_attribute(edge) == 0);
}

TEST_CASE("operation state")
{
    SECTION("single face")
    {
        DEBUG_TriMesh m;
        {
            //         0
            //        / \   
            //       2   1  \ 
            //      /  0  \  \|
            //     /       \ 
            //  1  ----0---- 2
            //
            RowVectors3l tris;
            tris.resize(1, 3);
            tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(0, 2, 0);
        REQUIRE(m._debug_id(edge, PrimitiveType::Vertex) == 0);
        REQUIRE(m._debug_id(edge, PrimitiveType::Face) == 0);
        REQUIRE(
            m._debug_id(m.switch_tuple(edge, PrimitiveType::Vertex), PrimitiveType::Vertex) == 2);
        TMOP state(m, edge);

        REQUIRE(state.flag_accessors.size() == 3);
        REQUIRE(state.end_point_vids.size() == 2);
        REQUIRE(state.end_point_vids[0] == 0);
        REQUIRE(state.end_point_vids[1] == 2);
        REQUIRE(state.E_AB_id == m._debug_id(edge, PrimitiveType::Edge));
        REQUIRE(state.FaceDatas.size() == 1);
    }
    SECTION("one ear")
    {
        DEBUG_TriMesh m;
        {
            //  3--1--- 0
            //   |     / \ 
            //   2 f1 /2   1
            //   |  0/ f0  \ 
            //   |  /       \ 
            //  1  ----0---- 2
            //
            RowVectors3l tris;
            tris.resize(2, 3);
            tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
            tris.row(1) = Eigen::Matrix<long, 3, 1>{3, 1, 0};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        TMOP state(m, edge);

        REQUIRE(state.flag_accessors.size() == 3);
        REQUIRE(state.end_point_vids.size() == 2);
        REQUIRE(state.end_point_vids[0] == 1);
        REQUIRE(state.end_point_vids[1] == 2);
        REQUIRE(state.E_AB_id == m._debug_id(edge, PrimitiveType::Edge));
        REQUIRE(state.FaceDatas.size() == 1);

        REQUIRE(state.FaceDatas[0].ears.size() == 2);
    }
    SECTION("interior edge")
    {
        DEBUG_TriMesh m;
        {
            //  3--1--- 0
            //   |     / \ 
            //   2 f1 /2   1
            //   |  0/ f0  \ 
            //   |  /       \ 
            //  1  ----0---- 2
            //     \        /
            //      \  f2  /
            //       \    /
            //        \  /
            //         4
            RowVectors3l tris;
            tris.resize(3, 3);
            tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
            tris.row(1) = Eigen::Matrix<long, 3, 1>{3, 1, 0};
            tris.row(2) = Eigen::Matrix<long, 3, 1>{1, 4, 2};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        TMOP state(m, edge);

        REQUIRE(state.flag_accessors.size() == 3);
        REQUIRE(state.end_point_vids.size() == 2);
        REQUIRE(state.end_point_vids[0] == 1);
        REQUIRE(state.end_point_vids[1] == 2);
        REQUIRE(state.E_AB_id == m._debug_id(edge, PrimitiveType::Edge));
        REQUIRE(state.FaceDatas.size() == 2);

        REQUIRE(state.FaceDatas[0].V_C_id == 0);
        REQUIRE(state.FaceDatas[0].deleted_fid == 0);
        REQUIRE(state.FaceDatas[0].ears.size() == 2);
        TMOP::EarGlobalIDs ear1 = state.FaceDatas[0].ears[0];
        TMOP::EarGlobalIDs ear2 = state.FaceDatas[0].ears[1];
        REQUIRE(ear1.fid == 1);
        REQUIRE(ear1.eid > -1);
        REQUIRE(ear2.fid == -1);
        REQUIRE(ear2.eid > -1);

        REQUIRE(state.FaceDatas[1].V_C_id == 4);
        REQUIRE(state.FaceDatas[1].deleted_fid == 2);
        REQUIRE(state.FaceDatas[1].ears.size() == 2);
        ear1 = state.FaceDatas[1].ears[0];
        ear2 = state.FaceDatas[1].ears[1];
        REQUIRE(ear1.fid == -1);
        REQUIRE(ear1.eid > -1);
        REQUIRE(ear2.fid == -1);
        REQUIRE(ear2.eid > -1);
    }
}
TEST_CASE("glue ear to face") {}
TEST_CASE("hash update") {}

//////////// SPLIT TESTS ////////////
TEST_CASE("glue new faces across AB")
{
    // test the assumption of correct orientation
    // new face correspondance accross AB
    SECTION("single face")
    {
        // when the edge is on the boundary (indcated by FaceDatas size), there is no glue
        // across AB
        DEBUG_TriMesh m;
        {
            //         0
            //        / \   
            //       2   1  \ 
            //      /  0  \  \|
            //     /       \ 
            //  1  ----0---- 2
            //
            RowVectors3l tris;
            tris.resize(1, 3);
            tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(0, 2, 0);
        REQUIRE(m._debug_id(edge, PrimitiveType::Vertex) == 0);
        REQUIRE(m._debug_id(edge, PrimitiveType::Face) == 0);
        REQUIRE(
            m._debug_id(m.switch_tuple(edge, PrimitiveType::Vertex), PrimitiveType::Vertex) == 2);
        TMOP state(m, edge);
        REQUIRE(state.FaceDatas.size() == 1);
    }
    SECTION("interior edge")
    {
        DEBUG_TriMesh m;
        {
            //  3--1--- 0
            //   |     / \ 
            //   2 f1 /2   1
            //   |  0/ f0  \ 
            //   |  /  0    \ 
            //  1  --------- 2
            //     \   1    /
            //      2  f2  0
            //       \    /
            //        \  /
            //         4
            RowVectors3l tris;
            tris.resize(3, 3);
            tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
            tris.row(1) = Eigen::Matrix<long, 3, 1>{3, 1, 0};
            tris.row(2) = Eigen::Matrix<long, 3, 1>{1, 4, 2};
            m.initialize(tris);
        }
        m.reserve_attributes(PrimitiveType::Face, 10);
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        TMOP state(m, edge);

        REQUIRE(state.FaceDatas.size() == 2);

        auto new_fids = state.request_simplex_indices(PrimitiveType::Face, 4);
        std::array<long, 2> new_fids_top = {new_fids[0], new_fids[1]};
        std::array<long, 2> new_fids_bottom = {new_fids[2], new_fids[3]};
        state.glue_new_faces_across_AB(new_fids_top, new_fids_bottom);

        wmtk::logger().info("2");

        long local_eid_top = 0;
        long local_eid_bottom = 1;
        wmtk::logger().info("3");

        auto ff_accessor = m.create_base_accessor<long>(m.f_handle(PrimitiveType::Face));
        wmtk::logger().info("4");

        REQUIRE(ff_accessor.vector_attribute(new_fids_top[0])[local_eid_top] == new_fids_bottom[0]);
        wmtk::logger().info("5");
        REQUIRE(ff_accessor.vector_attribute(new_fids_top[1])[local_eid_top] == new_fids_bottom[1]);
        wmtk::logger().info("6");
        REQUIRE(
            ff_accessor.vector_attribute(new_fids_bottom[0])[local_eid_bottom] == new_fids_top[0]);
        wmtk::logger().info("7");
        REQUIRE(
            ff_accessor.vector_attribute(new_fids_bottom[1])[local_eid_bottom] == new_fids_top[1]);
        wmtk::logger().info("8");
    }
}

TEST_CASE("glue new triangle topology")
{
    // old faces are not recycled
}

TEST_CASE("simplices to delete for split") {}

//////////// COLLAPSE TESTS ////////////
TEST_CASE("2D link condition for collapse") {}
