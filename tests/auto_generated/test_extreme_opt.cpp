
    #include <catch2/catch_test_macros.hpp>
    #include <wmtk/TriMeshOperationExecutor.hpp>
    #include "../tools/DEBUG_TriMesh.hpp"
    #include "../tools/TriMesh_examples.hpp"
    #include <random>
    #include <igl/readOBJ.h>

    using namespace wmtk;
    using namespace wmtk::tests;

    using TM = TriMesh;
    using TMOE = decltype(std::declval<DEBUG_TriMesh>().get_tmoe(
        wmtk::Tuple(),
        std::declval<Accessor<long>&>()));

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Face;


    TEST_CASE("test_model_elk","[multimesh][autogen][2D]")
    {
        std::string path_to_data;
        path_to_data.append(WMTK_DATA_DIR);
        path_to_data.append("//extreme_opt_data");
        std::string input_cut_file = path_to_data + "/elk_init.obj";
        std::string input_seamed_file = path_to_data + "/elk_seamed.obj";
        
        Eigen::MatrixXd V_cut, uv_cut;
        RowVectors3l F_cut;
        igl::readOBJ(input_cut_file, V_cut, uv_cut, uv_cut, F_cut, F_cut, F_cut);
        Eigen::MatrixXd V_seamed, uv_seamed;
        RowVectors3l F_seamed;
        igl::readOBJ(input_seamed_file, V_seamed, uv_seamed, uv_seamed, F_seamed, F_seamed, F_seamed);

        DEBUG_TriMesh parent;
        parent.initialize(F_seamed);
        REQUIRE(parent.is_connectivity_valid());
        DEBUG_TriMesh child;
        child.initialize(F_cut);
        auto child_ptr = std::make_shared<DEBUG_TriMesh>(child);
        REQUIRE(child_ptr->is_connectivity_valid());
        std::vector<long> f_map(F_cut.rows());
        for (long i = 0; i < long(f_map.size()); i++)
        {
            f_map[i] = i;
        }

        MultiMeshManager::register_child_mesh(parent, child_ptr, f_map);
        REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);

        std::mt19937 rng(0);
        std::uniform_int_distribution<int> distribution(0, parent.capacity(PE) - 1);

        // test split
        std::cout << "test split" << std::endl;
        for (auto i = 0; i < 200; i++)
        {
            long random_edge_id = distribution(rng);
            std::cout << random_edge_id << std::endl;
            Tuple edge = parent.tuple_from_id(PE,random_edge_id);
            if (parent.is_edge_deleted(edge))
            {
                std::cout << "edge " << random_edge_id << " deleted" << std::endl;
                continue;
            }
            if (parent.is_valid_slow(edge) == false)
            {
                std::cout << "edge " << random_edge_id << " invalid" << std::endl;
                continue;
            }

            auto parent_hash_acc = parent.get_cell_hash_accessor();
            auto executor = parent.get_tmoe(edge, parent_hash_acc);
            executor.split_edge();
            REQUIRE(parent.is_connectivity_valid());
            REQUIRE(child_ptr->is_connectivity_valid());
            REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);
        }

        std::cout << "test collapse" << std::endl;
        // test collapse
        for (auto i = 0; i < 200; i++)
        {
            long random_edge_id = distribution(rng);
            std::cout << random_edge_id << std::endl;
            Tuple edge = parent.tuple_from_id(PE,random_edge_id);
            if (parent.is_edge_deleted(edge))
            {
                continue;
            }
            auto parent_hash_acc = parent.get_cell_hash_accessor();
            auto executor = parent.get_tmoe(edge, parent_hash_acc);
            if (executor.can_collapse() == false)
            {
                std::cout << "link cond fail, cannot collapse" << std::endl;
                continue;
            }
            executor.collapse_edge();
            REQUIRE(parent.is_connectivity_valid());
            REQUIRE(child_ptr->is_connectivity_valid());
            REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);
        }
}


    TEST_CASE("test_model_eight","[multimesh][autogen][2D]")
    {
        std::string path_to_data;
        path_to_data.append(WMTK_DATA_DIR);
        path_to_data.append("//extreme_opt_data");
        std::string input_cut_file = path_to_data + "/eight_init.obj";
        std::string input_seamed_file = path_to_data + "/eight_seamed.obj";
        
        Eigen::MatrixXd V_cut, uv_cut;
        RowVectors3l F_cut;
        igl::readOBJ(input_cut_file, V_cut, uv_cut, uv_cut, F_cut, F_cut, F_cut);
        Eigen::MatrixXd V_seamed, uv_seamed;
        RowVectors3l F_seamed;
        igl::readOBJ(input_seamed_file, V_seamed, uv_seamed, uv_seamed, F_seamed, F_seamed, F_seamed);

        DEBUG_TriMesh parent;
        parent.initialize(F_seamed);
        REQUIRE(parent.is_connectivity_valid());
        DEBUG_TriMesh child;
        child.initialize(F_cut);
        auto child_ptr = std::make_shared<DEBUG_TriMesh>(child);
        REQUIRE(child_ptr->is_connectivity_valid());
        std::vector<long> f_map(F_cut.rows());
        for (long i = 0; i < long(f_map.size()); i++)
        {
            f_map[i] = i;
        }

        MultiMeshManager::register_child_mesh(parent, child_ptr, f_map);
        REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);

        std::mt19937 rng(0);
        std::uniform_int_distribution<int> distribution(0, parent.capacity(PE) - 1);

        std::cout << "test split" << std::endl;
        // test split
        for (auto i = 0; i < 200; i++)
        {
            long random_edge_id = distribution(rng);
            std::cout << random_edge_id << std::endl;
            Tuple edge = parent.tuple_from_id(PE,random_edge_id);
            if (parent.is_edge_deleted(edge))
            {
                continue;
            }
            auto parent_hash_acc = parent.get_cell_hash_accessor();
            auto executor = parent.get_tmoe(edge, parent_hash_acc);
            executor.split_edge();
            REQUIRE(parent.is_connectivity_valid());
            REQUIRE(child_ptr->is_connectivity_valid());
            REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);
        }

        std::cout << "test collapse" << std::endl;
        // test collapse
        for (auto i = 0; i < 200; i++)
        {
            long random_edge_id = distribution(rng);
            std::cout << random_edge_id << std::endl;
            Tuple edge = parent.tuple_from_id(PE,random_edge_id);
            if (parent.is_edge_deleted(edge))
            {
                continue;
            }
            auto parent_hash_acc = parent.get_cell_hash_accessor();
            auto executor = parent.get_tmoe(edge, parent_hash_acc);
            if (executor.can_collapse() == false)
            {
                std::cout << "link cond fail, cannot collapse" << std::endl;
                continue;
            }
            executor.collapse_edge();
            REQUIRE(parent.is_connectivity_valid());
            REQUIRE(child_ptr->is_connectivity_valid());
            REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);
        }
}


    TEST_CASE("test_model_helmet","[multimesh][autogen][2D]")
    {
        std::string path_to_data;
        path_to_data.append(WMTK_DATA_DIR);
        path_to_data.append("//extreme_opt_data");
        std::string input_cut_file = path_to_data + "/helmet_init.obj";
        std::string input_seamed_file = path_to_data + "/helmet_seamed.obj";
        
        Eigen::MatrixXd V_cut, uv_cut;
        RowVectors3l F_cut;
        igl::readOBJ(input_cut_file, V_cut, uv_cut, uv_cut, F_cut, F_cut, F_cut);
        Eigen::MatrixXd V_seamed, uv_seamed;
        RowVectors3l F_seamed;
        igl::readOBJ(input_seamed_file, V_seamed, uv_seamed, uv_seamed, F_seamed, F_seamed, F_seamed);

        DEBUG_TriMesh parent;
        parent.initialize(F_seamed);
        REQUIRE(parent.is_connectivity_valid());
        DEBUG_TriMesh child;
        child.initialize(F_cut);
        auto child_ptr = std::make_shared<DEBUG_TriMesh>(child);
        REQUIRE(child_ptr->is_connectivity_valid());
        std::vector<long> f_map(F_cut.rows());
        for (long i = 0; i < long(f_map.size()); i++)
        {
            f_map[i] = i;
        }

        MultiMeshManager::register_child_mesh(parent, child_ptr, f_map);
        REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);

        std::mt19937 rng(0);
        std::uniform_int_distribution<int> distribution(0, parent.capacity(PE) - 1);

        std::cout << "test split" << std::endl;
        // test split
        for (auto i = 0; i < 200; i++)
        {
            long random_edge_id = distribution(rng);
            std::cout << random_edge_id << std::endl;
            Tuple edge = parent.tuple_from_id(PE,random_edge_id);
            if (parent.is_edge_deleted(edge))
            {
                continue;
            }
            auto parent_hash_acc = parent.get_cell_hash_accessor();
            auto executor = parent.get_tmoe(edge, parent_hash_acc);
            executor.split_edge();
            REQUIRE(parent.is_connectivity_valid());
            REQUIRE(child_ptr->is_connectivity_valid());
            REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);
        }

        std::cout << "test collapse" << std::endl;
        // test collapse
        for (auto i = 0; i < 200; i++)
        {
            long random_edge_id = distribution(rng);
            std::cout << random_edge_id << std::endl;
            Tuple edge = parent.tuple_from_id(PE,random_edge_id);
            if (parent.is_edge_deleted(edge))
            {
                continue;
            }
            auto parent_hash_acc = parent.get_cell_hash_accessor();
            auto executor = parent.get_tmoe(edge, parent_hash_acc);
            if (executor.can_collapse() == false)
            {
                std::cout << "link cond fail, cannot collapse" << std::endl;
                continue;
            }
            executor.collapse_edge();
            REQUIRE(parent.is_connectivity_valid());
            REQUIRE(child_ptr->is_connectivity_valid());
            REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);
        }
}

