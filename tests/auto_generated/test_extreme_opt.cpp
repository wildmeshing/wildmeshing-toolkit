
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


    TEST_CASE("test_model_chair","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/chair_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/chair_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_elephant","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/elephant_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/elephant_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_neptune0","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/neptune0_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/neptune0_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_rocker_arm","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/rocker_arm_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/rocker_arm_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_botijo","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/botijo_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/botijo_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_cup","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/cup_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/cup_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_elk","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/elk_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/elk_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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
        std::string input_cut_file = "../../extreme_opt_data/helmet_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/helmet_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_femur","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/femur_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/femur_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_holes3","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/holes3_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/holes3_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_sculpt","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/sculpt_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/sculpt_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_bumpy_torus","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/bumpy_torus_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/bumpy_torus_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_dancer2","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/dancer2_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/dancer2_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_fertility_tri","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/fertility_tri_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/fertility_tri_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_thai_statue","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/thai_statue_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/thai_statue_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_camel","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/camel_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/camel_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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
        std::string input_cut_file = "../../extreme_opt_data/eight_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/eight_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_bozbezbozzel100K","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/bozbezbozzel100K_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/bozbezbozzel100K_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_carter100K","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/carter100K_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/carter100K_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_chair100K","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/chair100K_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/chair100K_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_dancer_25k","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/dancer_25k_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/dancer_25k_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_dancing_children100K","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/dancing_children100K_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/dancing_children100K_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_dragonstand_recon100K","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/dragonstand_recon100K_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/dragonstand_recon100K_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_genus3","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/genus3_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/genus3_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_kitten100K","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/kitten100K_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/kitten100K_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_knot100K","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/knot100K_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/knot100K_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_master_cylinder100K","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/master_cylinder100K_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/master_cylinder100K_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_rolling_stage100K","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/rolling_stage100K_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/rolling_stage100K_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_wrench50K","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/wrench50K_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/wrench50K_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_pulley100K","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/pulley100K_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/pulley100K_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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


    TEST_CASE("test_model_pegaso","[multimesh][autogen][2D]")
    {
        std::string input_cut_file = "../../extreme_opt_data/pegaso_init.obj";
        std::string input_seamed_file = "../../extreme_opt_data/pegaso_seamed.obj";
        
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
        for (auto i = 0; i < 10; i++)
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

        // test collapse
        for (auto i = 0; i < 10; i++)
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

