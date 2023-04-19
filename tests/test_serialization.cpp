#include <wmtk/serialization/Eigen.h>
#include <wmtk/serialization/Macros.h>

//struct TestEigenStruct {
//    Eigen::Vector3d a;
//    boolean value;
//    Eigen::Vector2i b;
//
//    static HighFive::DataType datatype() {
//        return HighFive::CompoundType{
//            {"a", HighFive::create_datatype<decltype(a)>()},
//            {"value", HighFive::create_datatype<decltype(value)>()},
//            {"b", HighFive::create_datatype<decltype(b)>()},
//        };
//    }
//}
//
//HIGHFIVE_REGISTER_TYPE(TestEigenStruct , TestEigenStruct::datatype)
//
template <typename T, int R>
struct TestEigenStruct {
    Eigen::Matrix<T,R,1> a;

    static HighFive::DataType datatype() {
        return HighFive::CompoundType{
            {"a", HighFive::create_datatype<decltype(a)>()}
        };
    }
    TestEigenStruct() = default;
    TestEigenStruct(const TestEigenStruct&) = default;
    TestEigenStruct& operator=(const TestEigenStruct&) = default;
    TestEigenStruct(TestEigenStruct&&) = default;
    TestEigenStruct& operator=(TestEigenStruct&&) = default;
    ~TestEigenStruct() = default;
    bool operator==(const TestEigenStruct& o) const {
        return a == o.a;
    }
};

using TestEigenStruct2f = TestEigenStruct<float,2>;
//HIGHFIVE_REGISTER_TYPE( TestEigenStruct2f, TestEigenStruct2f::datatype)


#include <catch2/catch.hpp>
#include <iostream>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/serialization/AttributeCollectionSerializer.h>

using namespace wmtk;

template <typename T, int R>
void test_basic_eigen_serialization(HighFive::File& file, const std::string& name) {


    using VecType = TestEigenStruct<T,R>;
    //static_assert(std::is_trivially_copyable_v<VecType>);
    AttributeCollection<VecType> attribute_collection;
    attribute_collection.resize(5);
    for(auto& v: attribute_collection) {
        v.a.setRandom();
    }
    AttributeCollectionSerializer serializer(file,name,attribute_collection);
    serializer.serialize();

    //AttributeCollection<VecType> loaded_attribute_collection;

    //spdlog::error("Reading");;
    //AttributeCollectionSerializer deserializer(file,name,loaded_attribute_collection);
    //deserializer.deserialize();

    //REQUIRE(loaded_attribute_collection.size() == attribute_collection.size());
    //for(size_t j = 0; j < loaded_attribute_collection.size(); ++j) {
    //    CHECK(loaded_attribute_collection[j] == attribute_collection[j]);
    //}


}

TEST_CASE("basic_eigen_serialization", "[attribute_recording]")
{
    using namespace HighFive;
    File file("eigen_serialization.hd5", File::ReadWrite | File::Create | File::Truncate);




    test_basic_eigen_serialization<float,2>(file, "float2");

}
