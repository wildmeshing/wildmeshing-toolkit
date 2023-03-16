
#include <wmtk/AttributeCollectionRecorder.h>

WMTK_HDF5_REGISTER_ATTRIBUTE_TYPE(double)

struct TestComplexStruct
{
    double value;
    size_t index;
    static HighFive::CompoundType datatype()
    {
        return HighFive::CompoundType{
            {"value", HighFive::create_datatype<double>()},
            {"index", HighFive::create_datatype<size_t>()}};
    }
};

HIGHFIVE_REGISTER_TYPE(TestComplexStruct, TestComplexStruct::datatype)
WMTK_HDF5_REGISTER_ATTRIBUTE_TYPE(TestComplexStruct)


#include <catch2/catch.hpp>
#include <iostream>

using namespace wmtk;


TEST_CASE("double_recorder", "[attribute_recording]")
{
    using namespace HighFive;
    File file("double_recorder.hd5", File::ReadWrite | File::Create | File::Truncate);


    AttributeCollection<double> attribute_collection;
    attribute_collection.resize(20);
    AttributeCollectionRecorder attribute_recorder(file, "attribute1", attribute_collection);

    for (size_t j = 0; j < attribute_collection.size(); ++j) {
        attribute_collection[j] = -double(j);
    }

    attribute_collection.begin_protect();


    for (size_t j = 0; j < attribute_collection.size(); ++j) {
        attribute_collection[j] = double(2 * j);
    }

    for (size_t j = 0; j < attribute_collection.size(); ++j) {
        REQUIRE(attribute_collection[j] == 2 * j);
    }

    const auto& rollback_list = attribute_collection.m_rollback_list.local();


    REQUIRE(rollback_list.size() <= attribute_collection.size());

    size_t update_index = attribute_recorder.record();

    attribute_collection.end_protect();
    return;

    {
        AttributeCollectionUpdate update = attribute_recorder.update(update_index);
        CHECK(update.range.begin == 0);
        CHECK(update.range.end == 20);
        CHECK(update.new_attribute_size == 20);
    }

    attribute_collection.begin_protect();
    for (size_t j = 0; j < attribute_collection.size(); j += 2) {
        attribute_collection[j] = double(3 * j);
    }
    attribute_collection.end_protect();
    {
        size_t update_index = attribute_recorder.updates_size() - 1;
        AttributeCollectionUpdate update = attribute_recorder.update(update_index);
        CHECK(update.range.begin == update.new_attribute_size);
        CHECK(update.range.end == (update.new_attribute_size + update.new_attribute_size / 2));
        CHECK(update.new_attribute_size == attribute_collection.size());
    }

    for (size_t j = 0; j < attribute_collection.size(); ++j) {
        if (j % 2 == 0) {
            REQUIRE(attribute_collection[j] == 3 * j);
        } else {
            REQUIRE(attribute_collection[j] == 2 * j);
        }
    }

    {
        // File read_file("double_recorder.hd5", File::Read);
    }
}

TEST_CASE("complex_recorder", "[attribute_recording]")
{
    using namespace HighFive;
    File file("complex_recorder.hd5", File::ReadWrite | File::Create | File::Truncate);


    AttributeCollection<double> attribute_collection;
    attribute_collection.resize(5);
    AttributeCollectionRecorder attribute_recorder(file, "attribute1", attribute_collection);


    attribute_collection.begin_protect();

    for (size_t j = 0; j < attribute_collection.size(); ++j) {
        attribute_collection[j].index = j;
    }
    attribute_collection.end_protect();

    attribute_collection.begin_protect();

    for (size_t j = 0; j < attribute_collection.size(); ++j) {
        attribute_collection[j].index = 5 - j;
    }
    attribute_collection.rollback();
    attribute_collection.end_protect();
}
