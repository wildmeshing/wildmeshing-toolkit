
#include <wmtk/serialization/AttributeCollectionRecorder.h>
#include <wmtk/serialization/AttributeCollectionReplayer.h>
#include <wmtk/AttributeCollection.hpp>

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


#include <catch2/catch_test_macros.hpp>
#include <iostream>

using namespace wmtk;


TEST_CASE("double_recorder", "[attribute_recording]")
{
    using namespace HighFive;
    File file("double_recorder.hd5", File::ReadWrite | File::Create | File::Truncate);


    AttributeCollection<double> attribute_collection;
    attribute_collection.resize(20);

    for (size_t j = 0; j < attribute_collection.size(); ++j) {
        attribute_collection[j] = -double(j);
    }

    AttributeCollectionRecorder attribute_recorder(file, "attribute1", attribute_collection);
    size_t update_index = 0;
    {
        AttributeCollectionUpdate update = attribute_recorder.update(update_index);
        CHECK(update.range.begin == 0);
        CHECK(update.range.end == 20);
        CHECK(update.new_size == 20);
    }
    spdlog::info("State: {}", fmt::join(attribute_collection, ","));

    attribute_collection.begin_protect();


    for (size_t j = 0; j < attribute_collection.size(); ++j) {
        attribute_collection[j] = double(2 * j);
    }

    for (size_t j = 0; j < attribute_collection.size(); ++j) {
        CHECK(attribute_collection[j] == 2 * j);
    }

    const auto& rollback_list = attribute_collection.m_rollback_list.local();


    REQUIRE(rollback_list.size() <= attribute_collection.size());

    std::optional<size_t> update_index_opt;
    update_index_opt = attribute_collection.end_protect();
    REQUIRE(update_index_opt.has_value());
    update_index = update_index_opt.value();

    spdlog::info("State {}: {}", update_index, fmt::join(attribute_collection, ","));

    {
        AttributeCollectionUpdate update = attribute_recorder.update(update_index);
        CHECK(update.range.begin == 20);
        CHECK(update.range.end == 40);
        CHECK(update.new_size == 20);
        CHECK(update.old_size == 20);
    }

    attribute_collection.begin_protect();
    for (size_t j = 0; j < attribute_collection.size(); j += 2) {
        attribute_collection[j] = double(3 * j);
    }
    update_index_opt = attribute_collection.end_protect();
    REQUIRE(update_index_opt.has_value());
    update_index = update_index_opt.value();
    spdlog::info("State {}: {}", update_index, fmt::join(attribute_collection, ","));
    {
        REQUIRE(update_index == attribute_recorder.updates_size() - 1);
        AttributeCollectionUpdate update = attribute_recorder.update(update_index);
        CHECK(update.range.begin == 20 + 20);
        CHECK(update.range.end == (update.range.begin + update.new_size / 2));
        CHECK(update.new_size == attribute_collection.size());
        CHECK(update.old_size == attribute_collection.size());
    }


    for (size_t j = 0; j < attribute_collection.size(); ++j) {
        if (j % 2 == 0) {
            REQUIRE(3 * j == attribute_collection.at(j));
        } else {
            REQUIRE(2 * j == attribute_collection.at(j));
        }
    }

    attribute_collection.begin_protect();
    attribute_collection.resize(30);
    update_index_opt = attribute_collection.end_protect();
    REQUIRE(update_index_opt.has_value());
    update_index = update_index_opt.value();
    spdlog::info("State {}: {}", update_index, fmt::join(attribute_collection, ","));
    {
        REQUIRE(update_index == attribute_recorder.updates_size() - 1);
        AttributeCollectionUpdate update = attribute_recorder.update(update_index);
        CHECK(update.range.begin == 0);
        CHECK(update.range.end == 0);
        CHECK(update.new_size == 30);
        CHECK(update.old_size == 20);
    }

    attribute_collection.begin_protect();
    attribute_collection.resize(20);
    update_index_opt = attribute_collection.end_protect();
    REQUIRE(update_index_opt.has_value());
    update_index = update_index_opt.value();
    spdlog::info("State {}: {}", update_index, fmt::join(attribute_collection, ","));
    {
        REQUIRE(update_index == attribute_recorder.updates_size() - 1);
        AttributeCollectionUpdate update = attribute_recorder.update(update_index);
        CHECK(update.range.begin == 50);
        CHECK(update.range.end == update.range.begin + 10);
        CHECK(update.new_size == 20);
        CHECK(update.old_size == 30);
    }
    const AttributeCollection<double>& old_attribute_collection = attribute_collection;
    {
        update_index = 0;
        File read_file("double_recorder.hd5", File::ReadOnly);

        AttributeCollection<double> attribute_collection;
        AttributeCollectionReplayer replayer(read_file, "attribute1", attribute_collection);

        REQUIRE(attribute_collection.size() == 20);

        for (size_t j = 0; j < attribute_collection.size(); ++j) {
            CHECK(attribute_collection[j] == -double(j));
        }
        spdlog::info("State {}: {}", update_index, fmt::join(attribute_collection, ","));

        // manually play to end
        for (int j = 0; j < replayer.updates_size(); ++j) {
            replayer.step_forward();
            update_index = replayer.current_update_index() - 1;
            spdlog::info("State {}: {}", update_index, fmt::join(attribute_collection, ","));
        }
        REQUIRE(attribute_collection.size() == old_attribute_collection.size());
        for (size_t j = 0; j < old_attribute_collection.size(); ++j) {
            CHECK(attribute_collection[j] == old_attribute_collection[j]);
        }
        // see how randomly resetting state works
        replayer.reset_to_initial_state();
        REQUIRE(attribute_collection.size() == 20);
        for (size_t j = 0; j < attribute_collection.size(); ++j) {
            CHECK(attribute_collection[j] == -double(j));
        }
        spdlog::info("Running to end");
        replayer.run_to_end();
        spdlog::info("Done running to end {}", replayer.current_update_index());
        REQUIRE(attribute_collection.size() == old_attribute_collection.size());
        for (size_t j = 0; j < old_attribute_collection.size(); ++j) {
            CHECK(attribute_collection[j] == old_attribute_collection[j]);
        }
        replayer.run_to_step(0);
        REQUIRE(attribute_collection.size() == 20);
        for (size_t j = 0; j < attribute_collection.size(); ++j) {
            CHECK(attribute_collection[j] == -double(j));
        }
    }
}

TEST_CASE("complex_recorder", "[attribute_recording]")
{
    using namespace HighFive;
    File file("complex_recorder.hd5", File::ReadWrite | File::Create | File::Truncate);


    AttributeCollection<TestComplexStruct> attribute_collection;
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
