add_component_test(${COMPONENT_NAME})

set(SRC_FILES
    test_file_write.cpp
    test_insertion.cpp
    test_operation_smooth.cpp
    test_operations.cpp
    test_runner.cpp
    )

target_sources(wmtk_test_${COMPONENT_NAME} PRIVATE ${SRC_FILES})