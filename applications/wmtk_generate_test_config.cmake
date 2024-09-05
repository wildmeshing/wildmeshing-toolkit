function(wmtk_generate_test_config WMTK_TEST_CONFIG)
    FILE(WRITE ${CMAKE_BINARY_DIR}/test_config.json "{\n")

    foreach(TEST_CONFIG ${WMTK_TEST_CONFIG})
        FILE(APPEND ${CMAKE_BINARY_DIR}/test_config.json "${TEST_CONFIG},\n")
    endforeach()

    FILE(APPEND ${CMAKE_BINARY_DIR}/test_config.json "\"skip\":{}}\n")

endfunction()
