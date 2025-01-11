function(wmtk_generate_test_config CFG)
    set(FILE_CONTENTS "{\n")
    foreach(TEST_CONFIG ${CFG})
        string(APPEND FILE_CONTENTS "${TEST_CONFIG},\n")
    endforeach()

    string(APPEND FILE_CONTENTS "\"skip\":{}}\n")

    FILE(WRITE ${CMAKE_BINARY_DIR}/test_config.json "${FILE_CONTENTS}")
endfunction()


