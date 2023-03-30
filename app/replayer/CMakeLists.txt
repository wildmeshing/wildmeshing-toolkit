

add_subdirectory(lib)
#add_subdirectory(app)
if(WMTK_APP_UNIT_TESTS)
    add_subdirectory(tests)
endif()
