function(register_jse_json)
 set(options )
 set(oneValueArgs APPLICATION_NAME INPUT)# OUTPUT_PREFIX)
 set(multiValueArgs DIRECTORIES)
  cmake_parse_arguments(register_jse_json "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

  if(NOT EXPANDED_JS_OUTPUT)
      set(EXPANDED_JS_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/spec_expanded.json")
  endif()
  if(NOT OUTPUT_PREFIX)
      set(OUTPUT_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/spec")
  endif()

  set(WORKING_DIR ${CMAKE_CURRENT_SOURCE_DIR})

  foreach(DIR ${register_jse_json_DIRECTORIES} ${WORKING_DIR})

      file(GLOB_RECURSE JS_IN_DIR "${DIR}/*.js" "${DIR}/*.json")
      list(APPEND JS_IN_DIRS ${JS_IN_DIR})
  endforeach()


  MESSAGE(STATUS jse_app "in ${register_jse_json_INPUT} js_out ${EXPANDED_JS_OUTPUT} dirs ${register_jse_json_DIRECTORIES}")

  message(STATUS "JSE OUTPUT FILE ${EXPANDED_JS_OUTPUT}")
  MESSAGE(STATUS "Working dir: ${WORKING_DIR}")
  add_custom_target("wmtk_application_${register_jse_json_APPLICATION_NAME}_jse"
      COMMAND jse_app "${register_jse_json_INPUT}" "${EXPANDED_JS_OUTPUT}" ${register_jse_json_DIRECTORIES} "${WORKING_DIR}"
      WORKING_DIRECTORY ${WORKING_DIR}
      BYPRODUCTS "${EXPANDED_JS_OUTPUT}"
      SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/${register_jse_json_INPUT}" ${JS_IN_DIRS})

  message(STATUS "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/spec.hpp.in")
  add_custom_target("wmtk_application_${register_jse_json_APPLICATION_NAME}_spec_header"
      #COMMAND 'echo "#pragma once\n namespace wmtk::applications::${register_jse_json_APPLICATION_NAME}\n \{ \n \n \}"> "${OUTPUT_PREFIX}"'
      COMMAND python3 ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/generate_spec.py "${register_jse_json_APPLICATION_NAME}" "${EXPANDED_JS_OUTPUT}" "${OUTPUT_PREFIX}"
      DEPENDS "wmtk_application_${register_jse_json_APPLICATION_NAME}_jse" 
      WORKING_DIRECTORY ${WORKING_DIR}
      BYPRODUCTS "${OUTPUT_PREFIX}.cpp" "${OUTPUT_PREFIX}.hpp"
      SOURCES "${register_jse_json_INPUT}" ${JS_IN_DIRS} ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/generate_spec.py
  )


  list(APPEND WMTK_APPLICATION_HEADER_TARGETS "wmtk_application_${register_jse_json_APPLICATION_NAME}_spec_header")

  set(WMTK_APPLICATION_HEADER_TARGETS PARENT_SCOPE ${WMTK_APPLICATION_HEADER_TARGETS})

endfunction()
