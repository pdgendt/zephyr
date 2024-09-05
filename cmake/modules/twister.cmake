# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2024, Basalte bv

include(extensions)

function(zephyr_load_twister json_file)
  file(READ ${json_file} testplan_json)

  # We specify error variable to avoid a fatal error.
  # Searching for a non-existing key should just result in the output value '-NOTFOUND'
  string(
    JSON json_value
    ERROR_VARIABLE json_error
    GET "${testplan_json}" "environment" "zephyr_version"
  )
  set(TWISTER TRUE PARENT_SCOPE)
  set(TWISTER_TESTPLAN_ZEPHYR_VERSION ${json_value} PARENT_SCOPE)

  if(NOT EXISTS ${CMAKE_BINARY_DIR}/run_id.txt)
    # No testsuite run ID available
    return()
  endif()

  file(READ ${CMAKE_BINARY_DIR}/run_id.txt run_id)

  string(
    JSON testsuites_len
    ERROR_VARIABLE json_error
    LENGTH "${testplan_json}" "testsuites"
  )

  if(json_error OR ${testsuites_len} EQUAL 0)
    return()
  endif()

  math(EXPR testsuites_len "${testsuites_len} - 1")
  foreach(i RANGE ${testsuites_len})
    string(
      JSON testsuite_json
      ERROR_VARIABLE json_error
      GET "${testplan_json}" "testsuites" ${i}
    )
    string(
      JSON json_value
      ERROR_VARIABLE json_error
      GET "${testsuite_json}" "run_id"
    )
    if(NOT "${run_id}" STREQUAL "${json_value}")
      continue()
    endif()

    message(STATUS "Found twister testsuite ${run_id}")
    set(TWISTER_TESTSUITE_RUN_ID ${run_id} PARENT_SCOPE)

    string(
      JSON json_value
      ERROR_VARIABLE json_error
      GET "${testsuite_json}" "name"
    )
    set(TWISTER_TESTSUITE_NAME ${json_value} PARENT_SCOPE)
    string(
      JSON json_value
      ERROR_VARIABLE json_error
      GET "${testsuite_json}" "platform"
    )
    set(TWISTER_TESTSUITE_PLATFORM ${json_value} PARENT_SCOPE)
    return()
  endforeach()
endfunction()

zephyr_get(TWISTER_OUTPUT_DIR)

if(NOT TWISTER_OUTPUT_DIR OR NOT EXISTS ${TWISTER_OUTPUT_DIR}/testplan.json)
  # Not running in a twister context
  return()
endif()

message(STATUS "Found twister testplan: ${TWISTER_OUTPUT_DIR}/testplan.json")

zephyr_load_twister(${TWISTER_OUTPUT_DIR}/testplan.json)
