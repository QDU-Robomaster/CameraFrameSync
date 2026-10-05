find_package(Threads REQUIRED)

add_executable(camera_frame_sync_parts_test parts_test.cpp)
add_executable(camera_frame_sync_test camera_frame_sync_test.cpp)

foreach(test camera_frame_sync_parts_test camera_frame_sync_test)
  target_include_directories(${test} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
  target_link_libraries(${test} PRIVATE xr Threads::Threads)
  target_compile_features(${test} PRIVATE cxx_std_20)
  add_test(NAME ${test} COMMAND ${test})
endforeach()
set_tests_properties(camera_frame_sync_test PROPERTIES TIMEOUT 30)
