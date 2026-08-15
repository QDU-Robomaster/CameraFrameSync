cmake_minimum_required(VERSION 3.16)
project(CameraFrameSyncSequenceTests LANGUAGES CXX)

include(CTest)

add_executable(camera_frame_sync_sequence_test sequence_test.cpp)
add_executable(camera_frame_sync_frame_queue_test frame_queue_test.cpp)

if(TARGET xr)
  find_package(Threads REQUIRED)
  add_executable(camera_frame_sync_profile_switch_test profile_switch_test.cpp)
  target_link_libraries(camera_frame_sync_profile_switch_test
                        PRIVATE xr Threads::Threads)
  target_compile_features(camera_frame_sync_profile_switch_test
                          PRIVATE cxx_std_20)
endif()

target_include_directories(camera_frame_sync_sequence_test
                           PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)

target_compile_features(camera_frame_sync_sequence_test PRIVATE cxx_std_20)
target_compile_features(camera_frame_sync_frame_queue_test PRIVATE cxx_std_20)

if(MSVC)
  target_compile_options(camera_frame_sync_sequence_test PRIVATE /utf-8)
  target_compile_options(camera_frame_sync_frame_queue_test PRIVATE /utf-8)
  if(TARGET camera_frame_sync_profile_switch_test)
    target_compile_options(camera_frame_sync_profile_switch_test PRIVATE /utf-8)
  endif()
endif()

add_test(NAME camera_frame_sync_sequence_test
         COMMAND camera_frame_sync_sequence_test)
add_test(NAME camera_frame_sync_frame_queue_test
         COMMAND camera_frame_sync_frame_queue_test)

if(TARGET camera_frame_sync_profile_switch_test)
  foreach(
    _CASE
    startup_first_trigger
    trigger_stride
    invalid_gap_resync
    trigger_stream_resync
    profile_switch_order
    single_profile
    start_window
    ack_filter
    malformed_stop_ack
    malformed_start_ack
    retry
    retry_exhausted
    switch_failure
    ownership
    clock_domains
    latest_mode
    legacy_runtime_rejection
    deferred_dispatch_retry)
    add_test(NAME camera_frame_sync_profile_switch_${_CASE}
             COMMAND camera_frame_sync_profile_switch_test ${_CASE})
    set_tests_properties(camera_frame_sync_profile_switch_${_CASE}
                         PROPERTIES TIMEOUT 5)
  endforeach()
endif()
