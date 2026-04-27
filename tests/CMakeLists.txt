cmake_minimum_required(VERSION 3.16)
project(CameraFrameSyncSequenceTests LANGUAGES CXX)

add_executable(camera_frame_sync_sequence_test
  sequence_test.cpp
)

target_include_directories(camera_frame_sync_sequence_test PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/..
)

target_compile_features(camera_frame_sync_sequence_test PRIVATE cxx_std_20)
