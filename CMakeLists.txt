cmake_minimum_required (VERSION 2.8.7)
project (RoAH_RSBB_Comm)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

add_definitions("-Wall")
add_definitions("-std=c++0x")

add_subdirectory(gen/roah_rsbb_msgs/)
add_subdirectory(libs/)
add_subdirectory(include/)
include_directories(${INCLUDE_DIRS})

add_executable(capture_comm src/capture_comm.cpp)
target_link_libraries(capture_comm roah_rsbb_msgs protobuf_comm)
add_executable(send_hide src/send_hide.cpp)
target_link_libraries(send_hide roah_rsbb_msgs protobuf_comm)
add_executable(send_show src/send_show.cpp)
target_link_libraries(send_show roah_rsbb_msgs protobuf_comm)
