find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

file(GLOB ROAH_RSBB_MSGS_PROTOS ${CMAKE_CURRENT_SOURCE_DIR}/../../proto/*.proto)
protobuf_generate_cpp(RSBB_PROTO_SRCS RSBB_PROTO_HDRS ${ROAH_RSBB_MSGS_PROTOS})
add_library(${DISAMBIGUATION}roah_rsbb_msgs STATIC ${RSBB_PROTO_SRCS})
target_link_libraries(${DISAMBIGUATION}roah_rsbb_msgs ${PROTOBUF_LIBRARIES})

set(ROAH_RSBB_MSGS_HPP "${CMAKE_CURRENT_BINARY_DIR}/../roah_rsbb_msgs.h")
file(REMOVE ${ROAH_RSBB_MSGS_HPP})
foreach(PB_H ${RSBB_PROTO_HDRS})
  get_filename_component(FILE_PB_H ${PB_H} NAME)
  file(APPEND ${ROAH_RSBB_MSGS_HPP} "#include \"roah_rsbb_msgs/${FILE_PB_H}\"\n")
endforeach()

include_directories(${CMAKE_CURRENT_BINARY_DIR}/../)

set(INCLUDE_DIRS ${INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/../ PARENT_SCOPE)
