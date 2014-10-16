find_package(Boost 1.46.1 REQUIRED thread system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(OpenSSL REQUIRED)
include_directories(${OPENSSL_INCLUDE_DIR})
add_definitions("-DHAVE_LIBCRYPTO")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB PROTOBUF_COMM_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/protobuf_comm/*.cpp)
add_library(${DISAMBIGUATION}protobuf_comm STATIC ${PROTOBUF_COMM_SOURCES})
target_link_libraries(${DISAMBIGUATION}protobuf_comm ${OPENSSL_LIBRARIES} ${Boost_LIBRARIES})

set(INCLUDE_DIRS ${INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
